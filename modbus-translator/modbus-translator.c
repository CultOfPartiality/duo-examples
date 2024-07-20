/*
1. Recieve command from Modbus TCP
2. Parse bytes out to serial
3. Receive serial response
4. Send bytes back to TCP
*/

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "nanomodbus.h"
#include "platform.h" 

bool terminate;
int serial_port_fd;
bool printDebug = false;

void sighandler(int s) {
  UNUSED_PARAM(s);
  terminate = true;
}

int initialise_serial(const char tty_path[], int baud, int read_timeout_ds) {
  int serial_port_fd = open(tty_path, O_RDWR);
  // Check for errors
  if (serial_port_fd < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    return -1;
  }

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if (tcgetattr(serial_port_fd, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    close(serial_port_fd);
    return -1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in
                          // communication (most common)
  tty.c_cflag &= ~CSIZE;  // Clear all bits that set the data size
  tty.c_cflag |= CS8;     // 8 bits per byte (most common)
  tty.c_cflag &=
      ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |=
      CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;   // Disable echo
  tty.c_lflag &= ~ECHOE;  // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g.
                         // newline chars)
  tty.c_oflag &=
      ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT
  // PRESENT ON LINUX) tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars
  // (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] =
      read_timeout_ds; // Wait for up to 1s (10 deciseconds), returning as soon
                       // as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    close(serial_port_fd);
    return -1;
  }
  return serial_port_fd;
}

void handle_modbus(ClientConnectionDetails *);

int main(int argc, char *argv[]) {
  signal(SIGTERM, sighandler);
  signal(SIGSTOP, sighandler);
  signal(SIGINT, sighandler);
  signal(SIGQUIT, sighandler);

  int opt;
  char ipaddress[15];
  char port[5] = "502";
  while ((opt = getopt(argc, argv, "vp:")) != -1) {
    switch (opt) {
    case 'v':
      printDebug = true;
      break;
    case 'p':
      strncpy(port, optarg, sizeof(port) / sizeof(port[0]));
      break;
    default:
      fprintf(stderr, "Usage: %s [-v] [-p Port] [ipaddress]\n", argv[0]);
      exit(EXIT_FAILURE);
    }
  }

  if (optind == argc) {
    fprintf(stderr, "Usage: %s [-v] [-p Port] <ipaddress>\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  strncpy(ipaddress, argv[optind], sizeof(ipaddress) / sizeof(ipaddress[0]));

  // Set up the TCP server
  int ret = create_tcp_server(ipaddress, port);
  if (ret != 0) {
    fprintf(stderr, "Error creating TCP server - %s\n", strerror(ret));
    return 1;
  }

  serial_port_fd = initialise_serial("/dev/ttyS4", B9600, 10);
  if (serial_port_fd < 0) {
    fprintf(stderr, "Error setting up serial connection \n");
    return 1;
  }

  printf("Modbus TCP/Serial server started\n");

  // Our server supports requests from more than one client
  while (!terminate) {
    ClientConnectionDetails *conn = server_poll_detail();
    if (conn) {
      handle_modbus(conn);
    }
  }

  // Close the TCP server
  close_tcp_server();

  // No need to destroy the nmbs instance, bye bye
  return 0;
}

void printBinaryData(uint8_t *buf, int length) {
  for (int i = 0; i < length; i++) {
    if (i > 0)
      printf(":");
    printf("%02X", buf[i]);
  }
  printf("\n");
}

void handle_modbus(ClientConnectionDetails *conn) {

  // Buffers
  uint8_t tcp_in_buf[60];
  uint8_t serial_out_buf[60];
  uint8_t serial_in_buf[256];
  uint8_t tcp_out_buf[256];

  // Get data from TCP
  int tcp_in_total = read_fd_linux(tcp_in_buf, 60, 10, &(conn->fd));
  if (tcp_in_total <= 0) {
    printf("No data received from TCP\n\n");
    return;
  }
  if (printDebug) {
    printf("Data recieved from TCP: \t");
    printBinaryData(tcp_in_buf, tcp_in_total);
  }
  printf("Host %s requesting FC:%d, from unit: %d\n",inet_ntoa(conn->addr.sin_addr),tcp_in_buf[7],tcp_in_buf[6]);

  // Send data via serial
  int serial_out_total = tcp_in_total - 6;
  memcpy(serial_out_buf, &tcp_in_buf[6], serial_out_total);
  uint16_t CRC = nmbs_crc_calc(serial_out_buf, serial_out_total);
  serial_out_buf[serial_out_total] = (uint8_t)(CRC >> 8);
  serial_out_buf[serial_out_total + 1] = (uint8_t)(CRC & 0xFF);
  serial_out_total += 2;
  write(serial_port_fd, serial_out_buf,
        serial_out_total * sizeof(serial_out_buf[0]));
  if (printDebug) {
    printf("Data sent via serial: \t\t");
    printBinaryData(serial_out_buf, serial_out_total);
  }
  // Get data back from serial
  int serial_in_total =
      read(serial_port_fd, serial_in_buf, sizeof(serial_in_buf));
  if (serial_in_total <= 0) {
    printf("Error reading back serial response: %s\n\n", strerror(errno));
    return;
  }
  if (printDebug) {
    printf("Data received from serial: \t");
    printBinaryData(serial_in_buf, serial_in_total);
  }

  // Send data back via TCP
  int serial_in_total_wo_CRC = serial_in_total - 2;
  memcpy(&tcp_out_buf[0], &tcp_in_buf[0],
         4); // Transaction Identifier, Protocol Identifier
  tcp_out_buf[4] = serial_in_total_wo_CRC >> 8;
  tcp_out_buf[5] = serial_in_total_wo_CRC & 0xFF;
  memcpy(&tcp_out_buf[6], &serial_in_buf[0], serial_in_total_wo_CRC);
  int tcp_out_total = 6 + serial_in_total_wo_CRC;
  write_fd_linux(tcp_out_buf, tcp_out_total, 10, &(conn->fd));
  if (printDebug) {
    printf("Data sent back via TCP: \t");
    printBinaryData(tcp_out_buf, tcp_out_total);
    printf("\n");
  }
  printf("Sent response back to %s, FC:%d\n",inet_ntoa(conn->addr.sin_addr),tcp_out_buf[7]);
}
