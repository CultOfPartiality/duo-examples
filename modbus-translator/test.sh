#!/bin/bash

command="
  /root/modbus-translator 192.168.42.1;
"

scp modbus-translator root@192.168.42.1:/root/

ssh root@192.168.42.1 $command
