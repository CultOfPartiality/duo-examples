#!/bin/bash

ssh root@192.168.42.1 "kill -7 \`pidof modbus-translator\`"
scp modbus-translator root@192.168.42.1:/root/
#ssh root@192.168.42.1 "/root/modbus-translator -v 192.168.42.1"