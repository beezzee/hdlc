#!/bin/bash

DEVICE=/dev/ttyACM1
echo
echo "Configure serial interface $DEVICE:"
echo "8 bit characters"
echo "reception of data"
echo "one stop bit"
echo "no parity bit"
echo "9600 baud"
echo
COMMAND="stty -F $DEVICE cs8 cread -cstopb -parenb -crtscts 9600"
echo $COMMAND
$COMMAND
echo 
echo "query settings..."

stty -F $DEVICE -a




