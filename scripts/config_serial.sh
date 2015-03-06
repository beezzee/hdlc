#!/bin/bash

DEVICE0=/dev/ttyUSB0
DEVICE1=/dev/ttyACM1
FREQ=115200

echo
echo "Configure serial interface $DEVICE:"
echo "8 bit characters"
echo "reception of data"
echo "one stop bit"
echo "no parity bit"
echo "$FREQ baud"
echo

COMMAND="stty -F $DEVICE0 cs8 cread -cstopb -parenb -crtscts $FREQ icanon"
echo $COMMAND
$COMMAND
echo 
COMMAND="stty -F $DEVICE1 cs8 cread -cstopb -parenb -crtscts $FREQ icanon"
echo $COMMAND
$COMMAND
echo 
echo "query settings..."

echo "$DEVICE0:"
stty -F $DEVICE0 -a

echo "$DEVICE1:"
stty -F $DEVICE1 -a




