stty -F /dev/ttyS0 57600
stty -F /dev/ttyS0 raw

./tclient localhost 9001 127.0.0.1 9005 1
