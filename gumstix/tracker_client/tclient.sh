stty -F /dev/ttyS0 57600
stty -F /dev/ttyS0 raw

while true; do

  echo "tclient started"
  /opt/tracker_client/tclient localhost 9001 127.0.0.1 9005 1 &> /dev/null
  echo "tclient terminated"

done
