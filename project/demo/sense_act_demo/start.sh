
./sendOdometryProtoPose -o /$(hostname)/pose -r 1 > /dev/0 &
./senseSickTim -o /$(hostname)/laserscan > /dev/0 &
./motorControl -i /$(hostname)/motor > /dev/0 &
