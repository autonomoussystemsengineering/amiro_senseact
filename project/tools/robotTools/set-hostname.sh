#!/bin/sh

set -e

prefix=${1:-amiro}

#inet addr:192.168.102.3 Bcast:192.168.102.255 Mask:255.255.255.0
ip=`ifconfig wlan0 | grep -F 'inet addr:192.168.102.' | cut -d \  -f 12 | cut -d : -f 2`

echo "ip $ip"
if [ -n "$ip" ]; then
	echo $ip
	suffix=`echo $ip | cut -d . -f 4`
	hostname "$prefix$suffix"
else
	echo "no ip found" >&2
	exit 1
fi
