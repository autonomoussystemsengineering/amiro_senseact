#!/bin/bash
# This script maps the MAC addesses of the BeBots
# to the IDs which are used in the TWB AR-Toolkit
# project

declare -A mac2id=( ['00:19:88:15:bd:b6']='1' \
                    ['00:19:88:34:8f:f5']='2' \
                    ['00:50:43:02:fe:01']='3' \
                    ['00:50:43:02:fe:01']='4' \
                    ['00:19:88:34:90:16']='5' \
                    ['00:19:88:34:90:39']='6' \
                    ['00:19:88:42:96:1c']='7' \
                    ['00:19:88:34:90:2e']='8' \
                    ['00:19:88:34:90:22']='9' \
                    ['00:19:88:10:f2:68']='10'\
                    ['00:19:88:10:f2:60']='11'\
                    ['00:19:88:10:f1:c1']='12'\
                    ['00:19:88:10:f3:1a']='13'\
                    ['00:19:88:10:f1:db']='14'\
                    ['00:19:88:10:f1:b9']='15'\
                    ['00:19:88:10:f1:d5']='16'\
                    ['00:19:88:10:f1:c6']='17'\
                    ['00:19:88:10:f1:c5']='18')
                     
# Read the MAC address
MAC=`cat /sys/class/net/wlan0/address`
# Print out the ID
echo "${mac2id["${MAC}"]}"
# Print the list
# for MAC in "${!mac2id[@]}"; do echo "$MAC - ${mac2id["$MAC"]}"; done

# 12:wlan0  HWaddr ['00:19:88:15:BD:B6']='1'
# 14:wlan0  HWaddr ['00:19:88:34:8F:F5']='2'
# 15:wlan0  HWaddr ['00:50:43:02:FE:01']='3'
# 16:wlan0  HWaddr ['00:50:43:02:FE:01']='4'
# 21:wlan0  HWaddr ['00:19:88:34:90:16']='5'
# 27:wlan0  HWaddr ['00:19:88:34:90:39']='6'
# 3:wlan0   HWaddr ['00:19:88:42:96:1C']='7'
# 4:wlan0   HWaddr ['00:19:88:34:90:2E']='8'
# 5:wlan0   HWaddr ['00:19:88:34:90:22']='9'
# 57:wlan0  HWaddr ['00:19:88:10:F2:68']='10'
# 60:wlan0  HWaddr ['00:19:88:10:F2:60']='11'
# 61:wlan0  HWaddr ['00:19:88:10:F1:C1']='12'
# 67:wlan0  HWaddr ['00:19:88:10:F3:1A']='13'
# 68:wlan0  HWaddr ['00:19:88:10:F1:DB']='14'
# 71:wlan0  HWaddr ['00:19:88:10:F1:B9']='15'
# 72:wlan0  HWaddr ['00:19:88:10:F1:D5']='16'
# 78:wlan0  HWaddr ['00:19:88:10:F1:C6']='17'
# 79:wlan0  HWaddr ['00:19:88:10:F1:C5']='18' 