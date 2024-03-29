#!/bin/bash

# ***** Manual *****

# function for usage information
manual () {
  echo "Call: ./createSpreadConfig.sh <Config Name> <Spread Port> <Net IP> <Host IP>+"
  echo ""
  echo "Necessary arguments:"
  echo " <Config Name>    Name of spread config."
  echo " <Spread Port>    The port of the spread."
  echo " <Net IP>         The net IP contains the first three bytes of the IP addresses."
  echo " <Host IP>        The host IP is the last byte of the IP address. There should be given at least one host IP, but there can also be more."
  echo ""
  echo "Example:"
  echo -e "Create spread config \"testconfig\" for the IPs 10.0.0.2, 10.0.0.3 and 10.0.0.4 for a spread running at port 4803"
  echo -e " - Filename is \"testconfig\""
  echo -e " - Port is \"4803\""
  echo -e " - Net IP is \"10.0.0\""
  echo " - There are 3 host IPs: 2, 3 and 4"
  echo " => Final call: ./createSpreadConfig.sh testconfig 4803 10.0.0 2 3 4"
}



# ***** Checking arguments and print information *****

# Check argument count
if [[ $# -lt 4 ]]; then
  echo "ERROR: There are missing arguments! Please have a look into the following manual:"
  echo ""
  manual
  exit 1
fi

# Loading arguments
idcount=$[ $# - 3];
filename=$1
port=$2
netip=$3

# Create pattern for integer numbers
intPattern='^[0-9]+$'

# Check port (if integer)
if ! [[ $port =~ $intPattern ]]; then
  echo -e "ERROR: The given port \"$port\" (2. argument) is not a number!"
  echo ""
  manual
  exit 1
fi

# Check Net IP
ipbytes=(${netip//./ })
# Check length
if [[ ${#ipbytes[@]} -ne 3 ]]; then
  echo -e "ERROR: The given Net IP \"${netip}\" (3. argument) does not have 3 bytes! Please have a look into the following manual:"
  echo ""
  manual
  exit 1
fi
# Check each byte
for ipbyte in ${ipbytes[@]}; do
  # Check if integer
  if ! [[ $ipbyte =~ $intPattern ]]; then
    echo -e "ERROR: The given Net IP \"$netip\" (3. argument) contains non-integer numbers!"
    echo ""
    manual
    exit 1
  fi
  # Check if smaller than 256
  if [[ $ipbyte -gt 255 ]]; then
    echo -e "ERROR: The given Net IP \"$netip\" (3. argument) has bytes greater than 255!"
    echo ""
    manual
    exit 1
  fi
done

# Check Host IPs
declare -a iparray
for p in `seq 4 $#`; do
  arrid=$[ $p - 4 ]
  # Check if integer
  if ! [[ ${!p} =~ $intPattern ]]; then
    echo -e "ERROR: The given Host IP \"${!p}\" ($p. argument) is not an integer number!"
    echo ""
    manual
    exit 1
  fi
  # Check if smaller than 256
  if [[ ${!p} -gt 255 ]]; then
    echo -e "ERROR: The given Host IP \"${!p}\" ($p. argument) is a byte greater than 255!"
    echo ""
    manual
    exit 1
  fi
  # Check if unique
  for arr in `seq 0 $[ $arrid - 1]`; do
    if [[ ${!p} -eq ${iparray[$arr]} ]]; then
      echo -e "ERROR: The given Host IP \"${!p}\" ($p. argument) was already listed as $[ $arr + 4]. argument."
      echo ""
      manual
      exit 1
    fi
  done
  iparray[$arrid]=${!p}
done


# Print information
echo -e "Creating spread config \"${filename}\" for the following $idcount IPs:"
for p in `seq 4 $#`; do
  echo " - ${netip}.${!p}:${port}"
done



# ***** Writing File *****

# Printing introduction
echo -e "# Blank lines are permitted in this file.
# spread.conf sample file
# 
# questions to spread@spread.org
#

#MINIMAL REQUIRED FILE
#
# Spread should work fine on one machine with just the uncommented 
# lines below. The rest of the file documents all the options and\n# more complex network setups.
#
# This configures one spread daemon running on port ${port} on localhost.
" > ${filename}


# Printing device IPs and ports
for p in `seq 4 $#`; do
  echo "Spread_Segment ${netip}.255:${port} {" >> ${filename}
  echo -e "    device$[ $p - 3 ]      ${netip}.${!p}" >> ${filename}
  echo "}" >> ${filename}
  echo "" >> ${filename}
done


# Printing additional information
echo -e "# Spread options
#---------------------------------------------------------------------------

#---------------------------------------------------------------------------
#Set what internal Spread events are logged to the screen or file 
# (see EventLogFile).
# Default setting is to enable PRINT and EXIT events only. 
#The PRINT and EXIT types should always be enabled. The names of others are:
#        EXIT PRINT DEBUG DATA_LINK NETWORK PROTOCOL SESSION 
#    CONFIGURATION MEMBERSHIP FLOW_CONTROL STATUS EVENTS 
#    GROUPS MEMORY SKIPLIST ALL NONE    
#    ALL and NONE are special and represent either enabling every type 
#                                           or enabling none of them.
#    You can also use a \"!\" sign to negate a type, 
#        so { ALL !DATA_LINK } means log all events except data_link ones.

# DebugFlags = { PRINT EXIT }

# Set priority level of events to output to log file or screen
# The possible levels are: 
#    pDEBUG INFO WARNING ERROR CRITICAL FATAL
# Once selected all events tagged with that priority or higher will
# be output. FATAL events are always output and cause the daemon to 
# shut down. Some Events are tagged with a priority of PRINT which
# causes them to print out no matter what priority level is set. 
#
# The default level used if nothing is set is INFO.
    
#EventPriority =  INFO

#Set whether to log to a file as opposed to stdout/stderr and what 
# file to log to.
# Default is to log to stdout.
#
#If option is not set then logging is to stdout.
#If option is set then logging is to the filename specified.
# The filename can include a %h or %H escape that will be replaced at runtime
# by the hostname of the machine upon which the daemon is running.
# For example \"EventLogFile = spreadlog_%h.log\" with 2 machines 
# running Spread (machine1.mydomain.com and machine2.mydomain.com) will
# cause the daemons to log to \"spreadlog_machine1.mydomain.com.log\" and
# \"spreadlog_machine2.mydomain.com.log\" respectively.

#EventLogFile = testlog.out

#Set whether to add a timestamp in front of all logged events or not.
# Default is no timestamps. Default format is \"[%a %d %b %Y %H:%M:%S]\".
#If option is commented out then no timestamp is added.
#If option is enabled then a timestamp is added with the default format
#If option is enabled and set equal to a string, then that string is used
#   as the format string for the timestamp. The string must be a valid time
#   format string as used by the strftime() function.

#EventTimeStamp
# or
#EventTimeStamp = \"[%a %d %b %Y %H:%M:%S]\"

#Set whether to add a precise (microsecond) resolution timestamp to all logged
# events or not. This option requires that EventTimeStamp is also enabled. 
# If the option is commented out then the microsecond timestamp is not added
# If the option is uncommented then a microsecond time will print in addition
#  to the H:M:S resolution timestamp provided by EventTimeStamp. 

#EventPreciseTimeStamp

# Set to initialize daemon sequence numbers to a 'large' number for testing
# this is purely a debugging capability and should never be enabled on
# production systems (note one side effect of enabling this is that 
# your system will experience an extra daemon membership every few messages
# so you REALLY do not want this turned on)
# If you want to change the initial value the sequence number is set to
# you need to edit the #define INITIAL_SEQUENCE_NEAR_WRAP at the top
# of configuration.h

#DebugInitialSequence

#Set whether to allow dangerous monitor commands 
# like \"partition, flow_control, or kill\"
# Default setting is FALSE.
#If option is set to false then only \"safe\" monitor commands are allowed 
#    (such as requesting a status update).
#If option is set to true then all monitor commands are enabled. 
#   THIS IS A SECURTIY RISK IF YOUR NETWORK IS NOT PROTECTED!

DangerousMonitor = true

#Set handling of SO_REUSEADDR socket option for the daemon's TCP
# listener.  This is useful for facilitating quick daemon restarts (OSes
# often hold onto the interface/port combination for a short period of time
# after daemon shut " >> ${filename}




