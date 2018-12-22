##
## Severity level constants
##
byte DEBUG=10 #debug level
byte INFO=20  #general level
byte WARN=30  #warning level
byte ERROR=40 #error level
byte FATAL=50 #fatal/critical level

##
## Fields
##
builtin_interfaces/Time stamp
uint8 level
string name # the name representing the logger this message came from
string msg # message
string file # file the message came from
string function # function the message came from
uint32 line # line the message came from
