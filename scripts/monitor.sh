# This script should not be directly called. Only used by the Makefile.
cd esp-idf &&
idf.py -p /dev/ttyUSB0 monitor
