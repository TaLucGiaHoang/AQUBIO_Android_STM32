1. Download srec_cat here:
http://srecord.sourceforge.net/download.html

2. Convert from .srec to .bin (and remove the bootloader area)
srec_cat.exe asp.srec -offset -0x08040000 -o asp.bin -binary

