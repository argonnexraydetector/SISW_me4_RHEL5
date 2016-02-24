This code is the Silicon Software linux source code for their
frame grabbers. I had to alter a bit to make it build on my install
of Linux, RHEL 5.

It has hacks... It is based on 
menable_linux_drv_4.1.5 from silicon sw website.


So far at 1st commit, Makefile and menable_core.c was changed to remove
me5 support. only menable5.c called list.h, so that is what I removed. I
onlu have a menable 4 card, so we should be OK...

T

