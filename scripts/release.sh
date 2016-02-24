#!/bin/bash

cd $(pwd)/$(dirname ${0})/..

DRVVERSION=$(grep -o '^MODULE_VERSION("[^"]*' menable_core.c | sed 's/.*"//' | tr '\n' '_' | sed 's/_$//')
DRVDIR=menable_linuxdrv_src_${DRVVERSION}

MAKEFILE=Makefile
README=INSTALL

ADDITIONAL_HEADERS="../men_ioctl_codes.h"

rm -rf ${DRVDIR}
mkdir -p ${DRVDIR}

SRCLIST=$(grep ^menable-objs ${MAKEFILE} | sed 's/.*:= //;s/\.o/.c/g')
cp ${SRCLIST} ${DRVDIR}
cp $(grep -h '#include ".*\.h"' ${SRCLIST} | sort | uniq | sed 's/[^"]*"//;s/"$//') ${DRVDIR}

cp ${ADDITIONAL_HEADERS} Makefile INSTALL ${DRVDIR}

sed -i 's,\.\.\/men_ioctl_codes\.h,men_ioctl_codes\.h,' ${DRVDIR}/menable_ioctl.h

mkdir ${DRVDIR}/udev
cp 10-siso.rules men_path_id men_uiq ${DRVDIR}/udev

tar cjf ${DRVDIR}.tar.bz2 --owner=root --group=root ${DRVDIR}
