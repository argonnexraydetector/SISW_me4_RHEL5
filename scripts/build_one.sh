#!/bin/bash

if [ $# != 4 ]; then
	echo "Usage: $0 KERNELDIR KERNELVERSION MACHINE CONFIGURATION" >&2
	exit 1
fi

cd $(dirname $0)/..

if [ -e CMakeCache.txt ]; then
	echo "A CMakeCache.txt exists in your driver directory. You must build out of tree!" >&2
	exit 1
fi

KERNELDIR=$1
KV=$2
MACHINE=$3
ARCH=$4

make -q clean

make M=$(pwd) O=${KERNELDIR}/build/${KV}/${MACHINE}_${ARCH}/ -C ${KERNELDIR}/linux-${KV} || exit 1
