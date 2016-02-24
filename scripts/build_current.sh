#!/bin/bash

if [ -n "${1}" ]; then
	cd ${1}
fi

make -q clean

make M=$(pwd) O=/lib/modules/$(uname -r)/build/ -C /lib/modules/$(uname -r)/source/
