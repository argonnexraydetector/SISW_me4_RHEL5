#!/bin/bash

# get settings from build script
eval $(grep -e '^KVERSIONS=' -e '^ARCHS=' -e '^KERNELDIR=' $(dirname $0)/release.sh)

# location of kernel images
KERNELLOC=ftp://agis.ma.silicon-software.de/Linux/kernel

PATCHOPTS="-p1"
WGETOPTS="-P ${KERNELDIR} -nc"

if [ "${1}" != "--verbose" ]; then
	PATCHOPTS="${PATCHOPTS} -s"
	WGETOPTS="${WGETOPTS} -q"
fi

SCRIPTDIR=$(pwd)/$(dirname ${0})

if [ "$(uname -p)" = "x86_64" ]; then
	MACHINE="x86_64"
else
	MACHINE="i386"
fi

CPUS=$(cat /proc/cpuinfo |grep ^processor|wc -l)

if [ "${CPUS}" = "1" ]; then
	MAKEOPTS=""
else
	MAKEOPTS="-j $((${CPUS}+1))"
fi

#usage unpack_to target_path tarfile [patch]
unpack_to()
{
	echo unpack_to $@

	local TARFILE=${2}
	local TARVER=$(echo ${TARFILE} | sed 's/.*linux-//;s/\.tar.*//')

	mkdir -p ${1} || return

	cd ${1}
	tar xjf ${TARFILE}
	mv linux-${TARVER}/* linux-${TARVER}/.[a-z]* .
	rmdir linux-${TARVER}

	shift
	shift
	for PATCHFILE in $@; do
		bzcat ${PATCHFILE} | patch ${PATCHOPTS}
	done
}

unpack_kernel()
{
	if [ -r ${KERNELDIR}/linux-${1}/Makefile ]; then
		echo kernel $1 seems to exists, skipping
		return 0;
	fi

	wget ${WGETOPTS} ${KERNELLOC}/linux-${1}.tar.bz2 && (
		unpack_to ${KERNELDIR}/linux-${1} ${KERNELDIR}/linux-${1}.tar.bz2 || return
	) || (
		local DOTVER=$(echo ${1} | sed 's/-.*//')
		local SUBVER=$(echo ${1} | sed 's/.*-//')
		local SUBPATCH=""
		if [ "${SUBVER}" != "${1}" ]; then
			wget ${WGETOPTS} ${KERNELLOC}/patch-${1}.bz2 || return
			SUBPATCH=${KERNELDIR}/patch-${1}.bz2
		fi
		wget ${WGETOPTS} ${KERNELLOC}/linux-${DOTVER}.tar.bz2 && (
			unpack_to ${KERNELDIR}/linux-${1} ${KERNELDIR}/linux-${DOTVER}.tar.bz2 ${SUBPATCH}
		) || (
			local MAINVER=$(echo ${DOTVER} | sed 's/\.[0-9]*$//')
			wget ${WGETOPTS} ${KERNELLOC}/patch-${DOTVER}.bz2 || return
			wget ${WGETOPTS} ${KERNELLOC}/linux-${MAINVER}.tar.bz2 || return
			if [ ! -d ${KERNELDIR}/${1} ]; then
				unpack_to ${KERNELDIR}/linux-${1} ${KERNELDIR}/linux-${MAINVER}.tar.bz2 ${KERNELDIR}/patch-${DOTVER}.bz2 ${SUBPATCH} || return
			fi
		)
	)

	cd ${KERNELDIR}/linux-${1}
	for i in ${SCRIPTDIR}/configs/patch_${1}_*; do
		patch -p1 -i $i || return 1
	done
	cd -

	return 0
}

mkdir -p ${KERNELDIR}

for vers in ${KVERSIONS}; do
	# for unpacking even if no configs present
	if [ "${1}" = "--unpack" ]; then
		unpack_kernel ${vers} || echo "unpack ${vers} failed"
	fi

	if [ "$(find ${SCRIPTDIR}/configs/config_${MACHINE}_${vers}_* | wc -l)" != "0" ]; then
		unpack_kernel ${vers} || echo "unpack ${vers} failed"
		for arch in ${ARCHS}; do
			cd ${KERNELDIR}/linux-${vers}
			if [ -r ${SCRIPTDIR}/configs/config_${MACHINE}_${vers}_${arch} ]; then
				BUILDDIR=${KERNELDIR}/build/${vers}/${MACHINE}_${arch}
				mkdir -p ${BUILDDIR} || return
				cp ${SCRIPTDIR}/configs/config_${MACHINE}_${vers}_${arch} ${KERNELDIR}/build/${vers}/${MACHINE}_${arch}/.config
				echo "#### config for ${vers}/${MACHINE}_${arch} created"
				make ${MAKEOPTS} O=${BUILDDIR}
			else
				echo "${SCRIPTDIR}/configs/config_${MACHINE}_${vers}_${arch} does not exist, skipping build"
			fi
		done
	fi
done
