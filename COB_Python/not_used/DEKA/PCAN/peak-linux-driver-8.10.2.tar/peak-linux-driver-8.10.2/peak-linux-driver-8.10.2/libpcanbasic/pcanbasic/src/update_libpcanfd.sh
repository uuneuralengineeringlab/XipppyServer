#!/bin/bash

echo "Fetching pcan driver latest version in pcan.git..."
if [ ! -d "git/pcan.git" ]; then
  mkdir git
  git clone --progress -v "git@10.1.12.14:sgrosjean/pcan" "git/pcan.git"
fi

cd git/pcan.git
git pull 
git reset --hard Release_20180720_n-8.6.0
PCAN_VERSION=`git describe master`
cd ../..

# old school (see below comment)
if [ "${should_run_as_before}es" == "yes" ]; then
	echo "Copying required files from pcan.git to libpcanfd folder..."
	if [ -d "libpcanfd" ]; then
		echo "Making a backup of existing directory..."
		rm -rf git/libpcanfd.old
		mv libpcanfd git/libpcanfd.old
	fi
	mkdir -p libpcanfd/src
	cp git/pcan.git/driver/pcan.h libpcanfd/
	cp git/pcan.git/driver/pcanfd.h libpcanfd/
	cp git/pcan.git/lib/libpcanfd.h libpcanfd/
	cp git/pcan.git/lib/src/libpcanfd.c libpcanfd/src/
	cp git/pcan.git/lib/src/libprivate.h libpcanfd/src/
else
	# 2019-02-06 - SGr
	# since pcanbasic is part of the pcan driver package, pcan files
	# are imported into a tree similar to the pcan one:
	#
	# src/pcan
	# ├── driver
	# │   ├── pcanfd.h
	# │   └── pcan.h
	# └── lib
	#    ├── libpcanfd.h
	#    └── src
	#        ├── libpcanfd.c
	# 	 └── libprivate.h
	echo "Copying required files from pcan.git to local pcan folder..."
	if [ -d "pcan" ]; then
		echo "Making a backup of existing directory..."
		rm -rf git/pcan.old
		mv pcan git/pcan.old
	fi
	mkdir -p pcan/driver
	mkdir -p pcan/lib/src
	cp git/pcan.git/driver/pcan.h pcan/driver
	cp git/pcan.git/driver/pcanfd.h pcan/driver
	cp git/pcan.git/lib/libpcanfd.h pcan/lib
	cp git/pcan.git/lib/src/libpcanfd.c pcan/lib/src
	cp git/pcan.git/lib/src/libprivate.h pcan/lib/src

	# New! This file MUST also exist in the standalone package, to drive
	# correctly the compilation
	echo "\
# .config
#
# This file is used to help in building the pcanbasic library for Linux.
# Its content MUST be readable (that is, included) from any Makefile.
# It SHOULD not be part of the libpcanbasic directory of the pcan driver
# package.
#
# This file is automatically created by `basename $0`.
# Creation date: `date +%Y/%m/%d-%H:%M:%S`
#
# (C) PEAK-System GmbH 
# www.peak-system.com
#
CONFIG_PCAN_VERSION=\"$PCAN_VERSION\"\
" > pcan/.config

fi

echo "Completed."
