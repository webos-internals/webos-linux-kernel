#!/bin/sh
#
# buildall.sh -- Build all the targets we care about at Palm
#
# (c) 2008, Palm, Inc.  All Rights Reserved
#
# This is a usefull tool to use before you make a submission to ensure that everything
# will work on all targets.  I made it because I broke the build when I didn't build for
# 3430 targets after making a 2430 change.
#

PALM_ARM_TARGETS=" \
    omap_joplin_2430 \
    omap_joplin_3430 \
    omap_sirloin_3430 \
"

print_help() {
	printf "\nUsage: %s: [-c]\n" $0
    printf "Options:\n"
	printf  "  -c    Clean target before building.\n"
	printf "\n\n"
	exit 2
}

failed_targets=""
do_clean=0

while getopts ch opt
do
    case $opt in
    c)    do_clean=1 ;;
	h)    print_help $0 ; exit 0 ;;
    *)    print_help $0 ; exit 2 ;;
    esac
done
shift $(($OPTIND - 1))

echo
echo "Building all Palm Targets"

for target in $PALM_ARM_TARGETS
do
	echo
	echo "**** Building $target"

	if [ -e $target ]
    then
		#
		# Only clean on demand
		#
		if [ "$do_clean" = "1" ]
        then
            echo "******** Removing old stuff 'make clean' style"
            make O=$target clean > /dev/null
        fi
	else
        echo "******** Making output directory for $target"
		mkdir $target
    fi

	#
	# If the defconfig is newer, copy over the .config file
    #
	if [ $target/.config -ot arch/arm/configs/${target}_defconfig ]
    then
        echo "******** Installing new .config"
        cp arch/arm/configs/${target}_defconfig $target/.config
    fi

	echo "******** Running 'make -j3' and logging into $target/build.log"
	make -j3 O=$target > $target/build.log 2>&1 
	make -j3 uImage O=$target >> $target/build.log 2>&1 
	if [ ! "$?" = "0" ]
	then
		echo "**** Failed to build kernel for $target!!!! Please check log." >&2
		failed_targets="${failed_targets} $target"
    else
        echo "**** Built $target"
    fi
done


echo
echo -n "Finished building all targets."
if [ "$failed_targets" = "" ]
then
    echo "  Total success!"
else
    echo "  These failed to build: $failed_targets."
fi
echo
