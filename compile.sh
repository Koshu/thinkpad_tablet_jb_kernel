export CCOMPILER=${HOME}/Android/clockworkmod/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
make ARCH=arm CROSS_COMPILE=$CCOMPILER oldconfig
#make ARCH=arm CROSS_COMPILE=$CCOMPILER -j`grep 'processor' /proc/cpuinfo | wc -l`
