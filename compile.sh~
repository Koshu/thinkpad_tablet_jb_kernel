export CCOMPILER=${HOME}/Android/clockworkmod/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
cp arch/arm/configs/tpt_kernel_defconfig .config
make ARCH=arm CROSS_COMPILE=$CCOMPILER oldconfig
mv .config arch/arm/configs/tpt_kernel_defconfig
#make ARCH=arm CROSS_COMPILE=$CCOMPILER -j`grep 'processor' /proc/cpuinfo | wc -l`
