#!/bin/sh
export TARGET_PRODUCT=ventana
export ARCH=arm
export CROSS_COMPILE=../prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
export COMPAL_BOARD=sit
make tegra_compal_indigo_sit_defconfig
make menuconfig
cp .config arch/arm/configs/tegra_compal_indigo_sit_defconfig
make mrproper
