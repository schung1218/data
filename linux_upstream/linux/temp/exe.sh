make LOCALVERSION= -j8
cp arch/arm64/boot/Image /media/sf_shared/home_work/image/
#git diff > modify.patch
#cp modify.patch /home/schung/work/ma35d1/ma35d1-yocto/sources/meta-ma35d1/recipes-kernel/linux/files/


# make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- LOCALVERSION= -j8

