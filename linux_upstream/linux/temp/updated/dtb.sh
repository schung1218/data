sudo dd if=arch/arm64/boot/dts/nuvoton/ma35d1-evb.dtb of=$1 conv=notrunc seek=5632 bs=512
sync
