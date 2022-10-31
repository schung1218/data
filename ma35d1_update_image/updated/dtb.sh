sudo dd if=arch/arm64/boot/dts/nuvoton/ma35d1-som-512m.dtb of=$1 conv=notrunc seek=5632 bs=512
#sudo dd if=arch/arm64/boot/dts/nuvoton/ma35d1-pm3156-256m.dtb of=$1 conv=notrunc seek=5632 bs=512
sync
