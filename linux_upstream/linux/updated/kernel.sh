sudo dd if=arch/arm64/boot/Image of=$1 conv=notrunc seek=6144 bs=512 status=progress
sync
