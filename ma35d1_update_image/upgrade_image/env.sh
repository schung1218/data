sudo dd if=$1 of=$2 conv=notrunc seek=512 bs=512
sync
