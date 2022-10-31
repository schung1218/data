sudo dd if=$1 of=$2 conv=notrunc seek=5632 bs=512
sync
