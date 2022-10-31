sudo dd if=$1 of=$2 conv=notrunc seek=6144 bs=512 status=progress
sync
