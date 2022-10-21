QSPI 需要做大量讀寫測試

我是拿 “rootfs” 這個 MTD partition (因為它比較大) 來做 UBI file system 讀寫測試

步驟如下:

./ubiformat /dev/mtd4
./ubiattach /dev/ubi_ctrl –m 4
mdev -s
./ubimkvol /dev/ubi0 –N test -m
mdev -s
mount –t ubifs /dev/ubi0_0 /mnt


mount 成功後, 可以用 dd 測試讀寫效能

測試performance command 如下

Write:
Command: “time dd if=/dev/zero of=/mnt/96m bs=65536 count=1536; time sync”

Read:
Command: “time dd if=/mnt/96m of=/dev/zero bs=65536 count=1536; time sync”

 

Read/write 之前要先下 flush cache
# sync; echo 1 > /proc/sys/vm/drop_caches
# sync; echo 2 > /proc/sys/vm/drop_caches

 

另外, 在 /mnt 目錄下, 新增目錄及檔案 後, 重新開機
./ubiattach /dev/ubi_ctrl –m 4
mdev -s
mount –t ubifs /dev/ubi0_0 /mnt
檢查目錄及檔案是否存在並正確.
