echo 2000 > /sys/module/dmatest/parameters/timeout
echo 2 > /sys/module/dmatest/parameters/iterations
echo dma0chan0 > /sys/module/dmatest/parameters/channel
echo 1 > /sys/module/dmatest/parameters/run


#   multi chan
#    % modprobe dmatest
#    % echo 2000 > /sys/module/dmatest/parameters/timeout
#    % echo 1 > /sys/module/dmatest/parameters/iterations
#    % echo dma0chan0 > /sys/module/dmatest/parameters/channel
#    % echo dma0chan1 > /sys/module/dmatest/parameters/channel
#    % echo dma0chan2 > /sys/module/dmatest/parameters/channel
#    % echo 1 > /sys/module/dmatest/parameters/run


