dts:

spidev@1{
        compatible = "semtech,sx1301";
        reg = <1>;
        spi-max-frequency = <12000000>;
    };


command:

./spidev_test -D /dev/spidev0.1 -p 1234567891239999988888888888888888888888888888888888888888888888888888888888888888888888887777776789
