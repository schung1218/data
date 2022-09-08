#!/bin/sh
/usr/sbin/rngd -f -r /dev/hwrng -o /dev/random -x jitter

