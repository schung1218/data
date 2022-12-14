# Copyright (C) 2019-2020
# Copyright 2019-2020 Nuvoton
# Released under the MIT license (see COPYING.MIT for the terms)

SUMMARY = "Linux Kernel provided and supported by Nuvoton"
DESCRIPTION = "Linux Kernel provided and supported by Nuvoton ma35d1"

inherit kernel

# We need to pass it as param since kernel might support more then one
# machine, with different entry points
ma35d1_KERNEL_LOADADDR = "0x80080000"
KERNEL_EXTRA_ARGS += "LOADADDR=${ma35d1_KERNEL_LOADADDR}"

LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://COPYING;md5=bbea815ee2795b2f4230826c0c6b8814"

SRCBRANCH = "linux-5.4.y"
LOCALVERSION = "-${SRCBRANCH}"

KERNEL_SRC ?= "git://github.com/OpenNuvoton/MA35D1_linux-5.4.y.git;branch=master;protocol=https"
SRC_URI = "${KERNEL_SRC}"

SRC_URI += " \
    file://optee.config \
    file://cfg80211.config \
    "


SRC_URI += "${@bb.utils.contains('DISTRO_FEATURES', '88x2bu', ' file://88x2bu.ko', '', d)}"

#SRCREV="${KERNEL_SRCREV}"
SRCREV= "d44bedf95d955d9562344569f819547b38e90f9a"
S = "${WORKDIR}/git"
B = "${WORKDIR}/build"

DEPENDS += "openssl-native util-linux-native libyaml-native"
DEFAULT_PREFERENCE = "1"
# =========================================================================
# Kernel
# =========================================================================
# Kernel image type
KERNEL_IMAGETYPE = "Image"

do_configure:prepend() {
    bbnote "Copying defconfig"
    cp ${S}/arch/${ARCH}/configs/${KERNEL_DEFCONFIG} ${WORKDIR}/defconfig
    cat ${WORKDIR}/cfg80211.config >> ${WORKDIR}/defconfig

    if ${@bb.utils.contains('MACHINE_FEATURES', 'optee', 'true', 'false', d)}; then
        cat ${WORKDIR}/optee.config >> ${WORKDIR}/defconfig
    fi
}

do_deploy:append() {
    for dtbf in ${KERNEL_DEVICETREE}; do
        dtb=`normalize_dtb "$dtbf"`
        dtb_ext=${dtb##*.}
        dtb_base_name=`basename $dtb .$dtb_ext`
	ln -sf $dtb_base_name.dtb ${DEPLOYDIR}/Image.dtb
    done
}

do_install:append() {
    if [ -e ${WORKDIR}/88x2bu.ko ]; then
        install -d ${D}/${base_libdir}/modules/${KV}
        install -m 0644 ${WORKDIR}/88x2bu.ko ${D}/${base_libdir}/modules/${PV}/88x2bu.ko
    fi
}

FILES_${PN} += "${base_libdir}/modules/${PV}/${@bb.utils.contains('DISTRO_FEATURES', '88x2bu', '88x2bu.ko', '', d)}"
COMPATIBLE_MACHINE = "(ma35d1)"

