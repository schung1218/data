FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI+="file://linuxfb_doubleubffer.patch"

PACKAGECONFIG_append = " accessibility examples directfb tslib linuxfb fontconfig"

