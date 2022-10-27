SUMMARY = " pyinstaller_4.00 "

LICENSE = "MIT"
#LIC_FILES_CHKSUM = "file://COPYING.txt;md5=702f886187082d5370323fae257c8523"
LIC_FILES_CHKSUM = "file://COPYING.txt;md5=752110777ecd9e72b16df0e59c1e0019"
inherit setuptools3 native

#SRCREV= "19fb799a11d2d796fc8758808f873c40e2bf5118"
SRCREV= "bcb824c1ad2989855a5fc01d95e5fc693b2f0966"

SRC_URI = "git://github.com/pyinstaller/pyinstaller.git;protocol=https;branch=master"
S = "${WORKDIR}/git"
B =  "${WORKDIR}/build"

BBCLASSEXTEND = "native nativesdk"

INHIBIT_SYSROOT_STRIP = "1"
INSANK_SKIP_${PN}:append = "already-stripped"
