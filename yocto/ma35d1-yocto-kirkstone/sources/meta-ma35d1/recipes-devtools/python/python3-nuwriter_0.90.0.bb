SUMMARY = "This is a python nuwriter for ma35d1 tool "

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=e49f4652534af377a713df3d9dec60cb"

inherit setuptools3 deploy native

SRC_URI = "git://github.com/OpenNuvoton/MA35D1_NuWriter.git;protocol=https;branch=master"
S = "${WORKDIR}/git"
B =  "${WORKDIR}/build"

SRCREV = "${AUTOREV}"

PACKAGES = ""

SRC_URI += " file://header-nand.json \
             file://header-sdcard.json \
             file://header-spinand.json \
             file://pack-nand.json \
             file://pack-sdcard.json \
             file://pack-spinand.json \
           "

DEPENDS += " \
    libusb1-native \
    pyinstaller-native \
    pyinstaller-hooks-contrib-native \
    python3-native \
    python3-altgraph-native \
    python3-pyusb-native \
    python3-crcmod-native \
    python3-tqdm-native \
    python3-ecdsa-native \
    python3-six-native \
    python3-pycryptodome-native \
    python3-py-native \
    jq-native \
"

BBCLASSEXTEND = "native nativesdk"

do_compile(){
    #define PY_SSIZE_T_CLEAN
    export LD_LIBRARY_PATH=${RECIPE_SYSROOT_NATIVE}/usr/lib/
#     pyinstaller --clean --win-private-assemblies ${S}/nuwriter.py -D -n nuwriter -y --distpath ${B}
    pyinstaller --exclude-module _bootlocale --clean --win-private-assemblies ${S}/nuwriter.py -D -n nuwriter -y --distpath ${B}
}

do_install(){
    install -d ${D}${bindir}
    install -d ${D}${datadir}/nuwriter
    install -d ${D}${datadir}/nuwriter/ddrimg
    cp ${B}/nuwriter/nuwriter ${D}${bindir}/
    cp ${WORKDIR}/header-nand.json  ${D}${datadir}/nuwriter/
    cp ${WORKDIR}/header-sdcard.json  ${D}${datadir}/nuwriter/
    cp ${WORKDIR}/header-spinand.json  ${D}${datadir}/nuwriter/
    cp ${WORKDIR}/pack-nand.json  ${D}${datadir}/nuwriter/
    cp ${WORKDIR}/pack-spinand.json  ${D}${datadir}/nuwriter/
    cp ${WORKDIR}/pack-sdcard.json  ${D}${datadir}/nuwriter/
   
    cp ${S}/ddrimg/* ${D}${datadir}/nuwriter/ddrimg/ 
    cp ${S}/xusb.bin ${D}${datadir}/nuwriter/
}

do_deploy() {
    install -d ${DEPLOYDIR}/${BOOT_TOOLS}/nuwriter
    install -d ${DEPLOYDIR}/${BOOT_TOOLS}/nuwriter/ddrimg
    cp -rf ${B}/nuwriter/* ${DEPLOYDIR}/${BOOT_TOOLS}/nuwriter
    
    cp ${S}/ddrimg/* ${DEPLOYDIR}/${BOOT_TOOLS}/nuwriter/ddrimg/
    cp ${S}/xusb.bin  ${DEPLOYDIR}/${BOOT_TOOLS}/nuwriter/
    cp ${S}/xusb.bin  ${DEPLOYDIR}/${BOOT_TOOLS}/
}

FILES_${PN} = ""
addtask install after do_compile
addtask deploy after do_compile
INHIBIT_SYSROOT_STRIP = "1"
INSANK_SKIP_${PN}:append = "already-stripped"

