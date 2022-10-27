SUMMARY = "This is a python nuwriter for ma35d1 tool "

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=3590eb8d695bdcea3ba57e74adf8a4ed"

inherit setuptools3 native

SRCREV= "bbd2ad60c9908b2de4c96dcb2eb0365bd61d4f9f"

SRC_URI = "git://github.com/ronaldoussoren/altgraph.git;protocol=https;branch=master"
S = "${WORKDIR}/git"
B =  "${WORKDIR}/build"

SRC_URI[sha256sum] = "1f05a47122542f97028caf78775a095fbe6a2699b5089de8477eb583167d69aa"

BBCLASSEXTEND = "native nativesdk"
