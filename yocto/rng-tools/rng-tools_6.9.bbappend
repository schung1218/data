FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"
SRC_URI += " \
	file://rngd.service2 \
	file://rngd.sh \
	"

do_install_append() {
    install -Dm 0755 ${WORKDIR}/rngd.sh ${D}${sysconfdir}/rngd.sh
    install -Dm 0644 ${WORKDIR}/default ${D}${sysconfdir}/default/rng-tools
    install -Dm 0755 ${WORKDIR}/init ${D}${sysconfdir}/init.d/rng-tools
    install -Dm 0644 ${WORKDIR}/rngd.service2 \
                     ${D}${systemd_system_unitdir}/rngd.service
}

