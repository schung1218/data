# The following boards were test in this release
```
numaker-som-ma35d16a81
```
# Using repo to download source
```
$ repo init -u https://github.com/schung1218/data.git -m openbmc/openbmc_ma35d1.xml
$ repo sync
```
###### NOTE: 
```
1.Probably you will get server certificate verification failed
Solve it in the following way: 
	export GIT_SSL_NO_NOTIFY=1
	or
	git config --global http.sslverify false

2.The setting of the board can be modified at openbmc/meta-ma35d1/meta-ma35d1/conf/machine/<machine>.conf
```

# Build openbmc
. setup numaker-som-ma35d16a81 build

###### Usage:
	MACHINE=<machine> DISTRO=<distro> source sources/init-build-env <build-dir>
	<machine>    machine name
	<distro>     distro name
	<build-dir>  build directory

# Step by step to build openbmc
To build and use the yocto, do the following:
```
$ repo init -u https://github.com/schung1218/data.git -m openbmc/openbmc_ma35d1.xml
$ repo sync
$ . meta-ma35d1/init-openbmc-env numaker-som-ma35d16a81 build
$ bitbake obmc-phosphor-image

```

# OpenBMC Login Default USER: root PASSWORD: 0penBmc


###### NOTE:
```
$ cd openbmc/meta-ma35d1
$ git am 0001-Support-OpenBmc.patch -3
