
.PHONY: clean, build, flash, flash-erase, debug, gdb, menu

.ONESHELL:
build: FORCE
	. ~/esp/v5.3.1/esp-idf/export.sh
	export IDF_TARGET=esp32h2
	idf.py build

# Force is needed because directory with the same name exists
FORCE:

.ONESHELL:
clean:
	. ~/esp/v5.3.1/esp-idf/export.sh
	export IDF_TARGET=esp32h2
	idf.py fullclean

.ONESHELL:
debug gdb:
	. ~/esp/v5.3.1/esp-idf/export.sh
	export IDF_TARGET=esp32h2
	idf.py gdb

.ONESHELL:
flash:
	. ~/esp/v5.3.1/esp-idf/export.sh
	export IDF_TARGET=esp32h2
	idf.py -p /dev/ttyACM0 flash

.ONESHELL:
flash-erase:
	. ~/esp/v5.3.1/esp-idf/export.sh
	export IDF_TARGET=esp32h2
	idf.py -p /dev/ttyACM0 erase-flash

.ONESHELL:
menu:
	. ~/esp/v5.3.1/esp-idf/export.sh
	export IDF_TARGET=esp32h2
	idf.py menuconfig

