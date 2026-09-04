BOARD = esp32:esp32:esp32
CORE_VERSION = 3.3.10

ifeq ($(OS),Windows_NT)
ARDUINO_CLI = "C:/Program Files/Arduino CLI/arduino-cli.exe"
else
ARDUINO_CLI = arduino-cli
endif

export ARDUINO_NETWORK_CONNECTION_TIMEOUT = 1h

build: .core .libs
	$(ARDUINO_CLI) compile flix --fqbn $(BOARD) --build-property build.core_debug_level=1 $(EXTRA)

core: .core

.core:
	$(ARDUINO_CLI) core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
	$(ARDUINO_CLI) core install esp32:esp32@$(CORE_VERSION) --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
	if not exist .core type nul > .core

libs: .libs

.libs:
	$(ARDUINO_CLI) lib update-index
	$(ARDUINO_CLI) lib install FlixPeriph
	$(ARDUINO_CLI) lib install MAVLink@2.0.33
	$(ARDUINO_CLI) lib install "Adafruit BMP280 Library"
	$(ARDUINO_CLI) lib install IBusBM
	if not exist .libs type nul > .libs

clean:
	rm -rf flix/build flix/cache .core .libs

.PHONY: build core libs clean