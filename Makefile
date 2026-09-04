BOARD = esp32:esp32:esp32
CORE_VERSION = 2.0.17

export ARDUINO_NETWORK_CONNECTION_TIMEOUT = 1h

build: .core .libs
	arduino-cli compile flix --fqbn $(BOARD) --build-property build.core_debug_level=1 $(EXTRA)

core: .core

.core:
	arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
	arduino-cli core install esp32:esp32@$(CORE_VERSION) --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
	if not exist .core type nul > .core

libs: .libs

.libs:
	arduino-cli lib update-index
	arduino-cli lib install FlixPeriph
	arduino-cli lib install MAVLink@2.0.33
	arduino-cli lib install "Adafruit BMP280 Library"
	arduino-cli lib install IBusBM
	if not exist .libs type nul > .libs

clean:
	rm -rf flix/build flix/cache .core .libs

.PHONY: build core libs clean