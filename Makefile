BUILD_DIR ?= build
BUILD_TYPE ?= Release
PREFIX ?= /usr/local
CMAKE ?= cmake
CMAKE_ARGS ?=

.PHONY: all configure build install uninstall clean

all: build

configure:
	$(CMAKE) -S . -B $(BUILD_DIR) \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DCMAKE_INSTALL_PREFIX=$(PREFIX) \
		$(CMAKE_ARGS)

build: configure
	$(CMAKE) --build $(BUILD_DIR) --parallel

install:
	$(CMAKE) --install $(BUILD_DIR) --prefix $(PREFIX)

uninstall:
	$(CMAKE) --build $(BUILD_DIR) --target uninstall

clean:
	$(CMAKE) --build $(BUILD_DIR) --target clean
