SHELL := /bin/sh

CURRENT_BUILD := $(CURDIR)/.build/current
MODEL_BUILD := $(CURDIR)/.build/model
READABLE_BUILD := $(CURDIR)/.build/readable

ifeq ($(shell uname -s),Darwin)
SHARED_LIBRARY_SUFFIX := dylib
else
SHARED_LIBRARY_SUFFIX := so
endif

.PHONY: all current readable model test verify hardware-smoke windows-x64 clean

all: verify

current:
	cmake -S current -B "$(CURRENT_BUILD)" -DCMAKE_BUILD_TYPE=Release
	cmake --build "$(CURRENT_BUILD)"

readable:
	cmake -S readable -B "$(READABLE_BUILD)" -DCMAKE_BUILD_TYPE=Debug
	cmake --build "$(READABLE_BUILD)"

model:
	cmake -S model -B "$(MODEL_BUILD)" -DCMAKE_BUILD_TYPE=Debug
	cmake --build "$(MODEL_BUILD)"

test: model
	ctest --test-dir "$(MODEL_BUILD)" --output-on-failure

verify: current readable test
	sh scripts/compare_exports.sh \
		"$(CURRENT_BUILD)/libairspy/src/libairspy.$(SHARED_LIBRARY_SUFFIX)" \
		"$(READABLE_BUILD)/libairspy_readable.$(SHARED_LIBRARY_SUFFIX)"

hardware-smoke: readable
	"$(READABLE_BUILD)/airspy-readable-hardware-smoke"

windows-x64:
	./windows/build-x64.sh

clean:
	cmake -E remove_directory "$(CURDIR)/.build"
