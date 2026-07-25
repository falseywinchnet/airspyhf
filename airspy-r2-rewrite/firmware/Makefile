SHELL := /bin/sh

QUALIFIED_TOOLCHAIN := arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi
LOCAL_TOOLCHAIN := $(CURDIR)/../.toolchains/$(QUALIFIED_TOOLCHAIN)
WORKSPACE_TOOLCHAIN := $(CURDIR)/../../.toolchains/$(QUALIFIED_TOOLCHAIN)
TOOLCHAIN_ROOT ?= $(firstword $(wildcard $(LOCAL_TOOLCHAIN) $(WORKSPACE_TOOLCHAIN)))
MODEL_BUILD := $(CURDIR)/.build/model

.PHONY: all firmware model test toolchain-check clean

all: firmware test

toolchain-check:
	@test -n "$(TOOLCHAIN_ROOT)" || { \
		echo "Arm GNU Toolchain 14.2.Rel1 was not found."; \
		echo "Install it under ../.toolchains/$(QUALIFIED_TOOLCHAIN)"; \
		echo "or set TOOLCHAIN_ROOT. See BUILDING.md."; \
		exit 1; \
	}
	@PATH="$(TOOLCHAIN_ROOT)/bin:$$PATH" arm-none-eabi-gcc --version | head -n 1
	@printf '#include <stdint.h>\nint main(void){return 0;}\n' | \
		PATH="$(TOOLCHAIN_ROOT)/bin:$$PATH" \
		arm-none-eabi-gcc -x c -c -o "$(CURDIR)/.toolchain-check.o" -
	@$(RM) "$(CURDIR)/.toolchain-check.o"

firmware: toolchain-check
	PATH="$(TOOLCHAIN_ROOT)/bin:$$PATH" $(MAKE) -C device

model:
	cmake -S model -B "$(MODEL_BUILD)" -DCMAKE_BUILD_TYPE=Debug
	cmake --build "$(MODEL_BUILD)"

test: model
	ctest --test-dir "$(MODEL_BUILD)" --output-on-failure

clean:
	$(MAKE) -C device clean
	cmake -E remove_directory "$(CURDIR)/.build"
