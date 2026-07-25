SHELL := /bin/sh

CURRENT_BUILD := $(CURDIR)/.build/current
MODEL_BUILD := $(CURDIR)/.build/model

.PHONY: all current model test clean

all: current test

current:
	cmake -S current -B "$(CURRENT_BUILD)" -DCMAKE_BUILD_TYPE=Release
	cmake --build "$(CURRENT_BUILD)"

model:
	cmake -S model -B "$(MODEL_BUILD)" -DCMAKE_BUILD_TYPE=Debug
	cmake --build "$(MODEL_BUILD)"

test: model
	ctest --test-dir "$(MODEL_BUILD)" --output-on-failure

clean:
	cmake -E remove_directory "$(CURDIR)/.build"
