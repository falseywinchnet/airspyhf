CC ?= cc
PKG_CONFIG ?= pkg-config
CFLAGS ?= -O2 -g

LIBUSB_CFLAGS := $(shell $(PKG_CONFIG) --cflags libusb-1.0)
LIBUSB_LIBS := $(shell $(PKG_CONFIG) --libs libusb-1.0)

.PHONY: all clean

all: memory_visibility_test atdtw_stress_test gpdma_visibility_test \
	adc_ring_telemetry spiflash_recover_varying lpcdfu_recover

memory_visibility_test: memory_visibility_test.c
	$(CC) $(CFLAGS) -std=c11 -Wall -Wextra -Wpedantic \
		$(LIBUSB_CFLAGS) $< -o $@ $(LIBUSB_LIBS)

atdtw_stress_test: atdtw_stress_test.c
	$(CC) $(CFLAGS) -std=c11 -Wall -Wextra -Wpedantic \
		$(LIBUSB_CFLAGS) $< -o $@ $(LIBUSB_LIBS) -pthread

gpdma_visibility_test: gpdma_visibility_test.c
	$(CC) $(CFLAGS) -std=c11 -Wall -Wextra -Wpedantic \
		$(LIBUSB_CFLAGS) $< -o $@ $(LIBUSB_LIBS)

adc_ring_telemetry: adc_ring_telemetry.c
	$(CC) $(CFLAGS) -std=c11 -Wall -Wextra -Wpedantic \
		$(LIBUSB_CFLAGS) $< -o $@ $(LIBUSB_LIBS)

spiflash_recover_varying: spiflash_recover_varying.c
	$(CC) $(CFLAGS) -std=c11 -Wall -Wextra -Wpedantic \
		$(LIBUSB_CFLAGS) $< -o $@ $(LIBUSB_LIBS)

lpcdfu_recover: lpcdfu_recover.c
	$(CC) $(CFLAGS) -std=c11 -Wall -Wextra -Wpedantic \
		$(LIBUSB_CFLAGS) $< -o $@ $(LIBUSB_LIBS)

clean:
	rm -f memory_visibility_test atdtw_stress_test gpdma_visibility_test \
		adc_ring_telemetry spiflash_recover_varying lpcdfu_recover
