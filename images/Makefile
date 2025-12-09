ROOT_DIR := $(notdir $(CURDIR))
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

HOST_MKIFS := mkifs
HOST_MKI := mki
HOST_SED := sed
SUFFIXES := .build .bin .raw .ui

# NOTE: This value must match the '[image=<start_addr>]' value in the build file.
IMAGE_LOAD_ADDR = 0x80000

PROCESSOR = aarch64
BUILD_TEMPLATE = $(CURDIR)/../../Templates_build_sdp800
BOARD=rpi4
GUEST_BUILD=rpi4_guest_qnx_OS.build
INSTALL=../install

.PHONY: all clean

# Build both host and guest
all: ifs-$(BOARD).bin guest-$(BOARD).ifs

# Host build rules
ifs-$(BOARD).bin: $(BOARD).build
	$(HOST_MKIFS) -v -r$(INSTALL) $(MKIFSFLAGS) $^ $@

ifs-$(BOARD).raw: $(BOARD).build
	$(CP_HOST) $(BOARD).build $(BOARD)-go.build
	$(HOST_SED) -i 's/u reg/u arg/' $(BOARD)-go.build
	$(HOST_MKIFS) -v -r$(INSTALL) $(MKIFSFLAGS)  $(BOARD)-go.build $@
	$(RM_HOST) $(BOARD)-go.build

ifs-$(BOARD).ui: ifs-$(BOARD).bin
	$(HOST_MKI) -a $(IMAGE_LOAD_ADDR) -A arm64 $^ $@

# Guest build rules
guest-$(BOARD).ifs: $(GUEST_BUILD)
	$(HOST_MKIFS) -v -r$(INSTALL) $(MKIFSFLAGS) $^ $@

# Clean
clean:
	$(RM_HOST) ifs-$(BOARD).* guest-$(BOARD).ifs *.sym

-include $(BUILD_TEMPLATE)/template.mk
