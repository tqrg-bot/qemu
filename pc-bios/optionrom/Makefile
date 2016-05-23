all: build-all
# Dummy command so that make thinks it has done something
	@true

include ../../config-host.mak
include $(SRC_PATH)/rules.mak

$(call set-vpath, $(SRC_PATH)/pc-bios/optionrom)

.PHONY : all clean build-all

CFLAGS := -Wall -Wstrict-prototypes -Werror -fomit-frame-pointer -fno-builtin
CFLAGS += -I$(SRC_PATH)
CFLAGS += $(call cc-option, $(CFLAGS), -fno-stack-protector)
CFLAGS += $(CFLAGS_NOPIE)
ifdef CONFIG_COMPILER_M16
CFLAGS += -m16
else
# Attempt to work around the lack of support for -m16 in gcc < 4.9.
CFLAGS += -m32 -fno-toplevel-reorder
linuxboot_dma.o-cflags += -include code16gcc.h
endif
QEMU_CFLAGS = $(CFLAGS)

ASFLAGS += -32

build-all: multiboot.bin linuxboot.bin linuxboot_dma.bin kvmvapic.bin

# suppress auto-removal of intermediate files
.SECONDARY:

ifdef CONFIG_WIN32
LD_EMULATION = i386pe
else
LD_EMULATION = elf_i386
endif

%.img: %.o
	$(call quiet-command,$(LD) $(LDFLAGS_NOPIE) -m $(LD_EMULATION) -Ttext 0 -e _start -s -o $@ $<,"  Building $(TARGET_DIR)$@")

%.raw: %.img
	$(call quiet-command,$(OBJCOPY) -O binary -j .text $< $@,"  Building $(TARGET_DIR)$@")

%.bin: %.raw
	$(call quiet-command,$(PYTHON) $(SRC_PATH)/scripts/signrom.py $< $@,"  Signing $(TARGET_DIR)$@")

clean:
	rm -f *.o *.d *.raw *.img *.bin *~
