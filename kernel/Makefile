FILENAME=aaa
ifneq (,$(wildcard /u/cs452/public/xdev))
XDIR:=/u/cs452/public/xdev
else ifneq (,$(wildcard ../../arm-gnu-toolchain-aarch64-none-elf))
XDIR:=../../arm-gnu-toolchain-aarch64-none-elf
else
$(error No valid XDIR found)
endif
TRIPLE=aarch64-none-elf
XBINDIR:=$(XDIR)/bin
CXX:=$(XBINDIR)/$(TRIPLE)-g++ -ffreestanding
OBJCOPY:=$(XBINDIR)/$(TRIPLE)-objcopy
OBJDUMP:=$(XBINDIR)/$(TRIPLE)-objdump

MMU?=on
OPT?=-O3
DATA_CACHE?=true
INSTRUCTION_CACHE?=true
PERF_TEST?=false
RPS_TEST?=false
CLOCK_TEST?=false
DATA_COLLECTION?=false

# COMPILE OPTIONS
ifeq ($(MMU),on)
MMUFLAGS:=-DMMU
else
MMUFLAGS:=-mstrict-align -mgeneral-regs-only
endif
WARNINGS:=-Wall -Wextra -Wpedantic -Wno-unused-const-variable
CXXFLAGS:=-g -pipe -static -march=armv8-a -mcpu=cortex-a72 -std=gnu++23 -DDATA_CACHE=$(DATA_CACHE) -DINSTRUCTION_CACHE=$(INSTRUCTION_CACHE) -DPERF_TEST=$(PERF_TEST) -DRPS_TEST=$(RPS_TEST) -DCLOCK_TEST=$(CLOCK_TEST) -DDATA_COLLECTION=$(DATA_COLLECTION) $(OPT) $(MMUFLAGS) $(WARNINGS)

# -Wl,option tells gcc to pass 'option' to the linker with commas replaced by spaces
# doing this rather than calling the linker directly simplifies the compilation procedure
LDFLAGS:=-Wl,-nmagic -Wl,-Tlinker.ld -Wl,--no-warn-rwx-segments -nostartfiles

# Source files and include dirs (recursive so files can live in subfolders)
SOURCES := $(shell find . -type f \( -name '*.S' -o -name '*.c' -o -name '*.cpp' \) | sed 's|^\./||' | sort)
INCLUDE_DIRS := $(shell find . -type d | sort)
CXXFLAGS += $(addprefix -I,$(INCLUDE_DIRS))

# Create .o and .d files for every source file.
OBJECTS := $(patsubst %.S, %.o, $(patsubst %.c, %.o, $(patsubst %.cpp, %.o, $(SOURCES))))
DEPENDS := $(patsubst %.S, %.d, $(patsubst %.c, %.d, $(patsubst %.cpp, %.d, $(SOURCES))))

.PHONY: all clean run

all: $(FILENAME).img

clean:
	rm -f $(OBJECTS) $(DEPENDS) $(FILENAME).elf $(FILENAME).img

run: $(FILENAME).img
	qemu-system-aarch64 -M raspi4b -cpu cortex-a53 -m 2G -nographic -kernel $(FILENAME).img

$(FILENAME).img: $(FILENAME).elf
	$(OBJCOPY) -S -O binary $< $@; sync

$(FILENAME).elf: $(OBJECTS) linker.ld
	$(CXX) $(CXXFLAGS) $(filter-out %.ld, $^) -o $@ $(LDFLAGS)
ifneq ($(MMU),on)
	@$(OBJDUMP) -d $@ | grep -Fq q0 && printf "\n***** WARNING: SIMD DETECTED! *****\n\n" || true
endif

%.o: %.S Makefile
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

%.o: %.c Makefile
	$(CXX) $(CXXFLAGS) -x c++ -MMD -MP -c $< -o $@

%.o: %.cpp Makefile
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

-include $(DEPENDS)
