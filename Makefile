
# The name of your project (used to name the compiled .hex file)
TARGET = main

# configurable options
OPTIONS = -DF_CPU=48000000 -DF_BUS=48000000

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

OPT = -Os
CPPFLAGS = -Wall -MMD $(OPTIONS) -I.

# compiler options for C only
CFLAGS = -mcpu=cortex-m4 -mthumb -g $(OPT) -nostdlib -DUSBSERIAL_DEBUG

ASFLAGS = 

# linker options
LDFLAGS = $(OPT) -Wl,--gc-sections -Wl,--print-map -mcpu=cortex-m4 -mthumb -Tmk20dx128.ld

# additional libraries to link
LIBS = -lc -lm

# names for the compiler programs
AS = ./tools/arm-none-eabi/bin/arm-none-eabi-as
CC = ./tools/arm-none-eabi/bin/arm-none-eabi-gcc
OBJCOPY = ./tools/arm-none-eabi/bin/arm-none-eabi-objcopy
SIZE = ./tools/arm-none-eabi/bin/arm-none-eabi-size

SOURCES := main.c reset.c usb.c util.c i2c.c sensors.c
SSOURCES := crt0.s

OBJS := $(SOURCES:.c=.o)
SOBJS := $(SSOURCES:.s=.o)

$(SOBJS) : %.o : %.s
	$(AS) $(ASFLAGS) -c $< -o $@

# the actual makefile rules (all .o files built by GNU make's default implicit rules)

all: $(TARGET).hex
install: $(TARGET).hex

$(TARGET).elf: $(OBJS) $(SOBJS) mk20dx128.ld
	$(CC) $(LDFLAGS) $(LIBS) -o $@ $(OBJS) $(SOBJS) > $(TARGET).map

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# compiler generated dependency info
-include $(OBJS:.o=.d)

clean:
	rm -f *.o *.d $(TARGET).elf $(TARGET).hex $(TARGET).map


