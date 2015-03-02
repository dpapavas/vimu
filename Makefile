TARGET = main
OPTIONS = -DF_CPU=48000000 -DF_BUS=48000000

OPT = -Os
CPPFLAGS = -Wall -Winline -MMD $(OPTIONS) -I.
CFLAGS = -mcpu=cortex-m4 -mthumb -g $(OPT) -nostdlib
ASFLAGS = 
LDFLAGS = $(OPT) -Wl,--gc-sections -Wl,--print-map -mcpu=cortex-m4 -mthumb -Tmk20dx128.ld
LIBS = -lc -lm

AS = ./tools/arm-none-eabi/bin/arm-none-eabi-as
CC = ./tools/arm-none-eabi/bin/arm-none-eabi-gcc
OBJCOPY = ./tools/arm-none-eabi/bin/arm-none-eabi-objcopy
SIZE = ./tools/arm-none-eabi/bin/arm-none-eabi-size

SOURCES := main.c reset.c usb.c usbserial.c util.c i2c.c sdio.c \
	   sensors.c log.c console.c fusion.c button.c

SSOURCES := crt0.s

OBJS := $(SOURCES:.c=.o)
SOBJS := $(SSOURCES:.s=.o)

$(SOBJS) : %.o : %.s
	$(AS) $(ASFLAGS) -c $< -o $@

all: $(TARGET).hex
install: $(TARGET).hex
	./sendbreak /dev/ttyACM0
	./loader -mmcu=mk20dx128  -w -v $(TARGET).hex

$(TARGET).elf: $(OBJS) $(SOBJS) mk20dx128.ld
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(SOBJS) $(LIBS) > $(TARGET).map

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@

-include $(OBJS:.o=.d)

clean:
	rm -f *.o *.d $(TARGET).elf $(TARGET).hex $(TARGET).map


