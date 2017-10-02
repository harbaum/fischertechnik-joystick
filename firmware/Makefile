#
#

CC = avr-gcc
F_CPU=16000000

APP=gamepad
OBJS=gamepad.o  usb_gamepad.o

MCU=atmega32u4

COPTS=-Wall -O2 -mmcu=$(MCU) -DF_CPU=$(F_CPU)

all: $(APP).hex

gamepad.o: usb_gamepad.h adctab.h
usb_gamepad.o: usb_gamepad.h

.c.o:
	$(CC) $(COPTS) -c $< -o $@

$(APP).elf: $(OBJS)
	$(CC) $(COPTS) $(OBJS) -o $(APP).elf

$(APP).hex: $(APP).elf
	avr-objcopy -R .eeprom -O ihex $(APP).elf $(APP).hex

# flash program directly into the avr overwriting any bootloader
flash: $(APP).hex
	avrdude -p m32u4 -P usb -c usbasp -U flash:w:$<

# flash using the arduino bootloader
program: $(APP).hex
	echo "Please press reset"
	sleep 3
	avrdude -pm32u4 -cavr109 -P/dev/ttyACM0 -b57600 -D -Uflash:w:$<

clean:
	rm -f $(APP).hex $(APP).elf *.o *~ 
