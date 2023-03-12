file = main
DEVICE = atmega328p	
PORT = /dev/ttyACM0 
BAUD = 115200

compile:
	avr-gcc -Wall -Os -DF_CPU=16000000UL -mmcu=${DEVICE} -c $(file).c -o $(file).o
	avr-gcc -o $(file).bin $(file).o
	avr-objcopy -O ihex -R .eeprom $(file).bin $(file).hex
	avr-size --format=avr --mcu=${DEVICE} $(file).bin
	sudo avrdude -F -V -c arduino -p ${DEVICE} -P ${PORT} -b ${BAUD} -U flash:w:$(file).hex
	make clean

clean:
	rm -r *.o *.bin *.hex

debug:
	avr-objdump -S $(file).bin

