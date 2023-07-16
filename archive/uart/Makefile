file = main
DEVICE = atmega328p	
PORT = /dev/ttyACM0 
BAUD = 115200

default:
	#flash microprocessor
	avr-gcc -Wall -lm -Os -DF_CPU=16000000UL -mmcu=${DEVICE} -c $(file).c -o $(file).o 
	#avr-gcc -Wall -Os -DF_CPU=16000000UL -mmcu=${DEVICE} -c uart_lib.c -o uart_lib.o 
	avr-gcc -o $(file).bin $(file).o #uart_lib.o
	avr-objcopy -O ihex -R .eeprom $(file).bin $(file).hex
	avr-size --format=avr --mcu=${DEVICE} $(file).hex
	sudo avrdude -F -V -c arduino -p ${DEVICE} -P ${PORT} -b ${BAUD} -U flash:w:$(file).hex
	make clean

clean:
	rm -r *.o *.bin *.hex

