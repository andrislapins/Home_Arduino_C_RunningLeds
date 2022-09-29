arduinouno:
	avr-gcc -Os -DF_CPU=16000000 -mmcu=atmega328p -c main.c
	avr-gcc -DF_CPU=16000000 -mmcu=atmega328p -o main.elf main.o
	avr-objcopy -O ihex main.elf main.hex
	rm main.o
	rm main.elf
	avrdude -c arduino -p atmega328p -P /dev/ttyACM0 -U flash:w:main.hex
	stty -F /dev/ttyACM0 raw 9600
	cat /dev/ttyACM0

help:
	@echo 'Help details:'
	@echo 'program: compile hex and install'