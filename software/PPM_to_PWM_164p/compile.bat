@echo off
del main.o
del main.elf
del main.hex
@echo on

make all

avrdude -p m164P -c usbasp -U flash:w:main.hex

pause