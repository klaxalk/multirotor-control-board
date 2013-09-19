@echo off
del main.o
del communication.o
del controllers.o
del system.o
del main.elf
del main.hex
@echo on

make all
pause