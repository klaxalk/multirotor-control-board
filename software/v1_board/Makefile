APP=main
HEX=$(APP).hex
ELF=$(APP).elf
OBJS=main.o controllers.o communication.o system.o

# the MCU type
CPU=atmega164p

# frequency of the oscilator
CXXDEFINE=-DF_CPU=18432000

CXX=avr-gcc
OBJCOPY=avr-objcopy

CXXFLAGS=-Wall -O2 -funsigned-char -fshort-enums
AVRFLAGS=-mmcu=$(CPU) -Wl,-u,vfprintf -lprintf_flt

CXXOBJFLAGS=$(CXXFLAGS) $(AVRFLAGS) $(CXXDEFINE) -c

RM=rm -f

obj: $(OBJS)

.c.o:
	$(CXX) $(CXXOBJFLAGS) $<

elf: obj
	$(CXX) $(AVRFLAGS) $(OBJS) -lm -o $(ELF)

hex: elf
	$(OBJCOPY) -O ihex -R .eeprom $(ELF) $(HEX)

all: hex

lst:
	avr-objdump -h -S $(ELF) >$(APP).lst

clean:
	$(RM) $(HEX)
	$(RM) $(ELF)
	$(RM) $(OBJS)
