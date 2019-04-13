MCU = atmega328p
F_CPU = 16000000UL
BAUD = 9600UL

LIBDIR = #/home/nick/C/pthreads
LDLIBS = 
PROGRAMMER_TYPE = usbtiny

CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
AVRSIZE = avr-size
AVRDUDE = avrdude

# Overview of steps involved in generating final code that is sent
# to AVR microchip
# 1. pre-processor stage which substitutes vars and macros 
# in the final program
# 2. compilation stage which converts source code to assembly code
# 3. assembler stage converts assembly code to machine lang. opcode
# generates an object file '.obj', note that calls to functions (addresses)
# that are external are left undefined (to be filled in by linker)
# 4. linker stage combines object files together to generate
# an .exe file or ELF file


# name of target project
# grab 'lastword' of directory which should be target name
#  CURDIR is an alias for current directory 
# remove '/' in current directory and get last folder name
# assume folder name == target name
TARGET1 := $(subst /, , $(CURDIR))
TARGET = $(lastword $(TARGET1))


# grab all '.c' and '.h files' located in current folder
# and LIBDIR folder defined above
SOURCES = $(wildcard *.c )
SOURCES += $(wildcard $(LIBDIR)/*.c )

# convert all .c files to .o files
OBJECTS = $(SOURCES:.c=.o)

# convert all .c files to .h files
HEADERS = $(wildcard *.h )
HEADERS += $(wildcard $(LIBDIR)/*.h )

# compilation options

# pre-processor compilation
# -Idir option includes in search path directory for .h files
# include current directory '.' and 'LIBDIR'
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I. -I$(LIBDIR)

# compilation option
# 'Os' = enable optimization options for size
# 'Wall' = enable all warnings
# 'std' = lang. standard
# 'g' = enable debugging with db
CFLAGS = -Os -g -std=gnu99 -Wall

# more compiler options
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums 
CFLAGS += -ffunction-sections -fdata-sections 

# Linker options
# Wl = pass an option to the linker
# "-Map target.map" is the option to pass the linker
LDFLAGS = -Wl -Map,$(TARGET).map

# target archetecture
TARGET_ARCH = -mmcu=$(MCU)

# geneate .o object files for each .c files (preproc. + compile + assemble)
# name.o is generated from name.c + .h files in HEADERS
# -c = compile + assemble but do not link, one .obj for each .c file
# -o = place output file (here .obj file) in filename to right of i
# $@ = target file name (.o file)
# $<; = first pre-ref file name (.c file)
%.o:%.c $(HEADERS)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c -o $@ $<;

# link .obj files to generate an .elf file
# $^ = filenames of all the pre-reqs seperated by spaces (no duplicates)
# $@ = targt file name
$(TARGET).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(TARGET_ARCH) $^ $(LDLIBS) -o $@

# generate a .lst file, an assembly code w/ memory options
$(TARGET).lst: $(TARGET).elf
	$(OBJDUMP) -h -S $< > $@ 

# generate a .hex file from .elf files
%.hex:%.elf:
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

# make the entire .hex file
all: $(TARGET).hex

# clear all files except .c files
clear: 
	rm -r *.o *.obj *.hex *.el
