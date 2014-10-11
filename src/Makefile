DEVICE      = msp430f5529
CC          = msp430-gcc
OBJCOPY     = msp430-objcopy
AR          = msp430-ar
OBJDUMP     = msp430-objdump
OS 			= $(shell uname -o)
######################################
INCDIR      = ../../driverlib_1_90_00_65/driverlib/MSP430F5xx_6xx
######################################
ifeq ($(OS),Cygwin)
GCCINCDIR   = $(shell cygpath -m $(dir $(shell which $(CC)))../../../../ccs_base/msp430/include_gcc)
endif
LDDIR       = $(GCCINCDIR)/$(shell echo $(DEVICE) | tr A-Z a-z)
CFLAGS      = -O2 -D__$(DEVICE)__ -mmcu=$(DEVICE) -ffunction-sections -fdata-sections
LDFLAGS     = -mmcu=$(DEVICE) -Wl,--gc-sections
######################################
EXECUTABLES = main
EXSRCDIR    = .
SRCDIR      = $(INCDIR)
OBJDIR      = objs
EXOBJECT    = $(OBJDIR)/$(EXECUTABLES).o
SOURCES     = $(notdir $(wildcard $(SRCDIR)/*.c) $(wildcard ./*.c)) 
OBJECTS     = $(SOURCES:.c=.o)
ELF         = $(addsuffix .elf,$(EXECUTABLES))
######################################
INCLUDES    = -I. -I$(INCDIR)
######################################

vpath %.c $(SRCDIR)

all:  $(ELF)

init:
	mkdir -p $(OBJDIR)

%.elf: $(OBJECTS)
	$(CC)  $(LDFLAGS) $^ -o $@

.SECONDARY: $(OBJECTS)
%.o: %.c 
	$(CC) $(INCLUDES) $(CFLAGS) -c $< -o $@

%.lst:  %.o
	$(OBJDUMP) -d -h -S -z $< > $@	

%.lss:  %.elf
	$(OBJDUMP) -d -h -S -z $< > $@	


.PHONY: clean
clean:
	rm -rf $(OBJECTS)
	rm -rf $(ELF)
	rm -rf *.lss
	rm -rf *.lst
