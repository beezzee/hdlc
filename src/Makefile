DEVICE      = MSP430F5529
CC          = msp430-elf-gcc
OBJCOPY     = msp430-elf-objcopy
AR          = msp430-elf-ar
OS 			= $(shell uname -o)
######################################
INCDIR      = ../../../../driverlib/MSP430F5xx_6xx
######################################
ifeq ($(OS),Cygwin)
GCCINCDIR   = $(shell cygpath -m $(dir $(shell which $(CC)))../../../../ccs_base/msp430/include_gcc)
endif
LDDIR       = $(GCCINCDIR)/$(shell echo $(DEVICE) | tr A-Z a-z)
CFLAGS      = -O2 -D__$(DEVICE)__ -mmcu=$(DEVICE) -ffunction-sections -fdata-sections
LDFLAGS     = -T $(LDDIR).ld -mmcu=$(DEVICE) -Wl,--gc-sections
######################################
EXECUTABLE  = adc12_a_ex5_repeatedSingle
EXSRCDIR    = ..
SRCDIR      = $(INCDIR)
OBJDIR      = objs
EXOBJECT    = $(OBJDIR)/$(EXECUTABLE).o
SOURCES     = $(wildcard $(SRCDIR)/*.c)
OBJECTS     = $(patsubst %.c,$(OBJDIR)/%.o,$(notdir $(SOURCES)))
######################################
INCLUDES    = -I$(GCCINCDIR) -I$(INCDIR)
######################################

all: init $(EXECUTABLE)

init:
	mkdir -p $(OBJDIR)

$(EXECUTABLE): $(EXOBJECT) $(OBJECTS)
	$(CC) $(LDFLAGS) $(EXOBJECT) $(OBJECTS) -o $@

$(EXOBJECT): $(EXSRCDIR)/$(EXECUTABLE).c
	$(CC) $(INCLUDES) $(CFLAGS) -c $< -o $@

$(OBJECTS): $(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(INCLUDES) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJDIR)
	rm -rf $(EXECUTABLE)
