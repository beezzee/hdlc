DEVICE      = msp430f5529
CC          = msp430-gcc
OBJCOPY     = msp430-objcopy
AR          = msp430-ar
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
EXECUTABLE  = main
EXSRCDIR    = .
SRCDIR      = $(INCDIR)
OBJDIR      = objs
EXOBJECT    = $(OBJDIR)/$(EXECUTABLE).o
SOURCES     = $(wildcard $(SRCDIR)/*.c)
OBJECTS     = $(patsubst %.c,$(OBJDIR)/%.o,$(notdir $(SOURCES)))
######################################
INCLUDES    = -I. -I$(INCDIR)
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
