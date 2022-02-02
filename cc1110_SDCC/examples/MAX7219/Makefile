CC = sdcc

CFLAGS = --model-small --opt-code-speed

# NOTE: code-loc should be the same as the value specified for
# USER_CODE_BASE in the bootloader!
LDFLAGS_FLASH = \
	--out-fmt-ihx \
	--code-loc 0x000 --code-size 0x8000 \
	--xram-loc 0xf000 --xram-size 0x300 \
	--iram-size 0x100

SRC =  max7219.c

ADB=$(SRC:.c=.adb)
ASM=$(SRC:.c=.asm)
LNK=$(SRC:.c=.lnk)
LST=$(SRC:.c=.lst)
REL=$(SRC:.c=.rel)
RST=$(SRC:.c=.rst)
SYM=$(SRC:.c=.sym)

PROGS=max7219.hex
PCDB=$(PROGS:.hex=.cdb)
PLNK=$(PROGS:.hex=.lnk)
PMAP=$(PROGS:.hex=.map)
PMEM=$(PROGS:.hex=.mem)
LK=$(PROGS:.hex=.lk)
PAOM=$(PROGS:.hex=)

%.rel : %.c
	$(CC) -c $(CFLAGS) -o$*.rel $<

all: $(PROGS) clean

max7219.hex: $(REL) Makefile
	$(CC) $(LDFLAGS_FLASH) $(CFLAGS) -o max7219.hex $(REL)

clean:
	rm -f $(ADB) $(ASM) $(LNK) $(LST) $(REL) $(RST) $(SYM) 
	rm -f $(PCDB) $(PLNK) $(PMAP) $(PMEM) $(PAOM) $(LK) 