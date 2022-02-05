CC = sdcc

CFLAGS = --model-small --opt-code-speed

LDFLAGS_FLASH = -L --out-fmt-ihx --code-loc 0x000 --code-size 0x7800 --xram-loc 0xf000 --xram-size 0x0f00 --iram-size 0x100 

SRC =  htu21.c ../../lib/delay.c ../../lib/i2c_bitbang.c

ADB=$(SRC:.c=.adb)
ASM=$(SRC:.c=.asm)
LNK=$(SRC:.c=.lnk)
LST=$(SRC:.c=.lst)
REL=$(SRC:.c=.rel)
RST=$(SRC:.c=.rst)
SYM=$(SRC:.c=.sym)

PROGS=htu21.hex
PCDB=$(PROGS:.hex=.cdb)
PLNK=$(PROGS:.hex=.lnk)
PMAP=$(PROGS:.hex=.map)
PMEM=$(PROGS:.hex=.mem)
LK=$(PROGS:.hex=.lk)
PAOM=$(PROGS:.hex=)

%.rel : %.c
	$(CC) -c $(CFLAGS) -o$*.rel $<

all: $(PROGS) clean

htu21.hex: $(REL) Makefile
	$(CC) $(LDFLAGS_FLASH) $(CFLAGS) -o htu21.hex $(REL)

clean:
	rm -f $(ADB) $(ASM) $(LNK) $(LST) $(REL) $(RST) $(SYM) 
	rm -f $(PCDB) $(PLNK) $(PMAP) $(PMEM) $(PAOM) $(LK) 