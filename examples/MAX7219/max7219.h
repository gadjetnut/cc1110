//MAX7219
#define REG_DECODE        0x09                        // "decode mode" register
#define REG_INTENSITY     0x0a                        // "intensity" register
#define REG_SCAN_LIMIT    0x0b                        // "scan limit" register
#define REG_SHUTDOWN      0x0c                        // "shutdown" register
#define REG_DISPLAY_TEST  0x0f                        // "display test" register

#define INTENSITY_MIN     0x00                        // minimum display intensity
#define INTENSITY_MAX     0x0f                        // maximum display intensity
#define MAX_DIN P1_7
#define MAX_CS P1_6 
#define MAX_CLK P1_5 

static const struct {
	char   ascii;
	char   segs;
} MAX7219_Font[] = {
  {'A',0b1110111},       
  {'B',0b0011111},       
  {'C',0b1001110},       
  {'D',0b0111101},       
  {'E',0b1001111},       
  {'F',0b1000111},       
  {'G',0b1011110},       
  {'H',0b0110111},       
  {'I',0b0110000},       
  {'J',0b0111100},       
  {'L',0b0001110},       
  {'N',0b1110110},       
  {'O',0b1111110},       
  {'P',0b1100111},       
  {'R',0b0000101},       
  {'S',0b1011011},       
  {'T',0b0001111},       
  {'U',0b0111110},       
  {'Y',0b0100111},       
  {'[',0b1001110},
  {']',0b1111000},
  {'_',0b0001000},       
  {'a',0b1110111},       
  {'b',0b0011111},       
  {'c',0b0001101},       
  {'d',0b0111101},       
  {'e',0b1001111},       
  {'f',0b1000111},       
  {'g',0b1011110},       
  {'h',0b0010111},       
  {'i',0b0010000},        
  {'j',0b0111100},       
  {'l',0b0001110},       
  {'n',0b0010101},       
  {'o',0b1111110},       
  {'p',0b1100111},       
  {'r',0b0000101},       
  {'s',0b1011011},       
  {'t',0b0001111},       
  {'u',0b0011100},       
  {'y',0b0100111},       
  {'-', 0x00},
  {' ', 0x00},
  {'0', 0x7e},
  {'1', 0x30},
  {'2', 0x6d},
  {'3', 0x79},
  {'4', 0x33},
  {'5', 0x5b},
  {'6', 0x5f},
  {'7', 0x70},
  {'8', 0x7f},
  {'9', 0x7b},
  {'\0', 0x00}
};
void MAX7219_Clear (void);
void MAX7219_DisplayChar (char digit, char character, uint8_t dp);
unsigned char MAX7219_LookupCode (char character, uint8_t dp);