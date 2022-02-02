#ifndef INTERRUPT_H
#define INTERRUPT_H
/*==== CONSTS ================================================================*/

#define INT_ON   1
#define INT_OFF  0
#define INT_SET  1
#define INT_CLR  0

// Global interrupt enables (without taking care of the current EA value
#define INT_GLOBAL_ENABLE(on) EA = (!!on)

// Disabling the individual interrupts
#define DISABLE_ALL_INTERRUPTS() (IEN0 = IEN1 = IEN2 = 0x00)

// Pausing the global interrupt (save current state)
#define INT_ENTER_CRITICAL_SECTION(current_EA) \
   do {                                        \
      current_EA = EA;                         \
      EA = INT_OFF;                            \
   } while (0)

// Bringing global interrupt back to previous
// state (using value saved from INT_ENTER_CRITICAL_SECTION)
#define INT_LEAVE_CRITICAL_SECTION(previous_EA) EA = (!!previous_EA)

#define INUM_RFTXRX 0
#define INUM_ADC   1
#define INUM_URX0  2
#define INUM_URX1  3
#define INUM_ENC   4
#define INUM_ST    5
#define INUM_P2INT 6
#define INUM_UTX0  7
#define INUM_DMA   8
#define INUM_T1    9
#define INUM_T2    10
#define INUM_T3    11
#define INUM_T4    12
#define INUM_P0INT 13
#define INUM_UTX1  14
#define INUM_P1INT 15
#define INUM_RF    16
#define INUM_WDT   17


/*==== TYPES =================================================================*/

/*==== EXPORTS ===============================================================*/

/*==== MACROS=================================================================*/

// Macro used together with the INUM_* constants
// to enable or disable certain interrupts.
// Example usage:
//   HAL_INT_ENABLE(INUM_RFERR, INT_ON);
//   HAL_INT_ENABLE(INUM_URX0, INT_OFF);
//   HAL_INT_ENABLE(INUM_T1, INT_ON);
//   HAL_INT_ENABLE(INUM_T2, INT_OFF);

#define HAL_INT_ENABLE(inum, on)                                                \
   do {                                                                         \
      if      (inum==INUM_RFTXRX) { RFTXRXIE = on; }                            \
      else if (inum==INUM_ADC)    { ADCIE   = on;  }                            \
      else if (inum==INUM_URX0)   { URX0IE  = on;  }                            \
      else if (inum==INUM_URX1)   { URX1IE  = on;  }                            \
      else if (inum==INUM_ENC)    { ENCIE   = on;  }                            \
      else if (inum==INUM_ST)     { STIE    = on;  }                            \
      else if (inum==INUM_P2INT)  { (on) ? (IEN2 |= 0x02) : (IEN2 &= ~0x02); }  \
      else if (inum==INUM_UTX0)   { (on) ? (IEN2 |= 0x04) : (IEN2 &= ~0x04); }  \
      else if (inum==INUM_DMA)    { DMAIE   = on;  }                            \
      else if (inum==INUM_T1)     { T1IE    = on;  }                            \
      else if (inum==INUM_T2)     { T2IE    = on;  }                            \
      else if (inum==INUM_T3)     { T3IE    = on;  }                            \
      else if (inum==INUM_T4)     { T4IE    = on;  }                            \
      else if (inum==INUM_P0INT)  { P0IE    = on;  }                            \
      else if (inum==INUM_UTX1)   { (on) ? (IEN2 |= 0x08) : (IEN2 &= ~0x08); }  \
      else if (inum==INUM_P1INT)  { (on) ? (IEN2 |= 0x10) : (IEN2 &= ~0x10); }  \
      else if (inum==INUM_RF)     { (on) ? (IEN2 |= 0x01) : (IEN2 &= ~0x01); }  \
      else if (inum==INUM_WDT)    { (on) ? (IEN2 |= 0x20) : (IEN2 &= ~0x20); }  \
   } while (0)

/*==== FUNCTIONS =============================================================*/


/*==== END OF FILE ==========================================================*/

#endif