                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 4.0.0 #11528 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module helloworld
                                      6 	.optsdcc -mmcs51 --model-small
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _main
                                     12 	.globl _rf_IRQ
                                     13 	.globl _delayms
                                     14 	.globl _radioConfigure
                                     15 	.globl _SetMainClkSrc
                                     16 	.globl _dmaRadioSetup
                                     17 	.globl ___memcpy
                                     18 	.globl _MODE
                                     19 	.globl _RE
                                     20 	.globl _SLAVE
                                     21 	.globl _FE
                                     22 	.globl _ERR
                                     23 	.globl _RX_BYTE
                                     24 	.globl _TX_BYTE
                                     25 	.globl _ACTIVE
                                     26 	.globl _B_7
                                     27 	.globl _B_6
                                     28 	.globl _B_5
                                     29 	.globl _B_4
                                     30 	.globl _B_3
                                     31 	.globl _B_2
                                     32 	.globl _B_1
                                     33 	.globl _B_0
                                     34 	.globl _WDTIF
                                     35 	.globl _P1IF
                                     36 	.globl _UTX1IF
                                     37 	.globl _UTX0IF
                                     38 	.globl _P2IF
                                     39 	.globl _ACC_7
                                     40 	.globl _ACC_6
                                     41 	.globl _ACC_5
                                     42 	.globl _ACC_4
                                     43 	.globl _ACC_3
                                     44 	.globl _ACC_2
                                     45 	.globl _ACC_1
                                     46 	.globl _ACC_0
                                     47 	.globl _OVFIM
                                     48 	.globl _T4CH1IF
                                     49 	.globl _T4CH0IF
                                     50 	.globl _T4OVFIF
                                     51 	.globl _T3CH1IF
                                     52 	.globl _T3CH0IF
                                     53 	.globl _T3OVFIF
                                     54 	.globl _CY
                                     55 	.globl _AC
                                     56 	.globl _F0
                                     57 	.globl _RS1
                                     58 	.globl _RS0
                                     59 	.globl _OV
                                     60 	.globl _F1
                                     61 	.globl _P
                                     62 	.globl _STIF
                                     63 	.globl _P0IF
                                     64 	.globl _T4IF
                                     65 	.globl _T3IF
                                     66 	.globl _T2IF
                                     67 	.globl _T1IF
                                     68 	.globl _DMAIF
                                     69 	.globl _P0IE
                                     70 	.globl _T4IE
                                     71 	.globl _T3IE
                                     72 	.globl _T2IE
                                     73 	.globl _T1IE
                                     74 	.globl _DMAIE
                                     75 	.globl _EA
                                     76 	.globl _STIE
                                     77 	.globl _ENCIE
                                     78 	.globl _URX1IE
                                     79 	.globl _URX0IE
                                     80 	.globl _ADCIE
                                     81 	.globl _RFTXRXIE
                                     82 	.globl _P2_7
                                     83 	.globl _P2_6
                                     84 	.globl _P2_5
                                     85 	.globl _P2_4
                                     86 	.globl _P2_3
                                     87 	.globl _P2_2
                                     88 	.globl _P2_1
                                     89 	.globl _P2_0
                                     90 	.globl _ENCIF_1
                                     91 	.globl _ENCIF_0
                                     92 	.globl _P1_7
                                     93 	.globl _P1_6
                                     94 	.globl _P1_5
                                     95 	.globl _P1_4
                                     96 	.globl _P1_3
                                     97 	.globl _P1_2
                                     98 	.globl _P1_1
                                     99 	.globl _P1_0
                                    100 	.globl _URX1IF
                                    101 	.globl _ADCIF
                                    102 	.globl _URX0IF
                                    103 	.globl _IT1
                                    104 	.globl _RFTXRXIF
                                    105 	.globl _IT0
                                    106 	.globl _P0_7
                                    107 	.globl _P0_6
                                    108 	.globl _P0_5
                                    109 	.globl _P0_4
                                    110 	.globl _P0_3
                                    111 	.globl _P0_2
                                    112 	.globl _P0_1
                                    113 	.globl _P0_0
                                    114 	.globl _P2DIR
                                    115 	.globl _P1DIR
                                    116 	.globl _P0DIR
                                    117 	.globl _U1GCR
                                    118 	.globl _U1UCR
                                    119 	.globl _U1BAUD
                                    120 	.globl _U1DBUF
                                    121 	.globl _U1CSR
                                    122 	.globl _P2INP
                                    123 	.globl _P1INP
                                    124 	.globl _P2SEL
                                    125 	.globl _P1SEL
                                    126 	.globl _P0SEL
                                    127 	.globl _ADCCFG
                                    128 	.globl _PERCFG
                                    129 	.globl _B
                                    130 	.globl _T4CC1
                                    131 	.globl _T4CCTL1
                                    132 	.globl _T4CC0
                                    133 	.globl _T4CCTL0
                                    134 	.globl _T4CTL
                                    135 	.globl _T4CNT
                                    136 	.globl _RFIF
                                    137 	.globl _IRCON2
                                    138 	.globl _T1CCTL2
                                    139 	.globl _T1CCTL1
                                    140 	.globl _T1CCTL0
                                    141 	.globl _T1CTL
                                    142 	.globl _T1CNTH
                                    143 	.globl _T1CNTL
                                    144 	.globl _RFST
                                    145 	.globl _ACC
                                    146 	.globl _T1CC2H
                                    147 	.globl _T1CC2L
                                    148 	.globl _T1CC1H
                                    149 	.globl _T1CC1L
                                    150 	.globl _T1CC0H
                                    151 	.globl _T1CC0L
                                    152 	.globl _RFD
                                    153 	.globl _TIMIF
                                    154 	.globl _DMAREQ
                                    155 	.globl _DMAARM
                                    156 	.globl _DMA0CFGH
                                    157 	.globl _DMA0CFGL
                                    158 	.globl _DMA1CFGH
                                    159 	.globl _DMA1CFGL
                                    160 	.globl _DMAIRQ
                                    161 	.globl _PSW
                                    162 	.globl _T3CC1
                                    163 	.globl _T3CCTL1
                                    164 	.globl _T3CC0
                                    165 	.globl _T3CCTL0
                                    166 	.globl _T3CTL
                                    167 	.globl _T3CNT
                                    168 	.globl _WDCTL
                                    169 	.globl __SFRC8
                                    170 	.globl _MEMCTR
                                    171 	.globl _CLKCON
                                    172 	.globl _U0GCR
                                    173 	.globl _U0UCR
                                    174 	.globl __SFRC3
                                    175 	.globl _U0BAUD
                                    176 	.globl _U0DBUF
                                    177 	.globl _IRCON
                                    178 	.globl __SFRBF
                                    179 	.globl _SLEEP
                                    180 	.globl _RNDH
                                    181 	.globl _RNDL
                                    182 	.globl _ADCH
                                    183 	.globl _ADCL
                                    184 	.globl _IP1
                                    185 	.globl _IEN1
                                    186 	.globl __SFRB7
                                    187 	.globl _ADCCON3
                                    188 	.globl _ADCCON2
                                    189 	.globl _ADCCON1
                                    190 	.globl _ENCCS
                                    191 	.globl _ENCDO
                                    192 	.globl _ENCDI
                                    193 	.globl __SFRB0
                                    194 	.globl _FWDATA
                                    195 	.globl _FCTL
                                    196 	.globl _FADDRH
                                    197 	.globl _FADDRL
                                    198 	.globl _FWT
                                    199 	.globl __SFRAA
                                    200 	.globl _IP0
                                    201 	.globl _IEN0
                                    202 	.globl __SFRA7
                                    203 	.globl _WORTIME1
                                    204 	.globl _WORTIME0
                                    205 	.globl _WOREVT1
                                    206 	.globl _WOREVT0
                                    207 	.globl _WORCTRL
                                    208 	.globl _WORIRQ
                                    209 	.globl _P2
                                    210 	.globl __SFR9F
                                    211 	.globl _T2CTL
                                    212 	.globl _T2PR
                                    213 	.globl _T2CT
                                    214 	.globl _S1CON
                                    215 	.globl _IEN2
                                    216 	.globl __SFR99
                                    217 	.globl _S0CON
                                    218 	.globl __SFR97
                                    219 	.globl __SFR96
                                    220 	.globl __SFR95
                                    221 	.globl __SFR94
                                    222 	.globl __XPAGE
                                    223 	.globl _MPAGE
                                    224 	.globl _DPS
                                    225 	.globl _RFIM
                                    226 	.globl _P1
                                    227 	.globl _P0INP
                                    228 	.globl __SFR8E
                                    229 	.globl _P1IEN
                                    230 	.globl _PICTL
                                    231 	.globl _P2IFG
                                    232 	.globl _P1IFG
                                    233 	.globl _P0IFG
                                    234 	.globl _TCON
                                    235 	.globl _PCON
                                    236 	.globl _U0CSR
                                    237 	.globl _DPH1
                                    238 	.globl _DPL1
                                    239 	.globl _DPH0
                                    240 	.globl _DPL0
                                    241 	.globl _SP
                                    242 	.globl _P0
                                    243 	.globl _radioPktBuffer
                                    244 	.globl _X_P2DIR
                                    245 	.globl _X_P1DIR
                                    246 	.globl _X_P0DIR
                                    247 	.globl _X_U1GCR
                                    248 	.globl _X_U1UCR
                                    249 	.globl _X_U1BAUD
                                    250 	.globl _X_U1DBUF
                                    251 	.globl _X_U1CSR
                                    252 	.globl _X_P2INP
                                    253 	.globl _X_P1INP
                                    254 	.globl _X_P2SEL
                                    255 	.globl _X_P1SEL
                                    256 	.globl _X_P0SEL
                                    257 	.globl _X_ADCCFG
                                    258 	.globl _X_PERCFG
                                    259 	.globl __NA_B
                                    260 	.globl _X_T4CC1
                                    261 	.globl _X_T4CCTL1
                                    262 	.globl _X_T4CC0
                                    263 	.globl _X_T4CCTL0
                                    264 	.globl _X_T4CTL
                                    265 	.globl _X_T4CNT
                                    266 	.globl _X_RFIF
                                    267 	.globl __NA_IRCON2
                                    268 	.globl _X_T1CCTL2
                                    269 	.globl _X_T1CCTL1
                                    270 	.globl _X_T1CCTL0
                                    271 	.globl _X_T1CTL
                                    272 	.globl _X_T1CNTH
                                    273 	.globl _X_T1CNTL
                                    274 	.globl _X_RFST
                                    275 	.globl __NA_ACC
                                    276 	.globl _X_T1CC2H
                                    277 	.globl _X_T1CC2L
                                    278 	.globl _X_T1CC1H
                                    279 	.globl _X_T1CC1L
                                    280 	.globl _X_T1CC0H
                                    281 	.globl _X_T1CC0L
                                    282 	.globl _X_RFD
                                    283 	.globl _X_TIMIF
                                    284 	.globl _X_DMAREQ
                                    285 	.globl _X_DMAARM
                                    286 	.globl _X_DMA0CFGH
                                    287 	.globl _X_DMA0CFGL
                                    288 	.globl _X_DMA1CFGH
                                    289 	.globl _X_DMA1CFGL
                                    290 	.globl _X_DMAIRQ
                                    291 	.globl __NA_PSW
                                    292 	.globl _X_T3CC1
                                    293 	.globl _X_T3CCTL1
                                    294 	.globl _X_T3CC0
                                    295 	.globl _X_T3CCTL0
                                    296 	.globl _X_T3CTL
                                    297 	.globl _X_T3CNT
                                    298 	.globl _X_WDCTL
                                    299 	.globl __X_SFRC8
                                    300 	.globl _X_MEMCTR
                                    301 	.globl _X_CLKCON
                                    302 	.globl _X_U0GCR
                                    303 	.globl _X_U0UCR
                                    304 	.globl __X_SFRC3
                                    305 	.globl _X_U0BAUD
                                    306 	.globl _X_U0DBUF
                                    307 	.globl __NA_IRCON
                                    308 	.globl __X_SFRBF
                                    309 	.globl _X_SLEEP
                                    310 	.globl _X_RNDH
                                    311 	.globl _X_RNDL
                                    312 	.globl _X_ADCH
                                    313 	.globl _X_ADCL
                                    314 	.globl __NA_IP1
                                    315 	.globl __NA_IEN1
                                    316 	.globl __X_SFRB7
                                    317 	.globl _X_ADCCON3
                                    318 	.globl _X_ADCCON2
                                    319 	.globl _X_ADCCON1
                                    320 	.globl _X_ENCCS
                                    321 	.globl _X_ENCDO
                                    322 	.globl _X_ENCDI
                                    323 	.globl __X_SFRB0
                                    324 	.globl _X_FWDATA
                                    325 	.globl _X_FCTL
                                    326 	.globl _X_FADDRH
                                    327 	.globl _X_FADDRL
                                    328 	.globl _X_FWT
                                    329 	.globl __X_SFRAA
                                    330 	.globl __NA_IP0
                                    331 	.globl __NA_IEN0
                                    332 	.globl __X_SFRA7
                                    333 	.globl _X_WORTIME1
                                    334 	.globl _X_WORTIME0
                                    335 	.globl _X_WOREVT1
                                    336 	.globl _X_WOREVT0
                                    337 	.globl _X_WORCTRL
                                    338 	.globl _X_WORIRQ
                                    339 	.globl __NA_P2
                                    340 	.globl __X_SFR9F
                                    341 	.globl _X_T2CTL
                                    342 	.globl _X_T2PR
                                    343 	.globl _X_T2CT
                                    344 	.globl __NA_S1CON
                                    345 	.globl __NA_IEN2
                                    346 	.globl __X_SFR99
                                    347 	.globl __NA_S0CON
                                    348 	.globl __X_SFR97
                                    349 	.globl __X_SFR96
                                    350 	.globl __X_SFR95
                                    351 	.globl __X_SFR94
                                    352 	.globl _X_MPAGE
                                    353 	.globl __NA_DPS
                                    354 	.globl _X_RFIM
                                    355 	.globl __NA_P1
                                    356 	.globl _X_P0INP
                                    357 	.globl __X_SFR8E
                                    358 	.globl _X_P1IEN
                                    359 	.globl _X_PICTL
                                    360 	.globl _X_P2IFG
                                    361 	.globl _X_P1IFG
                                    362 	.globl _X_P0IFG
                                    363 	.globl __NA_TCON
                                    364 	.globl __NA_PCON
                                    365 	.globl _X_U0CSR
                                    366 	.globl __NA_DPH1
                                    367 	.globl __NA_DPL1
                                    368 	.globl __NA_DPH0
                                    369 	.globl __NA_DPL0
                                    370 	.globl __NA_SP
                                    371 	.globl __NA_P0
                                    372 	.globl _I2SCLKF2
                                    373 	.globl _I2SCLKF1
                                    374 	.globl _I2SCLKF0
                                    375 	.globl _I2SSTAT
                                    376 	.globl _I2SWCNT
                                    377 	.globl _I2SDATH
                                    378 	.globl _I2SDATL
                                    379 	.globl _I2SCFG1
                                    380 	.globl _I2SCFG0
                                    381 	.globl _VCO_VC_DAC
                                    382 	.globl _PKTSTATUS
                                    383 	.globl _MARCSTATE
                                    384 	.globl _RSSI
                                    385 	.globl _LQI
                                    386 	.globl _FREQEST
                                    387 	.globl _VERSION
                                    388 	.globl _PARTNUM
                                    389 	.globl __XREGDF35
                                    390 	.globl __XREGDF34
                                    391 	.globl __XREGDF33
                                    392 	.globl __XREGDF32
                                    393 	.globl _IOCFG0
                                    394 	.globl _IOCFG1
                                    395 	.globl _IOCFG2
                                    396 	.globl _PA_TABLE0
                                    397 	.globl _PA_TABLE1
                                    398 	.globl _PA_TABLE2
                                    399 	.globl _PA_TABLE3
                                    400 	.globl _PA_TABLE4
                                    401 	.globl _PA_TABLE5
                                    402 	.globl _PA_TABLE6
                                    403 	.globl _PA_TABLE7
                                    404 	.globl __XREGDF26
                                    405 	.globl _TEST0
                                    406 	.globl _TEST1
                                    407 	.globl _TEST2
                                    408 	.globl __XREGDF22
                                    409 	.globl __XREGDF21
                                    410 	.globl __XREGDF20
                                    411 	.globl _FSCAL0
                                    412 	.globl _FSCAL1
                                    413 	.globl _FSCAL2
                                    414 	.globl _FSCAL3
                                    415 	.globl _FREND0
                                    416 	.globl _FREND1
                                    417 	.globl _AGCCTRL0
                                    418 	.globl _AGCCTRL1
                                    419 	.globl _AGCCTRL2
                                    420 	.globl _BSCFG
                                    421 	.globl _FOCCFG
                                    422 	.globl _MCSM0
                                    423 	.globl _MCSM1
                                    424 	.globl _MCSM2
                                    425 	.globl _DEVIATN
                                    426 	.globl _MDMCFG0
                                    427 	.globl _MDMCFG1
                                    428 	.globl _MDMCFG2
                                    429 	.globl _MDMCFG3
                                    430 	.globl _MDMCFG4
                                    431 	.globl _FREQ0
                                    432 	.globl _FREQ1
                                    433 	.globl _FREQ2
                                    434 	.globl _FSCTRL0
                                    435 	.globl _FSCTRL1
                                    436 	.globl _CHANNR
                                    437 	.globl _ADDR
                                    438 	.globl _PKTCTRL0
                                    439 	.globl _PKTCTRL1
                                    440 	.globl _PKTLEN
                                    441 	.globl _SYNC0
                                    442 	.globl _SYNC1
                                    443 	.globl _MDMCTRL0H
                                    444 ;--------------------------------------------------------
                                    445 ; special function registers
                                    446 ;--------------------------------------------------------
                                    447 	.area RSEG    (ABS,DATA)
      000000                        448 	.org 0x0000
                           000080   449 _P0	=	0x0080
                           000081   450 _SP	=	0x0081
                           000082   451 _DPL0	=	0x0082
                           000083   452 _DPH0	=	0x0083
                           000084   453 _DPL1	=	0x0084
                           000085   454 _DPH1	=	0x0085
                           000086   455 _U0CSR	=	0x0086
                           000087   456 _PCON	=	0x0087
                           000088   457 _TCON	=	0x0088
                           000089   458 _P0IFG	=	0x0089
                           00008A   459 _P1IFG	=	0x008a
                           00008B   460 _P2IFG	=	0x008b
                           00008C   461 _PICTL	=	0x008c
                           00008D   462 _P1IEN	=	0x008d
                           00008E   463 __SFR8E	=	0x008e
                           00008F   464 _P0INP	=	0x008f
                           000090   465 _P1	=	0x0090
                           000091   466 _RFIM	=	0x0091
                           000092   467 _DPS	=	0x0092
                           000093   468 _MPAGE	=	0x0093
                           000093   469 __XPAGE	=	0x0093
                           000094   470 __SFR94	=	0x0094
                           000095   471 __SFR95	=	0x0095
                           000096   472 __SFR96	=	0x0096
                           000097   473 __SFR97	=	0x0097
                           000098   474 _S0CON	=	0x0098
                           000099   475 __SFR99	=	0x0099
                           00009A   476 _IEN2	=	0x009a
                           00009B   477 _S1CON	=	0x009b
                           00009C   478 _T2CT	=	0x009c
                           00009D   479 _T2PR	=	0x009d
                           00009E   480 _T2CTL	=	0x009e
                           00009F   481 __SFR9F	=	0x009f
                           0000A0   482 _P2	=	0x00a0
                           0000A1   483 _WORIRQ	=	0x00a1
                           0000A2   484 _WORCTRL	=	0x00a2
                           0000A3   485 _WOREVT0	=	0x00a3
                           0000A4   486 _WOREVT1	=	0x00a4
                           0000A5   487 _WORTIME0	=	0x00a5
                           0000A6   488 _WORTIME1	=	0x00a6
                           0000A7   489 __SFRA7	=	0x00a7
                           0000A8   490 _IEN0	=	0x00a8
                           0000A9   491 _IP0	=	0x00a9
                           0000AA   492 __SFRAA	=	0x00aa
                           0000AB   493 _FWT	=	0x00ab
                           0000AC   494 _FADDRL	=	0x00ac
                           0000AD   495 _FADDRH	=	0x00ad
                           0000AE   496 _FCTL	=	0x00ae
                           0000AF   497 _FWDATA	=	0x00af
                           0000B0   498 __SFRB0	=	0x00b0
                           0000B1   499 _ENCDI	=	0x00b1
                           0000B2   500 _ENCDO	=	0x00b2
                           0000B3   501 _ENCCS	=	0x00b3
                           0000B4   502 _ADCCON1	=	0x00b4
                           0000B5   503 _ADCCON2	=	0x00b5
                           0000B6   504 _ADCCON3	=	0x00b6
                           0000B7   505 __SFRB7	=	0x00b7
                           0000B8   506 _IEN1	=	0x00b8
                           0000B9   507 _IP1	=	0x00b9
                           0000BA   508 _ADCL	=	0x00ba
                           0000BB   509 _ADCH	=	0x00bb
                           0000BC   510 _RNDL	=	0x00bc
                           0000BD   511 _RNDH	=	0x00bd
                           0000BE   512 _SLEEP	=	0x00be
                           0000BF   513 __SFRBF	=	0x00bf
                           0000C0   514 _IRCON	=	0x00c0
                           0000C1   515 _U0DBUF	=	0x00c1
                           0000C2   516 _U0BAUD	=	0x00c2
                           0000C3   517 __SFRC3	=	0x00c3
                           0000C4   518 _U0UCR	=	0x00c4
                           0000C5   519 _U0GCR	=	0x00c5
                           0000C6   520 _CLKCON	=	0x00c6
                           0000C7   521 _MEMCTR	=	0x00c7
                           0000C8   522 __SFRC8	=	0x00c8
                           0000C9   523 _WDCTL	=	0x00c9
                           0000CA   524 _T3CNT	=	0x00ca
                           0000CB   525 _T3CTL	=	0x00cb
                           0000CC   526 _T3CCTL0	=	0x00cc
                           0000CD   527 _T3CC0	=	0x00cd
                           0000CE   528 _T3CCTL1	=	0x00ce
                           0000CF   529 _T3CC1	=	0x00cf
                           0000D0   530 _PSW	=	0x00d0
                           0000D1   531 _DMAIRQ	=	0x00d1
                           0000D2   532 _DMA1CFGL	=	0x00d2
                           0000D3   533 _DMA1CFGH	=	0x00d3
                           0000D4   534 _DMA0CFGL	=	0x00d4
                           0000D5   535 _DMA0CFGH	=	0x00d5
                           0000D6   536 _DMAARM	=	0x00d6
                           0000D7   537 _DMAREQ	=	0x00d7
                           0000D8   538 _TIMIF	=	0x00d8
                           0000D9   539 _RFD	=	0x00d9
                           0000DA   540 _T1CC0L	=	0x00da
                           0000DB   541 _T1CC0H	=	0x00db
                           0000DC   542 _T1CC1L	=	0x00dc
                           0000DD   543 _T1CC1H	=	0x00dd
                           0000DE   544 _T1CC2L	=	0x00de
                           0000DF   545 _T1CC2H	=	0x00df
                           0000E0   546 _ACC	=	0x00e0
                           0000E1   547 _RFST	=	0x00e1
                           0000E2   548 _T1CNTL	=	0x00e2
                           0000E3   549 _T1CNTH	=	0x00e3
                           0000E4   550 _T1CTL	=	0x00e4
                           0000E5   551 _T1CCTL0	=	0x00e5
                           0000E6   552 _T1CCTL1	=	0x00e6
                           0000E7   553 _T1CCTL2	=	0x00e7
                           0000E8   554 _IRCON2	=	0x00e8
                           0000E9   555 _RFIF	=	0x00e9
                           0000EA   556 _T4CNT	=	0x00ea
                           0000EB   557 _T4CTL	=	0x00eb
                           0000EC   558 _T4CCTL0	=	0x00ec
                           0000ED   559 _T4CC0	=	0x00ed
                           0000EE   560 _T4CCTL1	=	0x00ee
                           0000EF   561 _T4CC1	=	0x00ef
                           0000F0   562 _B	=	0x00f0
                           0000F1   563 _PERCFG	=	0x00f1
                           0000F2   564 _ADCCFG	=	0x00f2
                           0000F3   565 _P0SEL	=	0x00f3
                           0000F4   566 _P1SEL	=	0x00f4
                           0000F5   567 _P2SEL	=	0x00f5
                           0000F6   568 _P1INP	=	0x00f6
                           0000F7   569 _P2INP	=	0x00f7
                           0000F8   570 _U1CSR	=	0x00f8
                           0000F9   571 _U1DBUF	=	0x00f9
                           0000FA   572 _U1BAUD	=	0x00fa
                           0000FB   573 _U1UCR	=	0x00fb
                           0000FC   574 _U1GCR	=	0x00fc
                           0000FD   575 _P0DIR	=	0x00fd
                           0000FE   576 _P1DIR	=	0x00fe
                           0000FF   577 _P2DIR	=	0x00ff
                                    578 ;--------------------------------------------------------
                                    579 ; special function bits
                                    580 ;--------------------------------------------------------
                                    581 	.area RSEG    (ABS,DATA)
      000000                        582 	.org 0x0000
                           000080   583 _P0_0	=	0x0080
                           000081   584 _P0_1	=	0x0081
                           000082   585 _P0_2	=	0x0082
                           000083   586 _P0_3	=	0x0083
                           000084   587 _P0_4	=	0x0084
                           000085   588 _P0_5	=	0x0085
                           000086   589 _P0_6	=	0x0086
                           000087   590 _P0_7	=	0x0087
                           000088   591 _IT0	=	0x0088
                           000089   592 _RFTXRXIF	=	0x0089
                           00008A   593 _IT1	=	0x008a
                           00008B   594 _URX0IF	=	0x008b
                           00008D   595 _ADCIF	=	0x008d
                           00008F   596 _URX1IF	=	0x008f
                           000090   597 _P1_0	=	0x0090
                           000091   598 _P1_1	=	0x0091
                           000092   599 _P1_2	=	0x0092
                           000093   600 _P1_3	=	0x0093
                           000094   601 _P1_4	=	0x0094
                           000095   602 _P1_5	=	0x0095
                           000096   603 _P1_6	=	0x0096
                           000097   604 _P1_7	=	0x0097
                           000098   605 _ENCIF_0	=	0x0098
                           000099   606 _ENCIF_1	=	0x0099
                           0000A0   607 _P2_0	=	0x00a0
                           0000A1   608 _P2_1	=	0x00a1
                           0000A2   609 _P2_2	=	0x00a2
                           0000A3   610 _P2_3	=	0x00a3
                           0000A4   611 _P2_4	=	0x00a4
                           0000A5   612 _P2_5	=	0x00a5
                           0000A6   613 _P2_6	=	0x00a6
                           0000A7   614 _P2_7	=	0x00a7
                           0000A8   615 _RFTXRXIE	=	0x00a8
                           0000A9   616 _ADCIE	=	0x00a9
                           0000AA   617 _URX0IE	=	0x00aa
                           0000AB   618 _URX1IE	=	0x00ab
                           0000AC   619 _ENCIE	=	0x00ac
                           0000AD   620 _STIE	=	0x00ad
                           0000AF   621 _EA	=	0x00af
                           0000B8   622 _DMAIE	=	0x00b8
                           0000B9   623 _T1IE	=	0x00b9
                           0000BA   624 _T2IE	=	0x00ba
                           0000BB   625 _T3IE	=	0x00bb
                           0000BC   626 _T4IE	=	0x00bc
                           0000BD   627 _P0IE	=	0x00bd
                           0000C0   628 _DMAIF	=	0x00c0
                           0000C1   629 _T1IF	=	0x00c1
                           0000C2   630 _T2IF	=	0x00c2
                           0000C3   631 _T3IF	=	0x00c3
                           0000C4   632 _T4IF	=	0x00c4
                           0000C5   633 _P0IF	=	0x00c5
                           0000C7   634 _STIF	=	0x00c7
                           0000D0   635 _P	=	0x00d0
                           0000D1   636 _F1	=	0x00d1
                           0000D2   637 _OV	=	0x00d2
                           0000D3   638 _RS0	=	0x00d3
                           0000D4   639 _RS1	=	0x00d4
                           0000D5   640 _F0	=	0x00d5
                           0000D6   641 _AC	=	0x00d6
                           0000D7   642 _CY	=	0x00d7
                           0000D8   643 _T3OVFIF	=	0x00d8
                           0000D9   644 _T3CH0IF	=	0x00d9
                           0000DA   645 _T3CH1IF	=	0x00da
                           0000DB   646 _T4OVFIF	=	0x00db
                           0000DC   647 _T4CH0IF	=	0x00dc
                           0000DD   648 _T4CH1IF	=	0x00dd
                           0000DE   649 _OVFIM	=	0x00de
                           0000E0   650 _ACC_0	=	0x00e0
                           0000E1   651 _ACC_1	=	0x00e1
                           0000E2   652 _ACC_2	=	0x00e2
                           0000E3   653 _ACC_3	=	0x00e3
                           0000E4   654 _ACC_4	=	0x00e4
                           0000E5   655 _ACC_5	=	0x00e5
                           0000E6   656 _ACC_6	=	0x00e6
                           0000E7   657 _ACC_7	=	0x00e7
                           0000E8   658 _P2IF	=	0x00e8
                           0000E9   659 _UTX0IF	=	0x00e9
                           0000EA   660 _UTX1IF	=	0x00ea
                           0000EB   661 _P1IF	=	0x00eb
                           0000EC   662 _WDTIF	=	0x00ec
                           0000F0   663 _B_0	=	0x00f0
                           0000F1   664 _B_1	=	0x00f1
                           0000F2   665 _B_2	=	0x00f2
                           0000F3   666 _B_3	=	0x00f3
                           0000F4   667 _B_4	=	0x00f4
                           0000F5   668 _B_5	=	0x00f5
                           0000F6   669 _B_6	=	0x00f6
                           0000F7   670 _B_7	=	0x00f7
                           0000F8   671 _ACTIVE	=	0x00f8
                           0000F9   672 _TX_BYTE	=	0x00f9
                           0000FA   673 _RX_BYTE	=	0x00fa
                           0000FB   674 _ERR	=	0x00fb
                           0000FC   675 _FE	=	0x00fc
                           0000FD   676 _SLAVE	=	0x00fd
                           0000FE   677 _RE	=	0x00fe
                           0000FF   678 _MODE	=	0x00ff
                                    679 ;--------------------------------------------------------
                                    680 ; overlayable register banks
                                    681 ;--------------------------------------------------------
                                    682 	.area REG_BANK_0	(REL,OVR,DATA)
      000000                        683 	.ds 8
                                    684 ;--------------------------------------------------------
                                    685 ; internal ram data
                                    686 ;--------------------------------------------------------
                                    687 	.area DSEG    (DATA)
      00000B                        688 _pktSentFlag:
      00000B                        689 	.ds 1
      00000C                        690 _pktRcvdFlag:
      00000C                        691 	.ds 1
      00000D                        692 _mode:
      00000D                        693 	.ds 1
      00000E                        694 _main_preamble_65536_37:
      00000E                        695 	.ds 3
                                    696 ;--------------------------------------------------------
                                    697 ; overlayable items in internal ram 
                                    698 ;--------------------------------------------------------
                                    699 ;--------------------------------------------------------
                                    700 ; Stack segment in internal ram 
                                    701 ;--------------------------------------------------------
                                    702 	.area	SSEG
      00001C                        703 __start__stack:
      00001C                        704 	.ds	1
                                    705 
                                    706 ;--------------------------------------------------------
                                    707 ; indirectly addressable internal ram data
                                    708 ;--------------------------------------------------------
                                    709 	.area ISEG    (DATA)
                                    710 ;--------------------------------------------------------
                                    711 ; absolute internal ram data
                                    712 ;--------------------------------------------------------
                                    713 	.area IABS    (ABS,DATA)
                                    714 	.area IABS    (ABS,DATA)
                                    715 ;--------------------------------------------------------
                                    716 ; bit data
                                    717 ;--------------------------------------------------------
                                    718 	.area BSEG    (BIT)
                                    719 ;--------------------------------------------------------
                                    720 ; paged external ram data
                                    721 ;--------------------------------------------------------
                                    722 	.area PSEG    (PAG,XDATA)
                                    723 ;--------------------------------------------------------
                                    724 ; external ram data
                                    725 ;--------------------------------------------------------
                                    726 	.area XSEG    (XDATA)
                           00DF02   727 _MDMCTRL0H	=	0xdf02
                           00DF00   728 _SYNC1	=	0xdf00
                           00DF01   729 _SYNC0	=	0xdf01
                           00DF02   730 _PKTLEN	=	0xdf02
                           00DF03   731 _PKTCTRL1	=	0xdf03
                           00DF04   732 _PKTCTRL0	=	0xdf04
                           00DF05   733 _ADDR	=	0xdf05
                           00DF06   734 _CHANNR	=	0xdf06
                           00DF07   735 _FSCTRL1	=	0xdf07
                           00DF08   736 _FSCTRL0	=	0xdf08
                           00DF09   737 _FREQ2	=	0xdf09
                           00DF0A   738 _FREQ1	=	0xdf0a
                           00DF0B   739 _FREQ0	=	0xdf0b
                           00DF0C   740 _MDMCFG4	=	0xdf0c
                           00DF0D   741 _MDMCFG3	=	0xdf0d
                           00DF0E   742 _MDMCFG2	=	0xdf0e
                           00DF0F   743 _MDMCFG1	=	0xdf0f
                           00DF10   744 _MDMCFG0	=	0xdf10
                           00DF11   745 _DEVIATN	=	0xdf11
                           00DF12   746 _MCSM2	=	0xdf12
                           00DF13   747 _MCSM1	=	0xdf13
                           00DF14   748 _MCSM0	=	0xdf14
                           00DF15   749 _FOCCFG	=	0xdf15
                           00DF16   750 _BSCFG	=	0xdf16
                           00DF17   751 _AGCCTRL2	=	0xdf17
                           00DF18   752 _AGCCTRL1	=	0xdf18
                           00DF19   753 _AGCCTRL0	=	0xdf19
                           00DF1A   754 _FREND1	=	0xdf1a
                           00DF1B   755 _FREND0	=	0xdf1b
                           00DF1C   756 _FSCAL3	=	0xdf1c
                           00DF1D   757 _FSCAL2	=	0xdf1d
                           00DF1E   758 _FSCAL1	=	0xdf1e
                           00DF1F   759 _FSCAL0	=	0xdf1f
                           00DF20   760 __XREGDF20	=	0xdf20
                           00DF21   761 __XREGDF21	=	0xdf21
                           00DF22   762 __XREGDF22	=	0xdf22
                           00DF23   763 _TEST2	=	0xdf23
                           00DF24   764 _TEST1	=	0xdf24
                           00DF25   765 _TEST0	=	0xdf25
                           00DF26   766 __XREGDF26	=	0xdf26
                           00DF27   767 _PA_TABLE7	=	0xdf27
                           00DF28   768 _PA_TABLE6	=	0xdf28
                           00DF29   769 _PA_TABLE5	=	0xdf29
                           00DF2A   770 _PA_TABLE4	=	0xdf2a
                           00DF2B   771 _PA_TABLE3	=	0xdf2b
                           00DF2C   772 _PA_TABLE2	=	0xdf2c
                           00DF2D   773 _PA_TABLE1	=	0xdf2d
                           00DF2E   774 _PA_TABLE0	=	0xdf2e
                           00DF2F   775 _IOCFG2	=	0xdf2f
                           00DF30   776 _IOCFG1	=	0xdf30
                           00DF31   777 _IOCFG0	=	0xdf31
                           00DF32   778 __XREGDF32	=	0xdf32
                           00DF33   779 __XREGDF33	=	0xdf33
                           00DF34   780 __XREGDF34	=	0xdf34
                           00DF35   781 __XREGDF35	=	0xdf35
                           00DF36   782 _PARTNUM	=	0xdf36
                           00DF37   783 _VERSION	=	0xdf37
                           00DF38   784 _FREQEST	=	0xdf38
                           00DF39   785 _LQI	=	0xdf39
                           00DF3A   786 _RSSI	=	0xdf3a
                           00DF3B   787 _MARCSTATE	=	0xdf3b
                           00DF3C   788 _PKTSTATUS	=	0xdf3c
                           00DF3D   789 _VCO_VC_DAC	=	0xdf3d
                           00DF40   790 _I2SCFG0	=	0xdf40
                           00DF41   791 _I2SCFG1	=	0xdf41
                           00DF42   792 _I2SDATL	=	0xdf42
                           00DF43   793 _I2SDATH	=	0xdf43
                           00DF44   794 _I2SWCNT	=	0xdf44
                           00DF45   795 _I2SSTAT	=	0xdf45
                           00DF46   796 _I2SCLKF0	=	0xdf46
                           00DF47   797 _I2SCLKF1	=	0xdf47
                           00DF48   798 _I2SCLKF2	=	0xdf48
                           00DF80   799 __NA_P0	=	0xdf80
                           00DF81   800 __NA_SP	=	0xdf81
                           00DF82   801 __NA_DPL0	=	0xdf82
                           00DF83   802 __NA_DPH0	=	0xdf83
                           00DF84   803 __NA_DPL1	=	0xdf84
                           00DF85   804 __NA_DPH1	=	0xdf85
                           00DF86   805 _X_U0CSR	=	0xdf86
                           00DF87   806 __NA_PCON	=	0xdf87
                           00DF88   807 __NA_TCON	=	0xdf88
                           00DF89   808 _X_P0IFG	=	0xdf89
                           00DF8A   809 _X_P1IFG	=	0xdf8a
                           00DF8B   810 _X_P2IFG	=	0xdf8b
                           00DF8C   811 _X_PICTL	=	0xdf8c
                           00DF8D   812 _X_P1IEN	=	0xdf8d
                           00DF8E   813 __X_SFR8E	=	0xdf8e
                           00DF8F   814 _X_P0INP	=	0xdf8f
                           00DF90   815 __NA_P1	=	0xdf90
                           00DF91   816 _X_RFIM	=	0xdf91
                           00DF92   817 __NA_DPS	=	0xdf92
                           00DF93   818 _X_MPAGE	=	0xdf93
                           00DF94   819 __X_SFR94	=	0xdf94
                           00DF95   820 __X_SFR95	=	0xdf95
                           00DF96   821 __X_SFR96	=	0xdf96
                           00DF97   822 __X_SFR97	=	0xdf97
                           00DF98   823 __NA_S0CON	=	0xdf98
                           00DF99   824 __X_SFR99	=	0xdf99
                           00DF9A   825 __NA_IEN2	=	0xdf9a
                           00DF9B   826 __NA_S1CON	=	0xdf9b
                           00DF9C   827 _X_T2CT	=	0xdf9c
                           00DF9D   828 _X_T2PR	=	0xdf9d
                           00DF9E   829 _X_T2CTL	=	0xdf9e
                           00DF9F   830 __X_SFR9F	=	0xdf9f
                           00DFA0   831 __NA_P2	=	0xdfa0
                           00DFA1   832 _X_WORIRQ	=	0xdfa1
                           00DFA2   833 _X_WORCTRL	=	0xdfa2
                           00DFA3   834 _X_WOREVT0	=	0xdfa3
                           00DFA4   835 _X_WOREVT1	=	0xdfa4
                           00DFA5   836 _X_WORTIME0	=	0xdfa5
                           00DFA6   837 _X_WORTIME1	=	0xdfa6
                           00DFA7   838 __X_SFRA7	=	0xdfa7
                           00DFA8   839 __NA_IEN0	=	0xdfa8
                           00DFA9   840 __NA_IP0	=	0xdfa9
                           00DFAA   841 __X_SFRAA	=	0xdfaa
                           00DFAB   842 _X_FWT	=	0xdfab
                           00DFAC   843 _X_FADDRL	=	0xdfac
                           00DFAD   844 _X_FADDRH	=	0xdfad
                           00DFAE   845 _X_FCTL	=	0xdfae
                           00DFAF   846 _X_FWDATA	=	0xdfaf
                           00DFB0   847 __X_SFRB0	=	0xdfb0
                           00DFB1   848 _X_ENCDI	=	0xdfb1
                           00DFB2   849 _X_ENCDO	=	0xdfb2
                           00DFB3   850 _X_ENCCS	=	0xdfb3
                           00DFB4   851 _X_ADCCON1	=	0xdfb4
                           00DFB5   852 _X_ADCCON2	=	0xdfb5
                           00DFB6   853 _X_ADCCON3	=	0xdfb6
                           00DFB7   854 __X_SFRB7	=	0xdfb7
                           00DFB8   855 __NA_IEN1	=	0xdfb8
                           00DFB9   856 __NA_IP1	=	0xdfb9
                           00DFBA   857 _X_ADCL	=	0xdfba
                           00DFBB   858 _X_ADCH	=	0xdfbb
                           00DFBC   859 _X_RNDL	=	0xdfbc
                           00DFBD   860 _X_RNDH	=	0xdfbd
                           00DFBE   861 _X_SLEEP	=	0xdfbe
                           00DFBF   862 __X_SFRBF	=	0xdfbf
                           00DFC0   863 __NA_IRCON	=	0xdfc0
                           00DFC1   864 _X_U0DBUF	=	0xdfc1
                           00DFC2   865 _X_U0BAUD	=	0xdfc2
                           00DFC3   866 __X_SFRC3	=	0xdfc3
                           00DFC4   867 _X_U0UCR	=	0xdfc4
                           00DFC5   868 _X_U0GCR	=	0xdfc5
                           00DFC6   869 _X_CLKCON	=	0xdfc6
                           00DFC7   870 _X_MEMCTR	=	0xdfc7
                           00DFC8   871 __X_SFRC8	=	0xdfc8
                           00DFC9   872 _X_WDCTL	=	0xdfc9
                           00DFCA   873 _X_T3CNT	=	0xdfca
                           00DFCB   874 _X_T3CTL	=	0xdfcb
                           00DFCC   875 _X_T3CCTL0	=	0xdfcc
                           00DFCD   876 _X_T3CC0	=	0xdfcd
                           00DFCE   877 _X_T3CCTL1	=	0xdfce
                           00DFCF   878 _X_T3CC1	=	0xdfcf
                           00DFD0   879 __NA_PSW	=	0xdfd0
                           00DFD1   880 _X_DMAIRQ	=	0xdfd1
                           00DFD2   881 _X_DMA1CFGL	=	0xdfd2
                           00DFD3   882 _X_DMA1CFGH	=	0xdfd3
                           00DFD4   883 _X_DMA0CFGL	=	0xdfd4
                           00DFD5   884 _X_DMA0CFGH	=	0xdfd5
                           00DFD6   885 _X_DMAARM	=	0xdfd6
                           00DFD7   886 _X_DMAREQ	=	0xdfd7
                           00DFD8   887 _X_TIMIF	=	0xdfd8
                           00DFD9   888 _X_RFD	=	0xdfd9
                           00DFDA   889 _X_T1CC0L	=	0xdfda
                           00DFDB   890 _X_T1CC0H	=	0xdfdb
                           00DFDC   891 _X_T1CC1L	=	0xdfdc
                           00DFDD   892 _X_T1CC1H	=	0xdfdd
                           00DFDE   893 _X_T1CC2L	=	0xdfde
                           00DFDF   894 _X_T1CC2H	=	0xdfdf
                           00DFE0   895 __NA_ACC	=	0xdfe0
                           00DFE1   896 _X_RFST	=	0xdfe1
                           00DFE2   897 _X_T1CNTL	=	0xdfe2
                           00DFE3   898 _X_T1CNTH	=	0xdfe3
                           00DFE4   899 _X_T1CTL	=	0xdfe4
                           00DFE5   900 _X_T1CCTL0	=	0xdfe5
                           00DFE6   901 _X_T1CCTL1	=	0xdfe6
                           00DFE7   902 _X_T1CCTL2	=	0xdfe7
                           00DFE8   903 __NA_IRCON2	=	0xdfe8
                           00DFE9   904 _X_RFIF	=	0xdfe9
                           00DFEA   905 _X_T4CNT	=	0xdfea
                           00DFEB   906 _X_T4CTL	=	0xdfeb
                           00DFEC   907 _X_T4CCTL0	=	0xdfec
                           00DFED   908 _X_T4CC0	=	0xdfed
                           00DFEE   909 _X_T4CCTL1	=	0xdfee
                           00DFEF   910 _X_T4CC1	=	0xdfef
                           00DFF0   911 __NA_B	=	0xdff0
                           00DFF1   912 _X_PERCFG	=	0xdff1
                           00DFF2   913 _X_ADCCFG	=	0xdff2
                           00DFF3   914 _X_P0SEL	=	0xdff3
                           00DFF4   915 _X_P1SEL	=	0xdff4
                           00DFF5   916 _X_P2SEL	=	0xdff5
                           00DFF6   917 _X_P1INP	=	0xdff6
                           00DFF7   918 _X_P2INP	=	0xdff7
                           00DFF8   919 _X_U1CSR	=	0xdff8
                           00DFF9   920 _X_U1DBUF	=	0xdff9
                           00DFFA   921 _X_U1BAUD	=	0xdffa
                           00DFFB   922 _X_U1UCR	=	0xdffb
                           00DFFC   923 _X_U1GCR	=	0xdffc
                           00DFFD   924 _X_P0DIR	=	0xdffd
                           00DFFE   925 _X_P1DIR	=	0xdffe
                           00DFFF   926 _X_P2DIR	=	0xdfff
      00F008                        927 _radioPktBuffer::
      00F008                        928 	.ds 20
                                    929 ;--------------------------------------------------------
                                    930 ; absolute external ram data
                                    931 ;--------------------------------------------------------
                                    932 	.area XABS    (ABS,XDATA)
                                    933 ;--------------------------------------------------------
                                    934 ; external initialized ram data
                                    935 ;--------------------------------------------------------
                                    936 	.area XISEG   (XDATA)
                                    937 	.area HOME    (CODE)
                                    938 	.area GSINIT0 (CODE)
                                    939 	.area GSINIT1 (CODE)
                                    940 	.area GSINIT2 (CODE)
                                    941 	.area GSINIT3 (CODE)
                                    942 	.area GSINIT4 (CODE)
                                    943 	.area GSINIT5 (CODE)
                                    944 	.area GSINIT  (CODE)
                                    945 	.area GSFINAL (CODE)
                                    946 	.area CSEG    (CODE)
                                    947 ;--------------------------------------------------------
                                    948 ; interrupt vector 
                                    949 ;--------------------------------------------------------
                                    950 	.area HOME    (CODE)
      000000                        951 __interrupt_vect:
      000000 02 00 89         [24]  952 	ljmp	__sdcc_gsinit_startup
      000003 32               [24]  953 	reti
      000004                        954 	.ds	7
      00000B 32               [24]  955 	reti
      00000C                        956 	.ds	7
      000013 32               [24]  957 	reti
      000014                        958 	.ds	7
      00001B 32               [24]  959 	reti
      00001C                        960 	.ds	7
      000023 32               [24]  961 	reti
      000024                        962 	.ds	7
      00002B 32               [24]  963 	reti
      00002C                        964 	.ds	7
      000033 32               [24]  965 	reti
      000034                        966 	.ds	7
      00003B 32               [24]  967 	reti
      00003C                        968 	.ds	7
      000043 32               [24]  969 	reti
      000044                        970 	.ds	7
      00004B 32               [24]  971 	reti
      00004C                        972 	.ds	7
      000053 32               [24]  973 	reti
      000054                        974 	.ds	7
      00005B 32               [24]  975 	reti
      00005C                        976 	.ds	7
      000063 32               [24]  977 	reti
      000064                        978 	.ds	7
      00006B 32               [24]  979 	reti
      00006C                        980 	.ds	7
      000073 32               [24]  981 	reti
      000074                        982 	.ds	7
      00007B 32               [24]  983 	reti
      00007C                        984 	.ds	7
      000083 02 06 24         [24]  985 	ljmp	_rf_IRQ
                                    986 ;--------------------------------------------------------
                                    987 ; global & static initialisations
                                    988 ;--------------------------------------------------------
                                    989 	.area HOME    (CODE)
                                    990 	.area GSINIT  (CODE)
                                    991 	.area GSFINAL (CODE)
                                    992 	.area GSINIT  (CODE)
                                    993 	.globl __sdcc_gsinit_startup
                                    994 	.globl __sdcc_program_startup
                                    995 	.globl __start__stack
                                    996 	.globl __mcs51_genXINIT
                                    997 	.globl __mcs51_genXRAMCLEAR
                                    998 	.globl __mcs51_genRAMCLEAR
                                    999 ;	../../lib/radio.h:48: static volatile uint8_t pktSentFlag = 0;            // Flag set whenever a packet is sent
      0000E8 75 0B 00         [24] 1000 	mov	_pktSentFlag,#0x00
                                   1001 ;	../../lib/radio.h:49: static volatile uint8_t pktRcvdFlag = 0;            // Flag set whenever a packet is received
      0000EB 75 0C 00         [24] 1002 	mov	_pktRcvdFlag,#0x00
                                   1003 	.area GSFINAL (CODE)
      0000EE 02 00 86         [24] 1004 	ljmp	__sdcc_program_startup
                                   1005 ;--------------------------------------------------------
                                   1006 ; Home
                                   1007 ;--------------------------------------------------------
                                   1008 	.area HOME    (CODE)
                                   1009 	.area HOME    (CODE)
      000086                       1010 __sdcc_program_startup:
      000086 02 06 43         [24] 1011 	ljmp	_main
                                   1012 ;	return from main will return to caller
                                   1013 ;--------------------------------------------------------
                                   1014 ; code
                                   1015 ;--------------------------------------------------------
                                   1016 	.area CSEG    (CODE)
                                   1017 ;------------------------------------------------------------
                                   1018 ;Allocation info for local variables in function 'rf_IRQ'
                                   1019 ;------------------------------------------------------------
                                   1020 ;	helloworld.c:11: void rf_IRQ(void) __interrupt RF_VECTOR{
                                   1021 ;	-----------------------------------------
                                   1022 ;	 function rf_IRQ
                                   1023 ;	-----------------------------------------
      000624                       1024 _rf_IRQ:
                           000007  1025 	ar7 = 0x07
                           000006  1026 	ar6 = 0x06
                           000005  1027 	ar5 = 0x05
                           000004  1028 	ar4 = 0x04
                           000003  1029 	ar3 = 0x03
                           000002  1030 	ar2 = 0x02
                           000001  1031 	ar1 = 0x01
                           000000  1032 	ar0 = 0x00
      000624 C0 E0            [24] 1033 	push	acc
      000626 C0 D0            [24] 1034 	push	psw
                                   1035 ;	helloworld.c:12: RFIF &= ~IRQ_DONE;        // Tx/Rx completed, clear interrupt flag
      000628 53 E9 EF         [24] 1036 	anl	_RFIF,#0xef
                                   1037 ;	helloworld.c:13: S1CON &= ~0x03;           // Clear the general RFIF interrupt registers
      00062B 53 9B FC         [24] 1038 	anl	_S1CON,#0xfc
                                   1039 ;	helloworld.c:15: if (mode == RADIO_MODE_RX) {
      00062E 74 20            [12] 1040 	mov	a,#0x20
      000630 B5 0D 05         [24] 1041 	cjne	a,_mode,00102$
                                   1042 ;	helloworld.c:16: pktRcvdFlag = 1;
      000633 75 0C 01         [24] 1043 	mov	_pktRcvdFlag,#0x01
      000636 80 06            [24] 1044 	sjmp	00104$
      000638                       1045 00102$:
                                   1046 ;	helloworld.c:19: pktSentFlag = 1;
      000638 75 0B 01         [24] 1047 	mov	_pktSentFlag,#0x01
                                   1048 ;	helloworld.c:20: RFST = RFST_SIDLE;      
      00063B 75 E1 04         [24] 1049 	mov	_RFST,#0x04
      00063E                       1050 00104$:
                                   1051 ;	helloworld.c:22: }
      00063E D0 D0            [24] 1052 	pop	psw
      000640 D0 E0            [24] 1053 	pop	acc
      000642 32               [24] 1054 	reti
                                   1055 ;	eliminated unneeded mov psw,# (no regs used in bank)
                                   1056 ;	eliminated unneeded push/pop dpl
                                   1057 ;	eliminated unneeded push/pop dph
                                   1058 ;	eliminated unneeded push/pop b
                                   1059 ;------------------------------------------------------------
                                   1060 ;Allocation info for local variables in function 'main'
                                   1061 ;------------------------------------------------------------
                                   1062 ;preamble                  Allocated with name '_main_preamble_65536_37'
                                   1063 ;cnt                       Allocated to registers r7 
                                   1064 ;------------------------------------------------------------
                                   1065 ;	helloworld.c:24: void main(void){
                                   1066 ;	-----------------------------------------
                                   1067 ;	 function main
                                   1068 ;	-----------------------------------------
      000643                       1069 _main:
                                   1070 ;	helloworld.c:25: uint8_t preamble[] = {0x0E, 0xA5, 0x5A};
      000643 75 0E 0E         [24] 1071 	mov	_main_preamble_65536_37,#0x0e
      000646 75 0F A5         [24] 1072 	mov	(_main_preamble_65536_37 + 0x0001),#0xa5
      000649 75 10 5A         [24] 1073 	mov	(_main_preamble_65536_37 + 0x0002),#0x5a
                                   1074 ;	helloworld.c:26: uint8_t cnt=48;
      00064C 7F 30            [12] 1075 	mov	r7,#0x30
                                   1076 ;	helloworld.c:28: mode = RADIO_MODE_TX;
      00064E 75 0D 10         [24] 1077 	mov	_mode,#0x10
                                   1078 ;	helloworld.c:32: P1SEL &= ~(BIT3);
      000651 53 F4 F7         [24] 1079 	anl	_P1SEL,#0xf7
                                   1080 ;	helloworld.c:33: P1DIR |= (BIT3);
      000654 43 FE 08         [24] 1081 	orl	_P1DIR,#0x08
                                   1082 ;	helloworld.c:34: P1_3 = 0;
                                   1083 ;	assignBit
      000657 C2 93            [12] 1084 	clr	_P1_3
                                   1085 ;	helloworld.c:37: SetMainClkSrc(CRYSTAL);
      000659 75 82 00         [24] 1086 	mov	dpl,#0x00
      00065C C0 07            [24] 1087 	push	ar7
      00065E 12 01 C5         [24] 1088 	lcall	_SetMainClkSrc
                                   1089 ;	helloworld.c:40: radioConfigure(DATA_RATE_1_CC1110, FREQUENCY_1_CC1110);
      000661 75 11 38         [24] 1090 	mov	_radioConfigure_PARM_2,#0x38
      000664 75 12 F6         [24] 1091 	mov	(_radioConfigure_PARM_2 + 1),#0xf6
      000667 75 13 0D         [24] 1092 	mov	(_radioConfigure_PARM_2 + 2),#0x0d
      00066A 75 14 00         [24] 1093 	mov	(_radioConfigure_PARM_2 + 3),#0x00
      00066D 90 D0 90         [24] 1094 	mov	dptr,#0xd090
      000670 75 F0 03         [24] 1095 	mov	b,#0x03
      000673 E4               [12] 1096 	clr	a
      000674 12 01 E6         [24] 1097 	lcall	_radioConfigure
                                   1098 ;	helloworld.c:43: dmaRadioSetup(RADIO_MODE_TX);
      000677 75 82 10         [24] 1099 	mov	dpl,#0x10
      00067A 12 00 F1         [24] 1100 	lcall	_dmaRadioSetup
                                   1101 ;	helloworld.c:46: HAL_INT_ENABLE(INUM_RF, INT_ON);    // Enable RF general interrupt
      00067D 43 9A 01         [24] 1102 	orl	_IEN2,#0x01
                                   1103 ;	helloworld.c:47: RFIM = IRQ_DONE;                    // Mask IRQ_DONE flag only
      000680 75 91 10         [24] 1104 	mov	_RFIM,#0x10
                                   1105 ;	helloworld.c:48: INT_GLOBAL_ENABLE(INT_ON);          // Enable interrupts globally
                                   1106 ;	assignBit
      000683 D2 AF            [12] 1107 	setb	_EA
                                   1108 ;	helloworld.c:52: radioPktBuffer[0] = PACKET_LENGTH;  // Length byte
      000685 90 F0 08         [24] 1109 	mov	dptr,#_radioPktBuffer
      000688 74 11            [12] 1110 	mov	a,#0x11
      00068A F0               [24] 1111 	movx	@dptr,a
                                   1112 ;	helloworld.c:53: radioPktBuffer[1] = (uint8_t) (NETWORK_ID_KEY>>8); // Network identifier
      00068B 90 F0 09         [24] 1113 	mov	dptr,#(_radioPktBuffer + 0x0001)
      00068E 74 5A            [12] 1114 	mov	a,#0x5a
      000690 F0               [24] 1115 	movx	@dptr,a
                                   1116 ;	helloworld.c:54: radioPktBuffer[2] = (uint8_t) NETWORK_ID_KEY;
      000691 90 F0 0A         [24] 1117 	mov	dptr,#(_radioPktBuffer + 0x0002)
      000694 F4               [12] 1118 	cpl	a
      000695 F0               [24] 1119 	movx	@dptr,a
                                   1120 ;	helloworld.c:55: memcpy(radioPktBuffer+3, "Hello", 5);
      000696 75 11 EF         [24] 1121 	mov	___memcpy_PARM_2,#___str_0
      000699 75 12 07         [24] 1122 	mov	(___memcpy_PARM_2 + 1),#(___str_0 >> 8)
      00069C 75 13 80         [24] 1123 	mov	(___memcpy_PARM_2 + 2),#0x80
      00069F 75 14 05         [24] 1124 	mov	___memcpy_PARM_3,#0x05
      0006A2 75 15 00         [24] 1125 	mov	(___memcpy_PARM_3 + 1),#0x00
      0006A5 90 F0 0B         [24] 1126 	mov	dptr,#(_radioPktBuffer + 0x0003)
      0006A8 75 F0 00         [24] 1127 	mov	b,#0x00
      0006AB 12 06 E1         [24] 1128 	lcall	___memcpy
      0006AE D0 07            [24] 1129 	pop	ar7
                                   1130 ;	helloworld.c:56: radioPktBuffer[8]=cnt;
      0006B0 90 F0 10         [24] 1131 	mov	dptr,#(_radioPktBuffer + 0x0008)
      0006B3 74 30            [12] 1132 	mov	a,#0x30
      0006B5 F0               [24] 1133 	movx	@dptr,a
                                   1134 ;	helloworld.c:58: while (1){
      0006B6                       1135 00163$:
                                   1136 ;	helloworld.c:59: DMAARM |= DMAARM_CHANNEL0;  // Arm DMA channel 0
      0006B6 43 D6 01         [24] 1137 	orl	_DMAARM,#0x01
                                   1138 ;	helloworld.c:60: RFST = STROBE_TX;           // Switch radio to TX
      0006B9 75 E1 03         [24] 1139 	mov	_RFST,#0x03
                                   1140 ;	helloworld.c:61: RFIF=1;
      0006BC 75 E9 01         [24] 1141 	mov	_RFIF,#0x01
                                   1142 ;	helloworld.c:65: while(!pktSentFlag);
      0006BF                       1143 00157$:
      0006BF E5 0B            [12] 1144 	mov	a,_pktSentFlag
      0006C1 60 FC            [24] 1145 	jz	00157$
                                   1146 ;	helloworld.c:66: pktSentFlag = 0;
      0006C3 75 0B 00         [24] 1147 	mov	_pktSentFlag,#0x00
                                   1148 ;	helloworld.c:68: P1_3 ^= 1; //toggle LED
      0006C6 B2 93            [12] 1149 	cpl	_P1_3
                                   1150 ;	helloworld.c:69: delayms(10); // Delay 
      0006C8 90 00 0A         [24] 1151 	mov	dptr,#0x000a
      0006CB C0 07            [24] 1152 	push	ar7
      0006CD 12 05 40         [24] 1153 	lcall	_delayms
      0006D0 D0 07            [24] 1154 	pop	ar7
                                   1155 ;	helloworld.c:70: radioPktBuffer[8]=cnt++;
      0006D2 8F 06            [24] 1156 	mov	ar6,r7
      0006D4 0F               [12] 1157 	inc	r7
      0006D5 90 F0 10         [24] 1158 	mov	dptr,#(_radioPktBuffer + 0x0008)
      0006D8 EE               [12] 1159 	mov	a,r6
      0006D9 F0               [24] 1160 	movx	@dptr,a
                                   1161 ;	helloworld.c:71: if (cnt==58) cnt=48;
      0006DA BF 3A D9         [24] 1162 	cjne	r7,#0x3a,00163$
      0006DD 7F 30            [12] 1163 	mov	r7,#0x30
                                   1164 ;	helloworld.c:74: }
      0006DF 80 D5            [24] 1165 	sjmp	00163$
                                   1166 	.area CSEG    (CODE)
                                   1167 	.area CONST   (CODE)
                                   1168 	.area CONST   (CODE)
      0007EF                       1169 ___str_0:
      0007EF 48 65 6C 6C 6F        1170 	.ascii "Hello"
      0007F4 00                    1171 	.db 0x00
                                   1172 	.area CSEG    (CODE)
                                   1173 	.area XINIT   (CODE)
                                   1174 	.area CABS    (ABS,CODE)
