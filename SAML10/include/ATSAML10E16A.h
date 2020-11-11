
/****************************************************************************************************//**
 * @file     ATSAML10E16A.h
 *
 * @brief    CMSIS Cortex-M3 Peripheral Access Layer Header File for
 *           ATSAML10E16A from Microchip Technology.
 *
 * @version  V0
 * @date     6. November 2020
 *
 * @note     Generated with SVDConv V2.87l 
 *           from CMSIS SVD File 'ATSAML10E16A.svd' Version 0,
 *******************************************************************************************************/



/** @addtogroup Microchip Technology
  * @{
  */

/** @addtogroup ATSAML10E16A
  * @{
  */

#ifndef ATSAML10E16A_H
#define ATSAML10E16A_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M3 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn         = -12,              /*!<   4  Memory Management, MPU mismatch, including Access Violation
                                                         and No Match                                                          */
  BusFault_IRQn                 = -11,              /*!<   5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                         related Fault                                                         */
  UsageFault_IRQn               = -10,              /*!<   6  Usage Fault, i.e. Undef Instruction, Illegal State Transition    */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  DebugMonitor_IRQn             =  -4,              /*!<  12  Debug Monitor                                                    */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* -------------------  ATSAML10E16A Specific Interrupt Numbers  ------------------ */
  WDT_IRQn                      =   1,              /*!<   1  WDT                                                              */
  RTC_IRQn                      =   2,              /*!<   2  RTC                                                              */
  EIC_0_IRQn                    =   3,              /*!<   3  EIC_0                                                            */
  EIC_1_IRQn                    =   4,              /*!<   4  EIC_1                                                            */
  EIC_2_IRQn                    =   5,              /*!<   5  EIC_2                                                            */
  EIC_3_IRQn                    =   6,              /*!<   6  EIC_3                                                            */
  EIC_OTHER_IRQn                =   7,              /*!<   7  EIC_OTHER                                                        */
  FREQM_IRQn                    =   8,              /*!<   8  FREQM                                                            */
  NVMCTRL_IRQn                  =   9,              /*!<   9  NVMCTRL                                                          */
  PORT_IRQn                     =  10,              /*!<  10  PORT                                                             */
  DMAC_0_IRQn                   =  11,              /*!<  11  DMAC_0                                                           */
  DMAC_1_IRQn                   =  12,              /*!<  12  DMAC_1                                                           */
  DMAC_2_IRQn                   =  13,              /*!<  13  DMAC_2                                                           */
  DMAC_3_IRQn                   =  14,              /*!<  14  DMAC_3                                                           */
  DMAC_OTHER_IRQn               =  15,              /*!<  15  DMAC_OTHER                                                       */
  EVSYS_0_IRQn                  =  16,              /*!<  16  EVSYS_0                                                          */
  EVSYS_1_IRQn                  =  17,              /*!<  17  EVSYS_1                                                          */
  EVSYS_2_IRQn                  =  18,              /*!<  18  EVSYS_2                                                          */
  EVSYS_3_IRQn                  =  19,              /*!<  19  EVSYS_3                                                          */
  EVSYS_NSCHK_IRQn              =  20,              /*!<  20  EVSYS_NSCHK                                                      */
  PAC_IRQn                      =  21,              /*!<  21  PAC                                                              */
  SERCOM0_0_IRQn                =  22,              /*!<  22  SERCOM0_0                                                        */
  SERCOM0_1_IRQn                =  23,              /*!<  23  SERCOM0_1                                                        */
  SERCOM0_2_IRQn                =  24,              /*!<  24  SERCOM0_2                                                        */
  SERCOM0_OTHER_IRQn            =  25,              /*!<  25  SERCOM0_OTHER                                                    */
  SERCOM1_0_IRQn                =  26,              /*!<  26  SERCOM1_0                                                        */
  SERCOM1_1_IRQn                =  27,              /*!<  27  SERCOM1_1                                                        */
  SERCOM1_2_IRQn                =  28,              /*!<  28  SERCOM1_2                                                        */
  SERCOM1_OTHER_IRQn            =  29,              /*!<  29  SERCOM1_OTHER                                                    */
  SERCOM2_0_IRQn                =  30,              /*!<  30  SERCOM2_0                                                        */
  SERCOM2_1_IRQn                =  31,              /*!<  31  SERCOM2_1                                                        */
  SERCOM2_2_IRQn                =  32,              /*!<  32  SERCOM2_2                                                        */
  SERCOM2_OTHER_IRQn            =  33,              /*!<  33  SERCOM2_OTHER                                                    */
  TC0_IRQn                      =  34,              /*!<  34  TC0                                                              */
  TC1_IRQn                      =  35,              /*!<  35  TC1                                                              */
  TC2_IRQn                      =  36,              /*!<  36  TC2                                                              */
  ADC_OTHER_IRQn                =  37,              /*!<  37  ADC_OTHER                                                        */
  ADC_RESRDY_IRQn               =  38,              /*!<  38  ADC_RESRDY                                                       */
  AC_IRQn                       =  39,              /*!<  39  AC                                                               */
  DAC_UNDERRUN_A_IRQn           =  40,              /*!<  40  DAC_UNDERRUN_A                                                   */
  DAC_EMPTY_IRQn                =  41,              /*!<  41  DAC_EMPTY                                                        */
  TRNG_IRQn                     =  43,              /*!<  43  TRNG                                                             */
  TRAM_IRQn                     =  44               /*!<  44  TRAM                                                             */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M3 Processor and Core Peripherals---------------- */
#define __CM3_REV                 0x0000            /*!< Cortex-M3 Core Revision                                               */
#define __MPU_PRESENT                  1            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               2            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm3.h"                               /*!< Cortex-M3 processor and core peripherals                              */
#include "system_SAML10.h"                          /*!< ATSAML10E16A System                                                   */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/** @addtogroup Device_Peripheral_Registers
  * @{
  */


/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif


typedef struct {
  __IO uint32_t  CHANNEL;                           /*!< Channel n Control                                                     */
  __IO uint8_t   CHINTENCLR;                        /*!< Channel n Interrupt Enable Clear                                      */
  __IO uint8_t   CHINTENSET;                        /*!< Channel n Interrupt Enable Set                                        */
  __IO uint8_t   CHINTFLAG;                         /*!< Channel n Interrupt Flag Status and Clear                             */
  __I  uint8_t   CHSTATUS;                          /*!< Channel n Status                                                      */
} EVSYS_CHANNEL_Type;

typedef struct {
  __IO uint32_t  DIR;                               /*!< Data Direction                                                        */
  __IO uint32_t  DIRCLR;                            /*!< Data Direction Clear                                                  */
  __IO uint32_t  DIRSET;                            /*!< Data Direction Set                                                    */
  __IO uint32_t  DIRTGL;                            /*!< Data Direction Toggle                                                 */
  __IO uint32_t  OUT;                               /*!< Data Output Value                                                     */
  __IO uint32_t  OUTCLR;                            /*!< Data Output Value Clear                                               */
  __IO uint32_t  OUTSET;                            /*!< Data Output Value Set                                                 */
  __IO uint32_t  OUTTGL;                            /*!< Data Output Value Toggle                                              */
  __I  uint32_t  IN;                                /*!< Data Input Value                                                      */
  __IO uint32_t  CTRL;                              /*!< Control                                                               */
  __O  uint32_t  WRCONFIG;                          /*!< Write Configuration                                                   */
  __IO uint32_t  EVCTRL;                            /*!< Event Input Control                                                   */
  __IO uint8_t   PMUX[16];                          /*!< Peripheral Multiplexing                                               */
  __IO uint8_t   PINCFG[32];                        /*!< Pin Configuration                                                     */
  __IO uint32_t  INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint32_t  INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint32_t  INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __IO uint32_t  NONSEC;                            /*!< Security Attribution                                                  */
  __IO uint32_t  NSCHK;                             /*!< Security Attribution Check                                            */
  __I  uint32_t  RESERVED[3];
} PORT_GROUP_Type;

typedef struct {
  __IO uint16_t  CTRLA;                             /*!< MODE0 Control A                                                       */
  __IO uint16_t  CTRLB;                             /*!< MODE0 Control B                                                       */
  __IO uint32_t  EVCTRL;                            /*!< MODE0 Event Control                                                   */
  __IO uint16_t  INTENCLR;                          /*!< MODE0 Interrupt Enable Clear                                          */
  __IO uint16_t  INTENSET;                          /*!< MODE0 Interrupt Enable Set                                            */
  __IO uint16_t  INTFLAG;                           /*!< MODE0 Interrupt Flag Status and Clear                                 */
  __IO uint8_t   DBGCTRL;                           /*!< Debug Control                                                         */
  __I  uint8_t   RESERVED1;
  __I  uint32_t  SYNCBUSY;                          /*!< MODE0 Synchronization Busy Status                                     */
  __IO uint8_t   FREQCORR;                          /*!< Frequency Correction                                                  */
  __I  uint8_t   RESERVED2[3];
  __IO uint32_t  COUNT;                             /*!< MODE0 Counter Value                                                   */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  COMP[1];                           /*!< MODE0 Compare n Value                                                 */
  __I  uint32_t  RESERVED4[7];
  __IO uint32_t  GP[2];                             /*!< General Purpose                                                       */
  __I  uint32_t  RESERVED5[6];
  __IO uint32_t  TAMPCTRL;                          /*!< Tamper Control                                                        */
  __I  uint32_t  TIMESTAMP;                         /*!< MODE0 Timestamp                                                       */
  __IO uint32_t  TAMPID;                            /*!< Tamper ID                                                             */
  __IO uint32_t  TAMPCTRLB;                         /*!< Tamper Control B                                                      */
} RtcMode0_Type;

typedef struct {
  __IO uint16_t  CTRLA;                             /*!< MODE1 Control A                                                       */
  __IO uint16_t  CTRLB;                             /*!< MODE1 Control B                                                       */
  __IO uint32_t  EVCTRL;                            /*!< MODE1 Event Control                                                   */
  __IO uint16_t  INTENCLR;                          /*!< MODE1 Interrupt Enable Clear                                          */
  __IO uint16_t  INTENSET;                          /*!< MODE1 Interrupt Enable Set                                            */
  __IO uint16_t  INTFLAG;                           /*!< MODE1 Interrupt Flag Status and Clear                                 */
  __IO uint8_t   DBGCTRL;                           /*!< Debug Control                                                         */
  __I  uint8_t   RESERVED6;
  __I  uint32_t  SYNCBUSY;                          /*!< MODE1 Synchronization Busy Status                                     */
  __IO uint8_t   FREQCORR;                          /*!< Frequency Correction                                                  */
  __I  uint8_t   RESERVED7[3];
  __IO uint16_t  COUNT;                             /*!< MODE1 Counter Value                                                   */
  __I  uint16_t  RESERVED8;
  __IO uint16_t  PER;                               /*!< MODE1 Counter Period                                                  */
  __I  uint16_t  RESERVED9;
  __IO uint16_t  COMP[2];                           /*!< MODE1 Compare n Value                                                 */
  __I  uint32_t  RESERVED10[7];
  __IO uint32_t  GP[2];                             /*!< General Purpose                                                       */
  __I  uint32_t  RESERVED11[6];
  __IO uint32_t  TAMPCTRL;                          /*!< Tamper Control                                                        */
  __I  uint32_t  TIMESTAMP;                         /*!< MODE1 Timestamp                                                       */
  __IO uint32_t  TAMPID;                            /*!< Tamper ID                                                             */
  __IO uint32_t  TAMPCTRLB;                         /*!< Tamper Control B                                                      */
} RtcMode1_Type;

typedef struct {
  __IO uint16_t  CTRLA;                             /*!< MODE2 Control A                                                       */
  __IO uint16_t  CTRLB;                             /*!< MODE2 Control B                                                       */
  __IO uint32_t  EVCTRL;                            /*!< MODE2 Event Control                                                   */
  __IO uint16_t  INTENCLR;                          /*!< MODE2 Interrupt Enable Clear                                          */
  __IO uint16_t  INTENSET;                          /*!< MODE2 Interrupt Enable Set                                            */
  __IO uint16_t  INTFLAG;                           /*!< MODE2 Interrupt Flag Status and Clear                                 */
  __IO uint8_t   DBGCTRL;                           /*!< Debug Control                                                         */
  __I  uint8_t   RESERVED12;
  __I  uint32_t  SYNCBUSY;                          /*!< MODE2 Synchronization Busy Status                                     */
  __IO uint8_t   FREQCORR;                          /*!< Frequency Correction                                                  */
  __I  uint8_t   RESERVED13[3];
  __IO uint32_t  CLOCK;                             /*!< MODE2 Clock Value                                                     */
  __I  uint32_t  RESERVED14[9];
  __IO uint32_t  GP[2];                             /*!< General Purpose                                                       */
  __I  uint32_t  RESERVED15[6];
  __IO uint32_t  TAMPCTRL;                          /*!< Tamper Control                                                        */
  __I  uint32_t  TIMESTAMP;                         /*!< MODE2 Timestamp                                                       */
  __IO uint32_t  TAMPID;                            /*!< Tamper ID                                                             */
  __IO uint32_t  TAMPCTRLB;                         /*!< Tamper Control B                                                      */
} RtcMode2_Type;

typedef struct {
  __IO uint32_t  ALARM;                             /*!< MODE2_ALARM Alarm n Value                                             */
  __IO uint8_t   MASK;                              /*!< MODE2_ALARM Alarm n Mask                                              */
  __I  uint8_t   RESERVED16[3];
} RTC_MODE2_ALARM_Type;

typedef struct {
  __IO uint32_t  CTRLA;                             /*!< I2CM Control A                                                        */
  __IO uint32_t  CTRLB;                             /*!< I2CM Control B                                                        */
  __I  uint32_t  RESERVED17;
  __IO uint32_t  BAUD;                              /*!< I2CM Baud Rate                                                        */
  __I  uint32_t  RESERVED18;
  __IO uint8_t   INTENCLR;                          /*!< I2CM Interrupt Enable Clear                                           */
  __I  uint8_t   RESERVED19;
  __IO uint8_t   INTENSET;                          /*!< I2CM Interrupt Enable Set                                             */
  __I  uint8_t   RESERVED20;
  __IO uint8_t   INTFLAG;                           /*!< I2CM Interrupt Flag Status and Clear                                  */
  __I  uint8_t   RESERVED21;
  __IO uint16_t  STATUS;                            /*!< I2CM Status                                                           */
  __I  uint32_t  SYNCBUSY;                          /*!< I2CM Synchronization Busy                                             */
  __I  uint32_t  RESERVED22;
  __IO uint32_t  ADDR;                              /*!< I2CM Address                                                          */
  __IO uint8_t   DATA;                              /*!< I2CM Data                                                             */
  __I  uint8_t   RESERVED23[7];
  __IO uint8_t   DBGCTRL;                           /*!< I2CM Debug Control                                                    */
} SercomI2cm_Type;

typedef struct {
  __IO uint32_t  CTRLA;                             /*!< I2CS Control A                                                        */
  __IO uint32_t  CTRLB;                             /*!< I2CS Control B                                                        */
  __I  uint32_t  RESERVED24[3];
  __IO uint8_t   INTENCLR;                          /*!< I2CS Interrupt Enable Clear                                           */
  __I  uint8_t   RESERVED25;
  __IO uint8_t   INTENSET;                          /*!< I2CS Interrupt Enable Set                                             */
  __I  uint8_t   RESERVED26;
  __IO uint8_t   INTFLAG;                           /*!< I2CS Interrupt Flag Status and Clear                                  */
  __I  uint8_t   RESERVED27;
  __IO uint16_t  STATUS;                            /*!< I2CS Status                                                           */
  __I  uint32_t  SYNCBUSY;                          /*!< I2CS Synchronization Busy                                             */
  __I  uint32_t  RESERVED28;
  __IO uint32_t  ADDR;                              /*!< I2CS Address                                                          */
  __IO uint8_t   DATA;                              /*!< I2CS Data                                                             */
} SercomI2cs_Type;

typedef struct {
  __IO uint32_t  CTRLA;                             /*!< SPI Control A                                                         */
  __IO uint32_t  CTRLB;                             /*!< SPI Control B                                                         */
  __I  uint32_t  RESERVED29;
  __IO uint8_t   BAUD;                              /*!< SPI Baud Rate                                                         */
  __I  uint8_t   RESERVED30[7];
  __IO uint8_t   INTENCLR;                          /*!< SPI Interrupt Enable Clear                                            */
  __I  uint8_t   RESERVED31;
  __IO uint8_t   INTENSET;                          /*!< SPI Interrupt Enable Set                                              */
  __I  uint8_t   RESERVED32;
  __IO uint8_t   INTFLAG;                           /*!< SPI Interrupt Flag Status and Clear                                   */
  __I  uint8_t   RESERVED33;
  __IO uint16_t  STATUS;                            /*!< SPI Status                                                            */
  __I  uint32_t  SYNCBUSY;                          /*!< SPI Synchronization Busy                                              */
  __I  uint32_t  RESERVED34;
  __IO uint32_t  ADDR;                              /*!< SPI Address                                                           */
  __IO uint32_t  DATA;                              /*!< SPI Data                                                              */
  __I  uint32_t  RESERVED35;
  __IO uint8_t   DBGCTRL;                           /*!< SPI Debug Control                                                     */
} SercomSpi_Type;

typedef struct {
  __IO uint32_t  CTRLA;                             /*!< USART Control A                                                       */
  __IO uint32_t  CTRLB;                             /*!< USART Control B                                                       */
  __IO uint32_t  CTRLC;                             /*!< USART Control C                                                       */
  
  union {
    __IO uint16_t  BAUD_FRACFP_MODE;                /*!< USART Baud Rate                                                       */
    __IO uint16_t  BAUD_USARTFP_MODE;               /*!< USART Baud Rate                                                       */
    __IO uint16_t  BAUD;                            /*!< USART Baud Rate                                                       */
    __IO uint16_t  BAUD_FRAC_MODE;                  /*!< USART Baud Rate                                                       */
  };
  __IO uint8_t   RXPL;                              /*!< USART Receive Pulse Length                                            */
  __I  uint8_t   RESERVED36[5];
  __IO uint8_t   INTENCLR;                          /*!< USART Interrupt Enable Clear                                          */
  __I  uint8_t   RESERVED37;
  __IO uint8_t   INTENSET;                          /*!< USART Interrupt Enable Set                                            */
  __I  uint8_t   RESERVED38;
  __IO uint8_t   INTFLAG;                           /*!< USART Interrupt Flag Status and Clear                                 */
  __I  uint8_t   RESERVED39;
  __IO uint16_t  STATUS;                            /*!< USART Status                                                          */
  __I  uint32_t  SYNCBUSY;                          /*!< USART Synchronization Busy                                            */
  __I  uint8_t   RXERRCNT;                          /*!< USART Receive Error Count                                             */
  __I  uint8_t   RESERVED40[7];
  __IO uint16_t  DATA;                              /*!< USART Data                                                            */
  __I  uint16_t  RESERVED41[3];
  __IO uint8_t   DBGCTRL;                           /*!< USART Debug Control                                                   */
} SercomUsart_Type;

typedef struct {
  __IO uint32_t  CTRLA;                             /*!< Control A                                                             */
  __IO uint8_t   CTRLBCLR;                          /*!< Control B Clear                                                       */
  __IO uint8_t   CTRLBSET;                          /*!< Control B Set                                                         */
  __IO uint16_t  EVCTRL;                            /*!< Event Control                                                         */
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __IO uint8_t   STATUS;                            /*!< Status                                                                */
  __IO uint8_t   WAVE;                              /*!< Waveform Generation Control                                           */
  __IO uint8_t   DRVCTRL;                           /*!< Control C                                                             */
  __I  uint8_t   RESERVED42;
  __IO uint8_t   DBGCTRL;                           /*!< Debug Control                                                         */
  __I  uint32_t  SYNCBUSY;                          /*!< Synchronization Status                                                */
  __IO uint8_t   COUNT;                             /*!< COUNT8 Count                                                          */
  __I  uint16_t  RESERVED43[3];
  __IO uint8_t   PER;                               /*!< COUNT8 Period                                                         */
  __IO uint8_t   CC[2];                             /*!< COUNT8 Compare and Capture                                            */
  __I  uint8_t   RESERVED44[17];
  __IO uint8_t   PERBUF;                            /*!< COUNT8 Period Buffer                                                  */
  __IO uint8_t   CCBUF[2];                          /*!< COUNT8 Compare and Capture Buffer                                     */
} TcCount8_Type;

typedef struct {
  __IO uint32_t  CTRLA;                             /*!< Control A                                                             */
  __IO uint8_t   CTRLBCLR;                          /*!< Control B Clear                                                       */
  __IO uint8_t   CTRLBSET;                          /*!< Control B Set                                                         */
  __IO uint16_t  EVCTRL;                            /*!< Event Control                                                         */
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __IO uint8_t   STATUS;                            /*!< Status                                                                */
  __IO uint8_t   WAVE;                              /*!< Waveform Generation Control                                           */
  __IO uint8_t   DRVCTRL;                           /*!< Control C                                                             */
  __I  uint8_t   RESERVED45;
  __IO uint8_t   DBGCTRL;                           /*!< Debug Control                                                         */
  __I  uint32_t  SYNCBUSY;                          /*!< Synchronization Status                                                */
  __IO uint16_t  COUNT;                             /*!< COUNT16 Count                                                         */
  __I  uint32_t  RESERVED46;
  __IO uint16_t  PER;                               /*!< COUNT16 Period                                                        */
  __IO uint16_t  CC[2];                             /*!< COUNT16 Compare and Capture                                           */
  __I  uint16_t  RESERVED47[7];
  __IO uint16_t  PERBUF;                            /*!< COUNT16 Period Buffer                                                 */
  __IO uint16_t  CCBUF[2];                          /*!< COUNT16 Compare and Capture Buffer                                    */
} TcCount16_Type;

typedef struct {
  __IO uint32_t  CTRLA;                             /*!< Control A                                                             */
  __IO uint8_t   CTRLBCLR;                          /*!< Control B Clear                                                       */
  __IO uint8_t   CTRLBSET;                          /*!< Control B Set                                                         */
  __IO uint16_t  EVCTRL;                            /*!< Event Control                                                         */
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __IO uint8_t   STATUS;                            /*!< Status                                                                */
  __IO uint8_t   WAVE;                              /*!< Waveform Generation Control                                           */
  __IO uint8_t   DRVCTRL;                           /*!< Control C                                                             */
  __I  uint8_t   RESERVED48;
  __IO uint8_t   DBGCTRL;                           /*!< Debug Control                                                         */
  __I  uint32_t  SYNCBUSY;                          /*!< Synchronization Status                                                */
  __IO uint32_t  COUNT;                             /*!< COUNT32 Count                                                         */
  __IO uint32_t  PER;                               /*!< COUNT32 Period                                                        */
  __IO uint32_t  CC[2];                             /*!< COUNT32 Compare and Capture                                           */
  __I  uint32_t  RESERVED49[2];
  __IO uint32_t  PERBUF;                            /*!< COUNT32 Period Buffer                                                 */
  __IO uint32_t  CCBUF[2];                          /*!< COUNT32 Compare and Capture Buffer                                    */
} TcCount32_Type;

typedef struct {
  __IO uint32_t  DWT_COMP;                          /*!< DWT Comparator Register n                                             */
  __I  uint32_t  RESERVED50;
  __IO uint32_t  DWT_FUNCTION;                      /*!< DWT Function Register x                                               */
  __I  uint32_t  RESERVED51;
} DWT_COMPARATOR_Type;


/* ================================================================================ */
/* ================                       AC                       ================ */
/* ================================================================================ */


/**
  * @brief Analog Comparators (AC)
  */

typedef struct {                                    /*!< AC Structure                                                          */
  __IO uint8_t   CTRLA;                             /*!< Control A                                                             */
  __O  uint8_t   CTRLB;                             /*!< Control B                                                             */
  __IO uint16_t  EVCTRL;                            /*!< Event Control                                                         */
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint8_t   STATUSA;                           /*!< Status A                                                              */
  __I  uint8_t   STATUSB;                           /*!< Status B                                                              */
  __IO uint8_t   DBGCTRL;                           /*!< Debug Control                                                         */
  __IO uint8_t   WINCTRL;                           /*!< Window Control                                                        */
  __I  uint8_t   RESERVED;
  __IO uint8_t   SCALER[2];                         /*!< Scaler n                                                              */
  __I  uint16_t  RESERVED1;
  __IO uint32_t  COMPCTRL[2];                       /*!< Comparator Control n                                                  */
  __I  uint32_t  RESERVED2[2];
  __I  uint32_t  SYNCBUSY;                          /*!< Synchronization Busy                                                  */
} AC_Type;


/* ================================================================================ */
/* ================                       ADC                      ================ */
/* ================================================================================ */


/**
  * @brief Analog Digital Converter (ADC)
  */

typedef struct {                                    /*!< ADC Structure                                                         */
  __IO uint8_t   CTRLA;                             /*!< Control A                                                             */
  __IO uint8_t   CTRLB;                             /*!< Control B                                                             */
  __IO uint8_t   REFCTRL;                           /*!< Reference Control                                                     */
  __IO uint8_t   EVCTRL;                            /*!< Event Control                                                         */
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint8_t   SEQSTATUS;                         /*!< Sequence Status                                                       */
  __IO uint16_t  INPUTCTRL;                         /*!< Input Control                                                         */
  __IO uint16_t  CTRLC;                             /*!< Control C                                                             */
  __IO uint8_t   AVGCTRL;                           /*!< Average Control                                                       */
  __IO uint8_t   SAMPCTRL;                          /*!< Sample Time Control                                                   */
  __IO uint16_t  WINLT;                             /*!< Window Monitor Lower Threshold                                        */
  __IO uint16_t  WINUT;                             /*!< Window Monitor Upper Threshold                                        */
  __IO uint16_t  GAINCORR;                          /*!< Gain Correction                                                       */
  __IO uint16_t  OFFSETCORR;                        /*!< Offset Correction                                                     */
  __I  uint16_t  RESERVED;
  __IO uint8_t   SWTRIG;                            /*!< Software Trigger                                                      */
  __I  uint8_t   RESERVED1[3];
  __IO uint8_t   DBGCTRL;                           /*!< Debug Control                                                         */
  __I  uint8_t   RESERVED2[3];
  __I  uint16_t  SYNCBUSY;                          /*!< Synchronization Busy                                                  */
  __I  uint16_t  RESERVED3;
  __I  uint16_t  RESULT;                            /*!< Result                                                                */
  __I  uint16_t  RESERVED4;
  __IO uint32_t  SEQCTRL;                           /*!< Sequence Control                                                      */
  __IO uint16_t  CALIB;                             /*!< Calibration                                                           */
} ADC_Type;


/* ================================================================================ */
/* ================                       CCL                      ================ */
/* ================================================================================ */


/**
  * @brief Configurable Custom Logic (CCL)
  */

typedef struct {                                    /*!< CCL Structure                                                         */
  __IO uint8_t   CTRL;                              /*!< Control                                                               */
  __I  uint8_t   RESERVED[3];
  __IO uint8_t   SEQCTRL[1];                        /*!< SEQ Control x                                                         */
  __I  uint8_t   RESERVED1[3];
  __IO uint32_t  LUTCTRL[2];                        /*!< LUT Control x                                                         */
} CCL_Type;


/* ================================================================================ */
/* ================                       DAC                      ================ */
/* ================================================================================ */


/**
  * @brief Digital Analog Converter (DAC)
  */

typedef struct {                                    /*!< DAC Structure                                                         */
  __IO uint8_t   CTRLA;                             /*!< Control A                                                             */
  __IO uint8_t   CTRLB;                             /*!< Control B                                                             */
  __IO uint8_t   EVCTRL;                            /*!< Event Control                                                         */
  __I  uint8_t   RESERVED;
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint8_t   STATUS;                            /*!< Status                                                                */
  __O  uint16_t  DATA;                              /*!< Data                                                                  */
  __I  uint16_t  RESERVED1;
  __O  uint16_t  DATABUF;                           /*!< Data Buffer                                                           */
  __I  uint16_t  RESERVED2;
  __I  uint32_t  SYNCBUSY;                          /*!< Synchronization Busy                                                  */
  __IO uint8_t   DBGCTRL;                           /*!< Debug Control                                                         */
} DAC_Type;


/* ================================================================================ */
/* ================                      DMAC                      ================ */
/* ================================================================================ */


/**
  * @brief Direct Memory Access Controller (DMAC)
  */

typedef struct {                                    /*!< DMAC Structure                                                        */
  __IO uint16_t  CTRL;                              /*!< Control                                                               */
  __IO uint16_t  CRCCTRL;                           /*!< CRC Control                                                           */
  __IO uint32_t  CRCDATAIN;                         /*!< CRC Data Input                                                        */
  __IO uint32_t  CRCCHKSUM;                         /*!< CRC Checksum                                                          */
  __IO uint8_t   CRCSTATUS;                         /*!< CRC Status                                                            */
  __IO uint8_t   DBGCTRL;                           /*!< Debug Control                                                         */
  __IO uint8_t   QOSCTRL;                           /*!< QOS Control                                                           */
  __I  uint8_t   RESERVED;
  __IO uint32_t  SWTRIGCTRL;                        /*!< Software Trigger Control                                              */
  __IO uint32_t  PRICTRL0;                          /*!< Priority Control 0                                                    */
  __I  uint32_t  RESERVED1[2];
  __IO uint16_t  INTPEND;                           /*!< Interrupt Pending                                                     */
  __I  uint16_t  RESERVED2;
  __I  uint32_t  INTSTATUS;                         /*!< Interrupt Status                                                      */
  __I  uint32_t  BUSYCH;                            /*!< Busy Channels                                                         */
  __I  uint32_t  PENDCH;                            /*!< Pending Channels                                                      */
  __I  uint32_t  ACTIVE;                            /*!< Active Channel and Levels                                             */
  __IO uint32_t  BASEADDR;                          /*!< Descriptor Memory Section Base Address                                */
  __IO uint32_t  WRBADDR;                           /*!< Write-Back Memory Section Base Address                                */
  __I  uint8_t   RESERVED3[3];
  __IO uint8_t   CHID;                              /*!< Channel ID                                                            */
  __IO uint8_t   CHCTRLA;                           /*!< Channel Control A                                                     */
  __I  uint8_t   RESERVED4[3];
  __IO uint32_t  CHCTRLB;                           /*!< Channel Control B                                                     */
  __I  uint32_t  RESERVED5;
  __IO uint8_t   CHINTENCLR;                        /*!< Channel Interrupt Enable Clear                                        */
  __IO uint8_t   CHINTENSET;                        /*!< Channel Interrupt Enable Set                                          */
  __IO uint8_t   CHINTFLAG;                         /*!< Channel Interrupt Flag Status and Clear                               */
  __I  uint8_t   CHSTATUS;                          /*!< Channel Status                                                        */
} DMAC_Type;


/* ================================================================================ */
/* ================                       DSU                      ================ */
/* ================================================================================ */


/**
  * @brief Device Service Unit (DSU)
  */

typedef struct {                                    /*!< DSU Structure                                                         */
  __O  uint8_t   CTRL;                              /*!< Control                                                               */
  __IO uint8_t   STATUSA;                           /*!< Status A                                                              */
  __I  uint8_t   STATUSB;                           /*!< Status B                                                              */
  __I  uint8_t   STATUSC;                           /*!< Status C                                                              */
  __IO uint32_t  ADDR;                              /*!< Address                                                               */
  __IO uint32_t  LENGTH;                            /*!< Length                                                                */
  __IO uint32_t  DATA;                              /*!< Data                                                                  */
  __IO uint32_t  DCC[2];                            /*!< Debug Communication Channel n                                         */
  __I  uint32_t  DID;                               /*!< Device Identification                                                 */
  __IO uint32_t  CFG;                               /*!< Configuration                                                         */
  __IO uint32_t  BCC[2];                            /*!< Boot ROM Communication Channel n                                      */
  __I  uint32_t  RESERVED[50];
  __IO uint32_t  DCFG[2];                           /*!< Device Configuration                                                  */
  __I  uint32_t  RESERVED1[962];
  __I  uint32_t  ENTRY0;                            /*!< CoreSight ROM Table Entry 0                                           */
  __I  uint32_t  ENTRY1;                            /*!< CoreSight ROM Table Entry 1                                           */
  __I  uint32_t  END;                               /*!< CoreSight ROM Table End                                               */
  __I  uint32_t  RESERVED2[1008];
  __I  uint32_t  MEMTYPE;                           /*!< CoreSight ROM Table Memory Type                                       */
  __I  uint32_t  PID4;                              /*!< Peripheral Identification 4                                           */
  __I  uint32_t  PID5;                              /*!< Peripheral Identification 5                                           */
  __I  uint32_t  PID6;                              /*!< Peripheral Identification 6                                           */
  __I  uint32_t  PID7;                              /*!< Peripheral Identification 7                                           */
  __I  uint32_t  PID0;                              /*!< Peripheral Identification 0                                           */
  __I  uint32_t  PID1;                              /*!< Peripheral Identification 1                                           */
  __I  uint32_t  PID2;                              /*!< Peripheral Identification 2                                           */
  __I  uint32_t  PID3;                              /*!< Peripheral Identification 3                                           */
  __I  uint32_t  CID0;                              /*!< Component Identification 0                                            */
  __I  uint32_t  CID1;                              /*!< Component Identification 1                                            */
  __I  uint32_t  CID2;                              /*!< Component Identification 2                                            */
  __I  uint32_t  CID3;                              /*!< Component Identification 3                                            */
} DSU_Type;


/* ================================================================================ */
/* ================                       EIC                      ================ */
/* ================================================================================ */


/**
  * @brief External Interrupt Controller (EIC)
  */

typedef struct {                                    /*!< EIC Structure                                                         */
  __IO uint8_t   CTRLA;                             /*!< Control A                                                             */
  __IO uint8_t   NMICTRL;                           /*!< Non-Maskable Interrupt Control                                        */
  __IO uint8_t   NMIFLAG;                           /*!< Non-Maskable Interrupt Flag Status and Clear                          */
  __I  uint8_t   RESERVED;
  __I  uint32_t  SYNCBUSY;                          /*!< Synchronization Busy                                                  */
  __IO uint32_t  EVCTRL;                            /*!< Event Control                                                         */
  __IO uint32_t  INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint32_t  INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint32_t  INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __IO uint32_t  ASYNCH;                            /*!< External Interrupt Asynchronous Mode                                  */
  __IO uint32_t  CONFIG[1];                         /*!< External Interrupt Sense Configuration                                */
  __I  uint32_t  RESERVED1[4];
  __IO uint32_t  DEBOUNCEN;                         /*!< Debouncer Enable                                                      */
  __IO uint32_t  DPRESCALER;                        /*!< Debouncer Prescaler                                                   */
  __I  uint32_t  PINSTATE;                          /*!< Pin State                                                             */
  __IO uint32_t  NSCHK;                             /*!< Non-secure Interrupt Check Enable                                     */
  __IO uint32_t  NONSEC;                            /*!< Non-secure Interrupt                                                  */
} EIC_Type;


/* ================================================================================ */
/* ================                      EVSYS                     ================ */
/* ================================================================================ */


/**
  * @brief Event System Interface (EVSYS)
  */

typedef struct {                                    /*!< EVSYS Structure                                                       */
  __O  uint8_t   CTRLA;                             /*!< Control                                                               */
  __I  uint8_t   RESERVED[3];
  __O  uint32_t  SWEVT;                             /*!< Software Event                                                        */
  __IO uint8_t   PRICTRL;                           /*!< Priority Control                                                      */
  __I  uint8_t   RESERVED1[7];
  __IO uint16_t  INTPEND;                           /*!< Channel Pending Interrupt                                             */
  __I  uint16_t  RESERVED2;
  __I  uint32_t  INTSTATUS;                         /*!< Interrupt Status                                                      */
  __I  uint32_t  BUSYCH;                            /*!< Busy Channels                                                         */
  __I  uint32_t  READYUSR;                          /*!< Ready Users                                                           */
  EVSYS_CHANNEL_Type CHANNEL[8];                    /*!< CHANNEL0                                                              */
  __I  uint32_t  RESERVED3[48];
  __IO uint8_t   USER[23];                          /*!< User Multiplexer n                                                    */
  __I  uint8_t   RESERVED4[157];
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint8_t   RESERVED5;
  __IO uint32_t  NONSECCHAN;                        /*!< Channels Security Attribution                                         */
  __IO uint32_t  NSCHKCHAN;                         /*!< Non-Secure Channels Check                                             */
  __IO uint32_t  NONSECUSER[1];                     /*!< Users Security Attribution                                            */
  __I  uint32_t  RESERVED6[3];
  __IO uint32_t  NSCHKUSER[1];                      /*!< Non-Secure Users Check                                                */
} EVSYS_Type;


/* ================================================================================ */
/* ================                      FREQM                     ================ */
/* ================================================================================ */


/**
  * @brief Frequency Meter (FREQM)
  */

typedef struct {                                    /*!< FREQM Structure                                                       */
  __IO uint8_t   CTRLA;                             /*!< Control A Register                                                    */
  __O  uint8_t   CTRLB;                             /*!< Control B Register                                                    */
  __IO uint16_t  CFGA;                              /*!< Config A register                                                     */
  __I  uint32_t  RESERVED;
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear Register                                       */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set Register                                         */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Register                                               */
  __IO uint8_t   STATUS;                            /*!< Status Register                                                       */
  __I  uint32_t  SYNCBUSY;                          /*!< Synchronization Busy Register                                         */
  __I  uint32_t  VALUE;                             /*!< Count Value Register                                                  */
} FREQM_Type;


/* ================================================================================ */
/* ================                      GCLK                      ================ */
/* ================================================================================ */


/**
  * @brief Generic Clock Generator (GCLK)
  */

typedef struct {                                    /*!< GCLK Structure                                                        */
  __IO uint8_t   CTRLA;                             /*!< Control                                                               */
  __I  uint8_t   RESERVED[3];
  __I  uint32_t  SYNCBUSY;                          /*!< Synchronization Busy                                                  */
  __I  uint32_t  RESERVED1[6];
  __IO uint32_t  GENCTRL[5];                        /*!< Generic Clock Generator Control                                       */
  __I  uint32_t  RESERVED2[19];
  __IO uint32_t  PCHCTRL[21];                       /*!< Peripheral Clock Control                                              */
} GCLK_Type;


/* ================================================================================ */
/* ================                      MCLK                      ================ */
/* ================================================================================ */


/**
  * @brief Main Clock (MCLK)
  */

typedef struct {                                    /*!< MCLK Structure                                                        */
  __IO uint8_t   CTRLA;                             /*!< Control                                                               */
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __IO uint8_t   CPUDIV;                            /*!< CPU Clock Division                                                    */
  __I  uint8_t   RESERVED[11];
  __IO uint32_t  AHBMASK;                           /*!< AHB Mask                                                              */
  __IO uint32_t  APBAMASK;                          /*!< APBA Mask                                                             */
  __IO uint32_t  APBBMASK;                          /*!< APBB Mask                                                             */
  __IO uint32_t  APBCMASK;                          /*!< APBC Mask                                                             */
} MCLK_Type;


/* ================================================================================ */
/* ================                     NVMCTRL                    ================ */
/* ================================================================================ */


/**
  * @brief Non-Volatile Memory Controller (NVMCTRL)
  */

typedef struct {                                    /*!< NVMCTRL Structure                                                     */
  __O  uint16_t  CTRLA;                             /*!< Control A                                                             */
  __I  uint16_t  RESERVED;
  __IO uint32_t  CTRLB;                             /*!< Control B                                                             */
  __IO uint8_t   CTRLC;                             /*!< Control C                                                             */
  __I  uint8_t   RESERVED1;
  __IO uint8_t   EVCTRL;                            /*!< Event Control                                                         */
  __I  uint8_t   RESERVED2;
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __I  uint8_t   RESERVED3[3];
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __I  uint8_t   RESERVED4[3];
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint8_t   RESERVED5[3];
  __I  uint16_t  STATUS;                            /*!< Status                                                                */
  __I  uint16_t  RESERVED6;
  __IO uint32_t  ADDR;                              /*!< Address                                                               */
  __IO uint16_t  SULCK;                             /*!< Secure Unlock Register                                                */
  __IO uint16_t  NSULCK;                            /*!< Non-Secure Unlock Register                                            */
  __IO uint32_t  PARAM;                             /*!< NVM Parameter                                                         */
  __I  uint32_t  RESERVED7[2];
  __O  uint32_t  DSCC;                              /*!< Data Scramble Configuration                                           */
  __IO uint32_t  SECCTRL;                           /*!< Security Control                                                      */
  __IO uint32_t  SCFGB;                             /*!< Secure Boot Configuration                                             */
  __IO uint32_t  SCFGAD;                            /*!< Secure Application and Data Configuration                             */
  __IO uint32_t  NONSEC;                            /*!< Non-secure Write Enable                                               */
  __IO uint32_t  NSCHK;                             /*!< Non-secure Write Reference Value                                      */
} NVMCTRL_Type;


/* ================================================================================ */
/* ================                      OPAMP                     ================ */
/* ================================================================================ */


/**
  * @brief Operational Amplifier (OPAMP)
  */

typedef struct {                                    /*!< OPAMP Structure                                                       */
  __IO uint8_t   CTRLA;                             /*!< Control A                                                             */
  __I  uint8_t   RESERVED;
  __I  uint8_t   STATUS;                            /*!< Status                                                                */
  __I  uint8_t   RESERVED1;
  __IO uint32_t  OPAMPCTRL[3];                      /*!< OPAMP n Control                                                       */
  __IO uint8_t   RESCTRL;                           /*!< Resister Control                                                      */
} OPAMP_Type;


/* ================================================================================ */
/* ================                     OSCCTRL                    ================ */
/* ================================================================================ */


/**
  * @brief Oscillators Control (OSCCTRL)
  */

typedef struct {                                    /*!< OSCCTRL Structure                                                     */
  __IO uint8_t   EVCTRL;                            /*!< Event Control                                                         */
  __I  uint8_t   RESERVED[3];
  __IO uint32_t  INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint32_t  INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint32_t  INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint32_t  STATUS;                            /*!< Status                                                                */
  __IO uint16_t  XOSCCTRL;                          /*!< External Multipurpose Crystal Oscillator (XOSC) Control               */
  __IO uint8_t   CFDPRESC;                          /*!< Clock Failure Detector Prescaler                                      */
  __I  uint8_t   RESERVED1;
  __IO uint8_t   OSC16MCTRL;                        /*!< 16MHz Internal Oscillator (OSC16M) Control                            */
  __I  uint8_t   RESERVED2[3];
  __IO uint16_t  DFLLULPCTRL;                       /*!< DFLLULP Control                                                       */
  __IO uint8_t   DFLLULPDITHER;                     /*!< DFLLULP Dither Control                                                */
  __IO uint8_t   DFLLULPRREQ;                       /*!< DFLLULP Read Request                                                  */
  __IO uint32_t  DFLLULPDLY;                        /*!< DFLLULP Delay Value                                                   */
  __IO uint32_t  DFLLULPRATIO;                      /*!< DFLLULP Target Ratio                                                  */
  __I  uint32_t  DFLLULPSYNCBUSY;                   /*!< DFLLULP Synchronization Busy                                          */
  __IO uint8_t   DPLLCTRLA;                         /*!< DPLL Control A                                                        */
  __I  uint8_t   RESERVED3[3];
  __IO uint32_t  DPLLRATIO;                         /*!< DPLL Ratio Control                                                    */
  __IO uint32_t  DPLLCTRLB;                         /*!< DPLL Control B                                                        */
  __IO uint8_t   DPLLPRESC;                         /*!< DPLL Prescaler                                                        */
  __I  uint8_t   RESERVED4[3];
  __I  uint8_t   DPLLSYNCBUSY;                      /*!< DPLL Synchronization Busy                                             */
  __I  uint8_t   RESERVED5[3];
  __I  uint8_t   DPLLSTATUS;                        /*!< DPLL Status                                                           */
} OSCCTRL_Type;


/* ================================================================================ */
/* ================                   OSC32KCTRL                   ================ */
/* ================================================================================ */


/**
  * @brief 32k Oscillators Control (OSC32KCTRL)
  */

typedef struct {                                    /*!< OSC32KCTRL Structure                                                  */
  __IO uint32_t  INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint32_t  INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint32_t  INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint32_t  STATUS;                            /*!< Power and Clocks Status                                               */
  __IO uint8_t   RTCCTRL;                           /*!< RTC Clock Selection                                                   */
  __I  uint8_t   RESERVED[3];
  __IO uint16_t  XOSC32K;                           /*!< 32kHz External Crystal Oscillator (XOSC32K) Control                   */
  __IO uint8_t   CFDCTRL;                           /*!< Clock Failure Detector Control                                        */
  __IO uint8_t   EVCTRL;                            /*!< Event Control                                                         */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  OSCULP32K;                         /*!< 32kHz Ultra Low Power Internal Oscillator (OSCULP32K) Control         */
} OSC32KCTRL_Type;


/* ================================================================================ */
/* ================                       PAC                      ================ */
/* ================================================================================ */


/**
  * @brief Peripheral Access Controller (PAC)
  */

typedef struct {                                    /*!< PAC Structure                                                         */
  __IO uint32_t  WRCTRL;                            /*!< Write control                                                         */
  __IO uint8_t   EVCTRL;                            /*!< Event control                                                         */
  __I  uint8_t   RESERVED[3];
  __IO uint8_t   INTENCLR;                          /*!< Interrupt enable clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt enable set                                                  */
  __I  uint16_t  RESERVED1[3];
  __IO uint32_t  INTFLAGAHB;                        /*!< Bridge interrupt flag status                                          */
  __IO uint32_t  INTFLAGA;                          /*!< Peripheral interrupt flag status - Bridge A                           */
  __IO uint32_t  INTFLAGB;                          /*!< Peripheral interrupt flag status - Bridge B                           */
  __IO uint32_t  INTFLAGC;                          /*!< Peripheral interrupt flag status - Bridge C                           */
  __I  uint32_t  RESERVED2[5];
  __I  uint32_t  STATUSA;                           /*!< Peripheral write protection status - Bridge A                         */
  __I  uint32_t  STATUSB;                           /*!< Peripheral write protection status - Bridge B                         */
  __I  uint32_t  STATUSC;                           /*!< Peripheral write protection status - Bridge C                         */
  __I  uint32_t  RESERVED3[5];
  __I  uint32_t  NONSECA;                           /*!< Peripheral non-secure status - Bridge A                               */
  __I  uint32_t  NONSECB;                           /*!< Peripheral non-secure status - Bridge B                               */
  __I  uint32_t  NONSECC;                           /*!< Peripheral non-secure status - Bridge C                               */
  __I  uint32_t  RESERVED4[5];
  __I  uint32_t  SECLOCKA;                          /*!< Peripheral secure status locked - Bridge A                            */
  __I  uint32_t  SECLOCKB;                          /*!< Peripheral secure status locked - Bridge B                            */
  __I  uint32_t  SECLOCKC;                          /*!< Peripheral secure status locked - Bridge C                            */
} PAC_Type;


/* ================================================================================ */
/* ================                       PM                       ================ */
/* ================================================================================ */


/**
  * @brief Power Manager (PM)
  */

typedef struct {                                    /*!< PM Structure                                                          */
  __I  uint8_t   RESERVED;
  __IO uint8_t   SLEEPCFG;                          /*!< Sleep Configuration                                                   */
  __IO uint8_t   PLCFG;                             /*!< Performance Level Configuration                                       */
  __IO uint8_t   PWCFG;                             /*!< Power Configuration                                                   */
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint8_t   RESERVED1;
  __IO uint16_t  STDBYCFG;                          /*!< Standby Configuration                                                 */
} PM_Type;


/* ================================================================================ */
/* ================                      PORT                      ================ */
/* ================================================================================ */


/**
  * @brief Port Module (PORT)
  */

typedef struct {                                    /*!< PORT Structure                                                        */
  PORT_GROUP_Type GROUP[1];                         /*!< GROUP0                                                                */
} PORT_Type;


/* ================================================================================ */
/* ================                      RSTC                      ================ */
/* ================================================================================ */


/**
  * @brief Reset Controller (RSTC)
  */

typedef struct {                                    /*!< RSTC Structure                                                        */
  __I  uint8_t   RCAUSE;                            /*!< Reset Cause                                                           */
} RSTC_Type;


/* ================================================================================ */
/* ================                       RTC                      ================ */
/* ================================================================================ */


/**
  * @brief Real-Time Counter (RTC)
  */

typedef struct {                                    /*!< RTC Structure                                                         */
  
  union {
    RtcMode2_Type MODE2;                            /*!< Clock/Calendar with Alarm                                             */
    RtcMode1_Type MODE1;                            /*!< 16-bit Counter with Two 16-bit Compares                               */
    RtcMode0_Type MODE0;                            /*!< 32-bit Counter with Single 32-bit Compare                             */
  };
  // SVD file produced an error here.
  //__I  uint32_t  RESERVED[1073741804];
  //#error "Reserved bytes calculation negative: -80"
  RTC_MODE2_ALARM_Type MODE2_ALARM[1];              /*!< MODE2_ALARM0                                                          */
} RTC_Type;


/* ================================================================================ */
/* ================                     SERCOM0                    ================ */
/* ================================================================================ */


/**
  * @brief Serial Communication Interface (SERCOM0)
  */

typedef struct {                                    /*!< SERCOM0 Structure                                                     */
  
  union {
    SercomSpi_Type SPI;                             /*!< SPI Mode                                                              */
    SercomUsart_Type USART;                         /*!< USART Mode                                                            */
    SercomI2cm_Type I2CM;                           /*!< I2C Master Mode                                                       */
    SercomI2cs_Type I2CS;                           /*!< I2C Slave Mode                                                        */
  };
} SERCOM0_Type;


/* ================================================================================ */
/* ================                      SUPC                      ================ */
/* ================================================================================ */


/**
  * @brief Supply Controller (SUPC)
  */

typedef struct {                                    /*!< SUPC Structure                                                        */
  __IO uint32_t  INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint32_t  INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint32_t  INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint32_t  STATUS;                            /*!< Power and Clocks Status                                               */
  __IO uint32_t  BOD33;                             /*!< BOD33 Control                                                         */
  __IO uint32_t  BOD12;                             /*!< BOD12 Control                                                         */
  __IO uint32_t  VREG;                              /*!< VREG Control                                                          */
  __IO uint32_t  VREF;                              /*!< VREF Control                                                          */
  __I  uint32_t  RESERVED[3];
  __IO uint32_t  EVCTRL;                            /*!< Event Control                                                         */
  __IO uint32_t  VREGSUSP;                          /*!< VREG Suspend Control                                                  */
} SUPC_Type;


/* ================================================================================ */
/* ================                       TC0                      ================ */
/* ================================================================================ */


/**
  * @brief Basic Timer Counter (TC0)
  */

typedef struct {                                    /*!< TC0 Structure                                                         */
  
  union {
    TcCount32_Type COUNT32;                         /*!< 32-bit Counter Mode                                                   */
    TcCount16_Type COUNT16;                         /*!< 16-bit Counter Mode                                                   */
    TcCount8_Type COUNT8;                           /*!< 8-bit Counter Mode                                                    */
  };
} TC0_Type;


/* ================================================================================ */
/* ================                      TRAM                      ================ */
/* ================================================================================ */


/**
  * @brief TrustRAM (TRAM)
  */

typedef struct {                                    /*!< TRAM Structure                                                        */
  __IO uint8_t   CTRLA;                             /*!< Control                                                               */
  __I  uint8_t   RESERVED[3];
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint8_t   STATUS;                            /*!< Status                                                                */
  __I  uint32_t  SYNCBUSY;                          /*!< Synchronization Busy Status                                           */
  __O  uint32_t  DSCC;                              /*!< Data Scramble Control                                                 */
  __O  uint8_t   PERMW;                             /*!< Permutation Write                                                     */
  __I  uint8_t   PERMR;                             /*!< Permutation Read                                                      */
  __I  uint16_t  RESERVED1[119];
  __IO uint32_t  RAM[64];                           /*!< TrustRAM                                                              */
} TRAM_Type;


/* ================================================================================ */
/* ================                      TRNG                      ================ */
/* ================================================================================ */


/**
  * @brief True Random Generator (TRNG)
  */

typedef struct {                                    /*!< TRNG Structure                                                        */
  __IO uint8_t   CTRLA;                             /*!< Control A                                                             */
  __I  uint8_t   RESERVED[3];
  __IO uint8_t   EVCTRL;                            /*!< Event Control                                                         */
  __I  uint8_t   RESERVED1[3];
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint8_t   RESERVED2[21];
  __I  uint32_t  DATA;                              /*!< Output Data                                                           */
} TRNG_Type;


/* ================================================================================ */
/* ================                       WDT                      ================ */
/* ================================================================================ */


/**
  * @brief Watchdog Timer (WDT)
  */

typedef struct {                                    /*!< WDT Structure                                                         */
  __IO uint8_t   CTRLA;                             /*!< Control                                                               */
  __IO uint8_t   CONFIG;                            /*!< Configuration                                                         */
  __IO uint8_t   EWCTRL;                            /*!< Early Warning Interrupt Control                                       */
  __I  uint8_t   RESERVED;
  __IO uint8_t   INTENCLR;                          /*!< Interrupt Enable Clear                                                */
  __IO uint8_t   INTENSET;                          /*!< Interrupt Enable Set                                                  */
  __IO uint8_t   INTFLAG;                           /*!< Interrupt Flag Status and Clear                                       */
  __I  uint8_t   RESERVED1;
  __I  uint32_t  SYNCBUSY;                          /*!< Synchronization Busy                                                  */
  __O  uint8_t   CLEAR;                             /*!< Clear                                                                 */
} WDT_Type;


/* ================================================================================ */
/* ================                    CoreDebug                   ================ */
/* ================================================================================ */


/**
  * @brief Debug Control Block (CoreDebug)
  */

typedef struct {                                    /*!< CoreDebug Structure                                                   */
  __IO uint32_t  DHCSR;                             /*!< Debug Halting Control and Status Register                             */
  __O  uint32_t  DCRSR;                             /*!< Debug Core Register Select Register                                   */
  __I  uint32_t  RESERVED;
  __IO uint32_t  DEMCR;                             /*!< Debug Exception and Monitor Control Register                          */
  __I  uint32_t  RESERVED1[2];
  __IO uint32_t  DSCSR;                             /*!< Debug Security Control and Status Register                            */
} CoreDebug_Type;


/* ================================================================================ */
/* ================                       DIB                      ================ */
/* ================================================================================ */


/**
  * @brief Debug Identification Block (DIB)
  */

typedef struct {                                    /*!< DIB Structure                                                         */
  __O  uint32_t  DLAR;                              /*!< SCS Software Lock Access Register                                     */
  __I  uint32_t  DLSR;                              /*!< SCS Software Lock Status Register                                     */
  __I  uint32_t  DAUTHSTATUS;                       /*!< Debug Authentication Status Register                                  */
  __I  uint32_t  DDEVARCH;                          /*!< SCS Device Architecture Register                                      */
  __I  uint32_t  RESERVED[3];
  __I  uint32_t  DDEVTYPE;                          /*!< SCS Device Type Register                                              */
  __I  uint32_t  DPIDR4;                            /*!< SCS Peripheral Identification Register 4                              */
  __I  uint32_t  DPIDR5;                            /*!< SCS Peripheral Identification Register 5                              */
  __I  uint32_t  DPIDR6;                            /*!< SCS Peripheral Identification Register 6                              */
  __I  uint32_t  DPIDR7;                            /*!< SCS Peripheral Identification Register 7                              */
  __I  uint32_t  DPIDR0;                            /*!< SCS Peripheral Identification Register 0                              */
  __I  uint32_t  DPIDR1;                            /*!< SCS Peripheral Identification Register 1                              */
  __I  uint32_t  DPIDR2;                            /*!< SCS Peripheral Identification Register 2                              */
  __I  uint32_t  DPIDR3;                            /*!< SCS Peripheral Identification Register 3                              */
  __I  uint32_t  DCIDR0;                            /*!< SCS Component Identification Register 0                               */
  __I  uint32_t  DCIDR1;                            /*!< SCS Component Identification Register 1                               */
  __I  uint32_t  DCIDR2;                            /*!< SCS Component Identification Register 2                               */
  __I  uint32_t  DCIDR3;                            /*!< SCS Component Identification Register 3                               */
} DIB_Type;


/* ================================================================================ */
/* ================                       DWT                      ================ */
/* ================================================================================ */


/**
  * @brief Data Watchpoint and Trace (DWT)
  */

typedef struct {                                    /*!< DWT Structure                                                         */
  __IO uint32_t  DWT_CTRL;                          /*!< DWT Control Register                                                  */
  __I  uint32_t  RESERVED[6];
  __I  uint32_t  DWT_PCSR;                          /*!< DWT Program Counter Sample Register                                   */
  DWT_COMPARATOR_Type COMPARATOR[2];                /*!< COMPARATOR0                                                           */
  __I  uint32_t  RESERVED1[988];
  __O  uint32_t  DWT_LAR;                           /*!< DWT Software Lock Access Register                                     */
  __I  uint32_t  DWT_LSR;                           /*!< DWT Software Lock Status Register                                     */
  __I  uint32_t  RESERVED2;
  __I  uint32_t  DWT_DEVARCH;                       /*!< DWT Device Architecture Register                                      */
  __I  uint32_t  RESERVED3[3];
  __I  uint32_t  DWT_DEVTYPE;                       /*!< DWT Device Type Register                                              */
  __I  uint32_t  DWT_PIDR4;                         /*!< DWT Peripheral Identification Register 4                              */
  __I  uint32_t  DWT_PIDR5;                         /*!< DWT Peripheral Identification Register 5                              */
  __I  uint32_t  DWT_PIDR6;                         /*!< DWT Peripheral Identification Register 6                              */
  __I  uint32_t  DWT_PIDR7;                         /*!< DWT Peripheral Identification Register 7                              */
  __I  uint32_t  DWT_PIDR0;                         /*!< DWT Peripheral Identification Register 0                              */
  __I  uint32_t  DWT_PIDR1;                         /*!< DWT Peripheral Identification Register 1                              */
  __I  uint32_t  DWT_PIDR2;                         /*!< DWT Peripheral Identification Register 2                              */
  __I  uint32_t  DWT_PIDR3;                         /*!< DWT Peripheral Identification Register 3                              */
  __I  uint32_t  DWT_CIDR0;                         /*!< DWT Component Identification Register 0                               */
  __I  uint32_t  DWT_CIDR1;                         /*!< DWT Component Identification Register 1                               */
  __I  uint32_t  DWT_CIDR2;                         /*!< DWT Component Identification Register 2                               */
  __I  uint32_t  DWT_CIDR3;                         /*!< DWT Component Identification Register 3                               */
} DWT_Type;


/* ================================================================================ */
/* ================                       FPB                      ================ */
/* ================================================================================ */


/**
  * @brief Flash Patch and Breakpoint (FPB)
  */

typedef struct {                                    /*!< FPB Structure                                                         */
  __IO uint32_t  FP_CTRL;                           /*!< Flash Patch Control Register                                          */
  __I  uint32_t  FP_REMAP;                          /*!< Flash Patch Remap Register                                            */
  
  union {
    __IO uint32_t  FP_COMP_BREAKPOINT_MODE[4];      /*!< Flash Patch Comparator Register n                                     */
    __IO uint32_t  FP_COMP[4];                      /*!< Flash Patch Comparator Register n                                     */
  };
  __I  uint32_t  RESERVED[998];
  __O  uint32_t  FP_LAR;                            /*!< FPB Software Lock Access Register                                     */
  __I  uint32_t  FP_LSR;                            /*!< FPB Software Lock Status Register                                     */
  __I  uint32_t  RESERVED1;
  __I  uint32_t  FP_DEVARCH;                        /*!< FPB Device Architecture Register                                      */
  __I  uint32_t  RESERVED2[3];
  __I  uint32_t  FP_DEVTYPE;                        /*!< FPB Device Type Register                                              */
  __I  uint32_t  FP_PIDR4;                          /*!< FP Peripheral Identification Register 4                               */
  __I  uint32_t  FP_PIDR5;                          /*!< FP Peripheral Identification Register 5                               */
  __I  uint32_t  FP_PIDR6;                          /*!< FP Peripheral Identification Register 6                               */
  __I  uint32_t  FP_PIDR7;                          /*!< FP Peripheral Identification Register 7                               */
  __I  uint32_t  FP_PIDR0;                          /*!< FP Peripheral Identification Register 0                               */
  __I  uint32_t  FP_PIDR1;                          /*!< FP Peripheral Identification Register 1                               */
  __I  uint32_t  FP_PIDR2;                          /*!< FP Peripheral Identification Register 2                               */
  __I  uint32_t  FP_PIDR3;                          /*!< FP Peripheral Identification Register 3                               */
  __I  uint32_t  FP_CIDR0;                          /*!< FP Component Identification Register 0                                */
  __I  uint32_t  FP_CIDR1;                          /*!< FP Component Identification Register 1                                */
  __I  uint32_t  FP_CIDR2;                          /*!< FP Component Identification Register 2                                */
  __I  uint32_t  FP_CIDR3;                          /*!< FP Component Identification Register 3                                */
} FPB_Type;


/* ================================================================================ */
/* ================                       ICB                      ================ */
/* ================================================================================ */


/**
  * @brief Implementation Control Block (ICB)
  */

typedef struct {                                    /*!< ICB Structure                                                         */
  __I  uint32_t  RESERVED;
  __I  uint32_t  ICTR;                              /*!< Interrupt Controller Type Register                                    */
  __IO uint32_t  ACTLR;                             /*!< Auxiliary Control Register                                            */
} ICB_Type;


/* ================================================================================ */
/* ================                       MPU                      ================ */
/* ================================================================================ */


/**
  * @brief Memory Protection Unit (MPU)
  */

typedef struct {                                    /*!< MPU Structure                                                         */
  __I  uint32_t  MPU_TYPE;                          /*!< MPU Type Register                                                     */
  __IO uint32_t  MPU_CTRL;                          /*!< MPU Control Register                                                  */
  __IO uint32_t  MPU_RNR;                           /*!< MPU Region Number Register                                            */
  __IO uint32_t  MPU_RBAR;                          /*!< MPU Region Base Address Register                                      */
  __IO uint32_t  MPU_RLAR;                          /*!< MPU Region Limit Address Register                                     */
  __I  uint32_t  RESERVED[7];
  __IO uint32_t  MPU_MAIR0;                         /*!< MPU Memory Attribute Indirection Register 0                           */
  __IO uint32_t  MPU_MAIR1;                         /*!< MPU Memory Attribute Indirection Register 1                           */
} MPU_Type;


/* ================================================================================ */
/* ================                      NVIC                      ================ */
/* ================================================================================ */


/**
  * @brief Nested Vectored Interrupt Controller (NVIC)
  */

typedef struct {                                    /*!< NVIC Structure                                                        */
  __IO uint32_t  NVIC_ISER[2];                      /*!< Interrupt Set Enable Register n                                       */
  __I  uint32_t  RESERVED[30];
  __IO uint32_t  NVIC_ICER[2];                      /*!< Interrupt Clear Enable Register n                                     */
  __I  uint32_t  RESERVED1[30];
  __IO uint32_t  NVIC_ISPR[2];                      /*!< Interrupt Set Pending Register n                                      */
  __I  uint32_t  RESERVED2[30];
  __IO uint32_t  NVIC_ICPR[2];                      /*!< Interrupt Clear Pending Register n                                    */
  __I  uint32_t  RESERVED3[30];
  __I  uint32_t  NVIC_IABR[2];                      /*!< Interrupt Active Bit Register n                                       */
  __I  uint32_t  RESERVED4[30];
  __IO uint32_t  NVIC_ITNS[2];                      /*!< Interrupt Target Non-secure Register n                                */
  __I  uint32_t  RESERVED5[30];
  __IO uint32_t  NVIC_IPR[12];                      /*!< Interrupt Priority Register n                                         */
} NVIC_Type;


/* ================================================================================ */
/* ================                       SCB                      ================ */
/* ================================================================================ */


/**
  * @brief System Control Block (SCB)
  */

typedef struct {                                    /*!< SCB Structure                                                         */
  __I  uint32_t  CPUID;                             /*!< CPUID base register                                                   */
  __IO uint32_t  ICSR;                              /*!< Interrupt Control and State Register                                  */
  __IO uint32_t  VTOR;                              /*!< Vector Table Offset Register                                          */
  __IO uint32_t  AIRCR;                             /*!< Application Interrupt and Reset Control Register                      */
  __IO uint32_t  SCR;                               /*!< System Control Register                                               */
  __IO uint32_t  CCR;                               /*!< Configuration and Control Register                                    */
  __I  uint32_t  RESERVED;
  __IO uint32_t  SHPR2;                             /*!< System Handler Priority Register 2                                    */
  __IO uint32_t  SHPR3;                             /*!< System Handler Priority Register 3                                    */
  __IO uint32_t  SHCSR;                             /*!< System Handler Control and State Register                             */
  __I  uint32_t  RESERVED1[2];
  __IO uint32_t  DFSR;                              /*!< Debug Fault Status Register                                           */
  __I  uint32_t  RESERVED2[2];
  __IO uint32_t  AFSR;                              /*!< Auxiliary Fault Status Register                                       */
  __I  uint32_t  RESERVED3[14];
  __I  uint32_t  CLIDR;                             /*!< Cache Level ID Register                                               */
  __I  uint32_t  CTR;                               /*!< Cache Type Register                                                   */
  __I  uint32_t  CCSIDR;                            /*!< Current Cache Size ID register                                        */
  __IO uint32_t  CSSELR;                            /*!< Cache Size Selection Register                                         */
} SCB_Type;


/* ================================================================================ */
/* ================                     SysTick                    ================ */
/* ================================================================================ */


/**
  * @brief SysTick Timer (SysTick)
  */

typedef struct {                                    /*!< SysTick Structure                                                     */
  __IO uint32_t  SYST_CSR;                          /*!< SysTick Control and Status Register                                   */
  __IO uint32_t  SYST_RVR;                          /*!< SysTick Reload Value Register                                         */
  __IO uint32_t  SYST_CVR;                          /*!< SysTick Current Value Register                                        */
  __I  uint32_t  SYST_CALIB;                        /*!< SysTick Calibration Value Register                                    */
} SysTick_Type;


/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define AC_BASE                         0x40003400UL
#define ADC_BASE                        0x42001C00UL
#define CCL_BASE                        0x42002C00UL
#define DAC_BASE                        0x42002000UL
#define DMAC_BASE                       0x41006000UL
#define DSU_BASE                        0x41002000UL
#define DSU_EXT_BASE                    0x41002100UL
#define EIC_BASE                        0x40002800UL
#define EVSYS_BASE                      0x42000000UL
#define FREQM_BASE                      0x40002C00UL
#define GCLK_BASE                       0x40001C00UL
#define IDAU_BASE                       0x41000000UL
#define MCLK_BASE                       0x40000800UL
#define NVMCTRL_BASE                    0x41004000UL
#define OPAMP_BASE                      0x42003000UL
#define OSCCTRL_BASE                    0x40001000UL
#define OSC32KCTRL_BASE                 0x40001400UL
#define PAC_BASE                        0x40000000UL
#define PM_BASE                         0x40000400UL
#define PORT_BASE                       0x40003000UL
#define PORT_IOBUS_BASE                 0x60000000UL
#define PTC_BASE                        0x42002400UL
#define RSTC_BASE                       0x40000C00UL
#define RTC_BASE                        0x40002400UL
#define SERCOM0_BASE                    0x42000400UL
#define SERCOM1_BASE                    0x42000800UL
#define SERCOM2_BASE                    0x42000C00UL
#define SUPC_BASE                       0x40001800UL
#define TC0_BASE                        0x42001000UL
#define TC1_BASE                        0x42001400UL
#define TC2_BASE                        0x42001800UL
#define TRAM_BASE                       0x42003400UL
#define TRNG_BASE                       0x42002800UL
#define WDT_BASE                        0x40002000UL
#define CoreDebug_BASE                  0xE000EDF0UL
#define DIB_BASE                        0xE000EFB0UL
#define DWT_BASE                        0xE0001000UL
#define FPB_BASE                        0xE0002000UL
#define ICB_BASE                        0xE000E000UL
#define MPU_BASE                        0xE000ED90UL
#define NVIC_BASE                       0xE000E100UL
#define SCB_BASE                        0xE000ED00UL
#define SysTick_BASE                    0xE000E010UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define AC                              ((AC_Type                 *) AC_BASE)
#define ADC                             ((ADC_Type                *) ADC_BASE)
#define CCL                             ((CCL_Type                *) CCL_BASE)
#define DAC                             ((DAC_Type                *) DAC_BASE)
#define DMAC                            ((DMAC_Type               *) DMAC_BASE)
#define DSU                             ((DSU_Type                *) DSU_BASE)
#define DSU_EXT                         ((DSU_Type                *) DSU_EXT_BASE)
#define EIC                             ((EIC_Type                *) EIC_BASE)
#define EVSYS                           ((EVSYS_Type              *) EVSYS_BASE)
#define FREQM                           ((FREQM_Type              *) FREQM_BASE)
#define GCLK                            ((GCLK_Type               *) GCLK_BASE)
#define IDAU                            ((IDAU_Type               *) IDAU_BASE)
#define MCLK                            ((MCLK_Type               *) MCLK_BASE)
#define NVMCTRL                         ((NVMCTRL_Type            *) NVMCTRL_BASE)
#define OPAMP                           ((OPAMP_Type              *) OPAMP_BASE)
#define OSCCTRL                         ((OSCCTRL_Type            *) OSCCTRL_BASE)
#define OSC32KCTRL                      ((OSC32KCTRL_Type         *) OSC32KCTRL_BASE)
#define PAC                             ((PAC_Type                *) PAC_BASE)
#define PM                              ((PM_Type                 *) PM_BASE)
#define PORT                            ((PORT_Type               *) PORT_BASE)
#define PORT_IOBUS                      ((PORT_Type               *) PORT_IOBUS_BASE)
#define PTC                             ((PTC_Type                *) PTC_BASE)
#define RSTC                            ((RSTC_Type               *) RSTC_BASE)
#define RTC                             ((RTC_Type                *) RTC_BASE)
#define SERCOM0                         ((SERCOM0_Type            *) SERCOM0_BASE)
#define SERCOM1                         ((SERCOM0_Type            *) SERCOM1_BASE)
#define SERCOM2                         ((SERCOM0_Type            *) SERCOM2_BASE)
#define SUPC                            ((SUPC_Type               *) SUPC_BASE)
#define TC0                             ((TC0_Type                *) TC0_BASE)
#define TC1                             ((TC0_Type                *) TC1_BASE)
#define TC2                             ((TC0_Type                *) TC2_BASE)
#define TRAM                            ((TRAM_Type               *) TRAM_BASE)
#define TRNG                            ((TRNG_Type               *) TRNG_BASE)
#define WDT                             ((WDT_Type                *) WDT_BASE)
#define CoreDebug                       ((CoreDebug_Type          *) CoreDebug_BASE)
#define DIB                             ((DIB_Type                *) DIB_BASE)
#define DWT                             ((DWT_Type                *) DWT_BASE)
#define FPB                             ((FPB_Type                *) FPB_BASE)
#define ICB                             ((ICB_Type                *) ICB_BASE)
#define MPU                             ((MPU_Type                *) MPU_BASE)
#define NVIC                            ((NVIC_Type               *) NVIC_BASE)
#define SCB                             ((SCB_Type                *) SCB_BASE)
#define SysTick                         ((SysTick_Type            *) SysTick_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group ATSAML10E16A */
/** @} */ /* End of group Microchip Technology */

#ifdef __cplusplus
}
#endif


#endif  /* ATSAML10E16A_H */

