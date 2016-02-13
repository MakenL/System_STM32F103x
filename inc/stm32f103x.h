/**\file    stm32f103x.h
 * \author  Lyga M.A.
 * \version 1.0
 * \date    20.07.2015    
 */
 
#ifndef __STM32F103X_H
#define __STM32F103X_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
#include <stdint.h>
	 
#define __MPU_PRESENT             0                      ///< Other STM32 devices does not provide an MPU
#define __CM3_REV                 0x0200                 ///< Core Revision r2p0
#define __NVIC_PRIO_BITS          4                      ///< STM32 uses 4 Bits for the Priority Levels
#define __Vendor_SysTickConfig    1                      ///< Set to 1 if different SysTick Config is used

#define FLASH_BASE               ((uint32_t)0x08000000)  ///< FLASH base address in the alias region
#define SRAM_BASE                ((uint32_t)0x20000000)  ///< SRAM base address in the alias region
#define PERIPH_BASE              ((uint32_t)0x40000000)  ///< Peripheral base address in the alias region
#define SRAM_BB_BASE             ((uint32_t)0x22000000)  ///< SRAM base address in the bit-band region
#define PERIPH_BB_BASE           ((uint32_t)0x42000000)  ///< Peripheral base address in the bit-band region
#define FSMC_R_BASE              ((uint32_t)0xA0000000)  ///< FSMC registers base address
#define OB_BASE                  ((uint32_t)0x1FFFF800)  ///< Flash Option Bytes base address
#define DBGMCU_BASE              ((uint32_t)0xE0042000)  ///< Debug MCU registers base address

#define APB1PERIPH_BASE          (PERIPH_BASE)
#define APB2PERIPH_BASE          (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE           (PERIPH_BASE + 0x20000)
#define SDIO_BASE                (PERIPH_BASE + 0x18000)

#define TIM2_BASE                (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE                (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE                (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE                (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE                (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE                (APB1PERIPH_BASE + 0x1400)
#define TIM12_BASE               (APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE               (APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE               (APB1PERIPH_BASE + 0x2000)
#define RTC_BASE                 (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE                (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE                (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASE                (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE                (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE              (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE              (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE               (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE               (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE                (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE                (APB1PERIPH_BASE + 0x5800)
#define CAN1_BASE                (APB1PERIPH_BASE + 0x6400)
#define CAN2_BASE                (APB1PERIPH_BASE + 0x6800)
#define BKP_BASE                 (APB1PERIPH_BASE + 0x6C00)
#define PWR_BASE                 (APB1PERIPH_BASE + 0x7000)
#define DAC_BASE                 (APB1PERIPH_BASE + 0x7400)
#define CEC_BASE                 (APB1PERIPH_BASE + 0x7800)

#define AFIO_BASE                (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE                (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE               (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE               (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE               (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE               (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE               (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE               (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE               (APB2PERIPH_BASE + 0x2000)
#define ADC1_BASE                (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE                (APB2PERIPH_BASE + 0x2800)
#define TIM1_BASE                (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE                (APB2PERIPH_BASE + 0x3000)
#define TIM8_BASE                (APB2PERIPH_BASE + 0x3400)
#define USART1_BASE              (APB2PERIPH_BASE + 0x3800)
#define ADC3_BASE                (APB2PERIPH_BASE + 0x3C00)
#define TIM15_BASE               (APB2PERIPH_BASE + 0x4000)
#define TIM16_BASE               (APB2PERIPH_BASE + 0x4400)
#define TIM17_BASE               (APB2PERIPH_BASE + 0x4800)
#define TIM9_BASE                (APB2PERIPH_BASE + 0x4C00)
#define TIM10_BASE               (APB2PERIPH_BASE + 0x5000)
#define TIM11_BASE               (APB2PERIPH_BASE + 0x5400)

#define DMA1_BASE                (AHBPERIPH_BASE + 0x0000)
#define DMA1_Channel1_BASE       (AHBPERIPH_BASE + 0x0008)
#define DMA1_Channel2_BASE       (AHBPERIPH_BASE + 0x001C)
#define DMA1_Channel3_BASE       (AHBPERIPH_BASE + 0x0030)
#define DMA1_Channel4_BASE       (AHBPERIPH_BASE + 0x0044)
#define DMA1_Channel5_BASE       (AHBPERIPH_BASE + 0x0058)
#define DMA1_Channel6_BASE       (AHBPERIPH_BASE + 0x006C)
#define DMA1_Channel7_BASE       (AHBPERIPH_BASE + 0x0080)
#define DMA2_BASE                (AHBPERIPH_BASE + 0x0400)
#define DMA2_Channel1_BASE       (AHBPERIPH_BASE + 0x0408)
#define DMA2_Channel2_BASE       (AHBPERIPH_BASE + 0x041C)
#define DMA2_Channel3_BASE       (AHBPERIPH_BASE + 0x0430)
#define DMA2_Channel4_BASE       (AHBPERIPH_BASE + 0x0444)
#define DMA2_Channel5_BASE       (AHBPERIPH_BASE + 0x0458)
#define RCC_BASE                 (AHBPERIPH_BASE + 0x1000)
#define CRC_BASE                 (AHBPERIPH_BASE + 0x3000)
#define FLASH_R_BASE             (AHBPERIPH_BASE + 0x2000) ///< Flash registers base address

#define FSMC_Bank1_R_BASE        (FSMC_R_BASE + 0x0000)    ///< FSMC Bank1 registers base address
#define FSMC_Bank1E_R_BASE       (FSMC_R_BASE + 0x0104)    ///< FSMC Bank1E registers base address
#define FSMC_Bank2_R_BASE        (FSMC_R_BASE + 0x0060)    ///< FSMC Bank2 registers base address
#define FSMC_Bank3_R_BASE        (FSMC_R_BASE + 0x0080)    ///< FSMC Bank3 registers base address
#define FSMC_Bank4_R_BASE        (FSMC_R_BASE + 0x00A0)    ///< FSMC Bank4 registers base address

typedef enum
{
	RESET = 0, 
  SET = !RESET
} FlagStatus;

typedef enum 
{
  DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;

typedef enum IRQn
{
  // Cortex-M3 Processor Exceptions Numbers
  NonMaskableInt_IRQn         = -14,    ///< 2 Non Maskable Interrupt
  MemoryManagement_IRQn       = -12,    ///< 4 Cortex-M3 Memory Management Interrupt
  BusFault_IRQn               = -11,    ///< 5 Cortex-M3 Bus Fault Interrupt
  UsageFault_IRQn             = -10,    ///< 6 Cortex-M3 Usage Fault Interrupt
  SVCall_IRQn                 = -5,     ///< 11 Cortex-M3 SV Call Interrupt
  DebugMonitor_IRQn           = -4,     ///< 12 Cortex-M3 Debug Monitor Interrupt
  PendSV_IRQn                 = -2,     ///< 14 Cortex-M3 Pend SV Interrupt
  SysTick_IRQn                = -1,     ///< 15 Cortex-M3 System Tick Interrupt
  // STM32 specific Interrupt Numbers
  WWDG_IRQn                   = 0,      ///< Window WatchDog Interrupt
  PVD_IRQn                    = 1,      ///< PVD through EXTI Line detection Interrupt
  TAMPER_IRQn                 = 2,      ///< Tamper Interrupt
  RTC_IRQn                    = 3,      ///< RTC global Interrupt
  FLASH_IRQn                  = 4,      ///< FLASH global Interrupt
  RCC_IRQn                    = 5,      ///< RCC global Interrupt
  EXTI0_IRQn                  = 6,      ///< EXTI Line0 Interrupt
  EXTI1_IRQn                  = 7,      ///< EXTI Line1 Interrupt
  EXTI2_IRQn                  = 8,      ///< EXTI Line2 Interrupt
  EXTI3_IRQn                  = 9,      ///< EXTI Line3 Interrupt
  EXTI4_IRQn                  = 10,     ///< EXTI Line4 Interrupt
  DMA1_Channel1_IRQn          = 11,     ///< DMA1 Channel 1 global Interrupt
  DMA1_Channel2_IRQn          = 12,     ///< DMA1 Channel 2 global Interrupt
  DMA1_Channel3_IRQn          = 13,     ///< DMA1 Channel 3 global Interrupt
  DMA1_Channel4_IRQn          = 14,     ///< DMA1 Channel 4 global Interrupt
  DMA1_Channel5_IRQn          = 15,     ///< DMA1 Channel 5 global Interrupt
  DMA1_Channel6_IRQn          = 16,     ///< DMA1 Channel 6 global Interrupt
  DMA1_Channel7_IRQn          = 17,     ///< DMA1 Channel 7 global Interrupt
  ADC1_2_IRQn                 = 18,     ///< ADC1 and ADC2 global Interrupt
  USB_HP_CAN1_TX_IRQn         = 19,     ///< USB Device High Priority or CAN1 TX Interrupts
  USB_LP_CAN1_RX0_IRQn        = 20,     ///< USB Device Low Priority or CAN1 RX0 Interrupts
  CAN1_RX1_IRQn               = 21,     ///< CAN1 RX1 Interrupt
  CAN1_SCE_IRQn               = 22,     ///< CAN1 SCE Interrupt
  EXTI9_5_IRQn                = 23,     ///< External Line[9:5] Interrupts
  TIM1_BRK_IRQn               = 24,     ///< TIM1 Break Interrupt
  TIM1_UP_IRQn                = 25,     ///< TIM1 Update Interrupt
  TIM1_TRG_COM_IRQn           = 26,     ///< TIM1 Trigger and Commutation Interrupt
  TIM1_CC_IRQn                = 27,     ///< TIM1 Capture Compare Interrupt
  TIM2_IRQn                   = 28,     ///< TIM2 global Interrupt
  TIM3_IRQn                   = 29,     ///< TIM3 global Interrupt
  TIM4_IRQn                   = 30,     ///< TIM4 global Interrupt
  I2C1_EV_IRQn                = 31,     ///< I2C1 Event Interrupt
  I2C1_ER_IRQn                = 32,     ///< I2C1 Error Interrupt
  I2C2_EV_IRQn                = 33,     ///< I2C2 Event Interrupt
  I2C2_ER_IRQn                = 34,     ///< I2C2 Error Interrupt
  SPI1_IRQn                   = 35,     ///< SPI1 global Interrupt
  SPI2_IRQn                   = 36,     ///< SPI2 global Interrupt
  USART1_IRQn                 = 37,     ///< USART1 global Interrupt
  USART2_IRQn                 = 38,     ///< USART2 global Interrupt
  USART3_IRQn                 = 39,     ///< USART3 global Interrupt
  EXTI15_10_IRQn              = 40,     ///< External Line[15:10] Interrupts
  RTCAlarm_IRQn               = 41,     ///< RTC Alarm through EXTI Line Interrupt
  USBWakeUp_IRQn              = 42,     ///< USB Device WakeUp from suspend through EXTI Line Interrupt
  TIM8_BRK_IRQn               = 43,     ///< TIM8 Break Interrupt
  TIM8_UP_IRQn                = 44,     ///< TIM8 Update Interrupt
  TIM8_TRG_COM_IRQn           = 45,     ///< TIM8 Trigger and Commutation Interrupt
  TIM8_CC_IRQn                = 46,     ///< TIM8 Capture Compare Interrupt
  ADC3_IRQn                   = 47,     ///< ADC3 global Interrupt
  FSMC_IRQn                   = 48,     ///< FSMC global Interrupt
  SDIO_IRQn                   = 49,     ///<  SDIO global Interrupt
  TIM5_IRQn                   = 50,     ///< TIM5 global Interrupt
  SPI3_IRQn                   = 51,     ///< SPI3 global Interrupt
  UART4_IRQn                  = 52,     ///< UART4 global Interrupt
  UART5_IRQn                  = 53,     ///< UART5 global Interrupt
  TIM6_IRQn                   = 54,     ///< TIM6 global Interrupt
  TIM7_IRQn                   = 55,     ///< TIM7 global Interrupt
  DMA2_Channel1_IRQn          = 56,     ///< DMA2 Channel 1 global Interrupt
  DMA2_Channel2_IRQn          = 57,     ///< DMA2 Channel 2 global Interrupt
  DMA2_Channel3_IRQn          = 58,     ///< DMA2 Channel 3 global Interrupt
  DMA2_Channel4_5_IRQn        = 59      ///< DMA2 Channel 4 and Channel 5 global Interrupt
} IRQn_Type;

#include "core_cm3.h"

// Analog to Digital Converter
typedef struct
{
  __IO uint32_t SR;
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SMPR1;
  __IO uint32_t SMPR2;
  __IO uint32_t JOFR1;
  __IO uint32_t JOFR2;
  __IO uint32_t JOFR3;
  __IO uint32_t JOFR4;
  __IO uint32_t HTR;
  __IO uint32_t LTR;
  __IO uint32_t SQR1;
  __IO uint32_t SQR2;
  __IO uint32_t SQR3;
  __IO uint32_t JSQR;
  __IO uint32_t JDR1;
  __IO uint32_t JDR2;
  __IO uint32_t JDR3;
  __IO uint32_t JDR4;
  __IO uint32_t DR;
} ADC_TypeDef;

// Backup Registers
typedef struct
{
  uint32_t  RESERVED0;
  __IO uint16_t DR1;
  uint16_t  RESERVED1;
  __IO uint16_t DR2;
  uint16_t  RESERVED2;
  __IO uint16_t DR3;
  uint16_t  RESERVED3;
  __IO uint16_t DR4;
  uint16_t  RESERVED4;
  __IO uint16_t DR5;
  uint16_t  RESERVED5;
  __IO uint16_t DR6;
  uint16_t  RESERVED6;
  __IO uint16_t DR7;
  uint16_t  RESERVED7;
  __IO uint16_t DR8;
  uint16_t  RESERVED8;
  __IO uint16_t DR9;
  uint16_t  RESERVED9;
  __IO uint16_t DR10;
  uint16_t  RESERVED10; 
  __IO uint16_t RTCCR;
  uint16_t  RESERVED11;
  __IO uint16_t CR;
  uint16_t  RESERVED12;
  __IO uint16_t CSR;
  uint16_t  RESERVED13[5];
  __IO uint16_t DR11;
  uint16_t  RESERVED14;
  __IO uint16_t DR12;
  uint16_t  RESERVED15;
  __IO uint16_t DR13;
  uint16_t  RESERVED16;
  __IO uint16_t DR14;
  uint16_t  RESERVED17;
  __IO uint16_t DR15;
  uint16_t  RESERVED18;
  __IO uint16_t DR16;
  uint16_t  RESERVED19;
  __IO uint16_t DR17;
  uint16_t  RESERVED20;
  __IO uint16_t DR18;
  uint16_t  RESERVED21;
  __IO uint16_t DR19;
  uint16_t  RESERVED22;
  __IO uint16_t DR20;
  uint16_t  RESERVED23;
  __IO uint16_t DR21;
  uint16_t  RESERVED24;
  __IO uint16_t DR22;
  uint16_t  RESERVED25;
  __IO uint16_t DR23;
  uint16_t  RESERVED26;
  __IO uint16_t DR24;
  uint16_t  RESERVED27;
  __IO uint16_t DR25;
  uint16_t  RESERVED28;
  __IO uint16_t DR26;
  uint16_t  RESERVED29;
  __IO uint16_t DR27;
  uint16_t  RESERVED30;
  __IO uint16_t DR28;
  uint16_t  RESERVED31;
  __IO uint16_t DR29;
  uint16_t  RESERVED32;
  __IO uint16_t DR30;
  uint16_t  RESERVED33; 
  __IO uint16_t DR31;
  uint16_t  RESERVED34;
  __IO uint16_t DR32;
  uint16_t  RESERVED35;
  __IO uint16_t DR33;
  uint16_t  RESERVED36;
  __IO uint16_t DR34;
  uint16_t  RESERVED37;
  __IO uint16_t DR35;
  uint16_t  RESERVED38;
  __IO uint16_t DR36;
  uint16_t  RESERVED39;
  __IO uint16_t DR37;
  uint16_t  RESERVED40;
  __IO uint16_t DR38;
  uint16_t  RESERVED41;
  __IO uint16_t DR39;
  uint16_t  RESERVED42;
  __IO uint16_t DR40;
  uint16_t  RESERVED43;
  __IO uint16_t DR41;
  uint16_t  RESERVED44;
  __IO uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;
  
// Controller Area Network TxMailBox
typedef struct
{
  __IO uint32_t TIR;
  __IO uint32_t TDTR;
  __IO uint32_t TDLR;
  __IO uint32_t TDHR;
} CAN_TxMailBox_TypeDef;

// Controller Area Network FIFOMailBox
typedef struct
{
  __IO uint32_t RIR;
  __IO uint32_t RDTR;
  __IO uint32_t RDLR;
  __IO uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;

// Controller Area Network FilterRegister
typedef struct
{
  __IO uint32_t FR1;
  __IO uint32_t FR2;
} CAN_FilterRegister_TypeDef;

// Controller Area Network
typedef struct
{
  __IO uint32_t MCR;
  __IO uint32_t MSR;
  __IO uint32_t TSR;
  __IO uint32_t RF0R;
  __IO uint32_t RF1R;
  __IO uint32_t IER;
  __IO uint32_t ESR;
  __IO uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  __IO uint32_t FMR;
  __IO uint32_t FM1R;
  uint32_t  RESERVED2;
  __IO uint32_t FS1R;
  uint32_t  RESERVED3;
  __IO uint32_t FFA1R;
  uint32_t  RESERVED4;
  __IO uint32_t FA1R;
  uint32_t  RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14]; 
} CAN_TypeDef;

// Consumer Electronics Control (CEC)
typedef struct
{
  __IO uint32_t CFGR;
  __IO uint32_t OAR;
  __IO uint32_t PRES;
  __IO uint32_t ESR;
  __IO uint32_t CSR;
  __IO uint32_t TXD;
  __IO uint32_t RXD;  
} CEC_TypeDef;

// CRC calculation unit
typedef struct
{
  __IO uint32_t DR;
  __IO uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  __IO uint32_t CR;
} CRC_TypeDef;

// Digital to Analog Converter
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t SWTRIGR;
  __IO uint32_t DHR12R1;
  __IO uint32_t DHR12L1;
  __IO uint32_t DHR8R1;
  __IO uint32_t DHR12R2;
  __IO uint32_t DHR12L2;
  __IO uint32_t DHR8R2;
  __IO uint32_t DHR12RD;
  __IO uint32_t DHR12LD;
  __IO uint32_t DHR8RD;
  __IO uint32_t DOR1;
  __IO uint32_t DOR2;
} DAC_TypeDef;

// Debug MCU
typedef struct
{
  __IO uint32_t IDCODE;
  __IO uint32_t CR;	
}DBGMCU_TypeDef;

// DMA Controller
typedef struct
{
  __IO uint32_t CCR;
  __IO uint32_t CNDTR;
  __IO uint32_t CPAR;
  __IO uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t ISR;
  __IO uint32_t IFCR;
} DMA_TypeDef;

// External Interrupt/Event Controller
typedef struct
{
  __IO uint32_t IMR;
  __IO uint32_t EMR;
  __IO uint32_t RTSR;
  __IO uint32_t FTSR;
  __IO uint32_t SWIER;
  __IO uint32_t PR;
} EXTI_TypeDef;

// FLASH Registers
typedef struct
{
  __IO uint32_t ACR;
  __IO uint32_t KEYR;
  __IO uint32_t OPTKEYR;
  __IO uint32_t SR;
  __IO uint32_t CR;
  __IO uint32_t AR;
  __IO uint32_t RESERVED;
  __IO uint32_t OBR;
  __IO uint32_t WRPR;
} FLASH_TypeDef;

// Option Bytes Registers
typedef struct
{
  __IO uint16_t RDP;
  __IO uint16_t USER;
  __IO uint16_t Data0;
  __IO uint16_t Data1;
  __IO uint16_t WRP0;
  __IO uint16_t WRP1;
  __IO uint16_t WRP2;
  __IO uint16_t WRP3;
} OB_TypeDef;

// Flexible Static Memory Controller
typedef struct
{
  __IO uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 

// Flexible Static Memory Controller Bank1E
typedef struct
{
  __IO uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;

// Flexible Static Memory Controller Bank2
typedef struct
{
  __IO uint32_t PCR2;
  __IO uint32_t SR2;
  __IO uint32_t PMEM2;
  __IO uint32_t PATT2;
  uint32_t  RESERVED0;   
  __IO uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  

// Flexible Static Memory Controller Bank3
typedef struct
{
  __IO uint32_t PCR3;
  __IO uint32_t SR3;
  __IO uint32_t PMEM3;
  __IO uint32_t PATT3;
  uint32_t  RESERVED0;   
  __IO uint32_t ECCR3; 
} FSMC_Bank3_TypeDef; 

// Flexible Static Memory Controller Bank4
typedef struct
{
  __IO uint32_t PCR4;
  __IO uint32_t SR4;
  __IO uint32_t PMEM4;
  __IO uint32_t PATT4;
  __IO uint32_t PIO4; 
} FSMC_Bank4_TypeDef; 

// General Purpose I/O
typedef struct
{
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;

// Alternate Function I/O
typedef struct
{
  __IO uint32_t EVCR;
  __IO uint32_t MAPR;
  __IO uint32_t EXTICR[4];
  uint32_t RESERVED0;
  __IO uint32_t MAPR2;  
} AFIO_TypeDef;

// Inter Integrated Circuit Interface
typedef struct
{
  __IO uint16_t CR1;
  uint16_t  RESERVED0;
  __IO uint16_t CR2;
  uint16_t  RESERVED1;
  __IO uint16_t OAR1;
  uint16_t  RESERVED2;
  __IO uint16_t OAR2;
  uint16_t  RESERVED3;
  __IO uint16_t DR;
  uint16_t  RESERVED4;
  __IO uint16_t SR1;
  uint16_t  RESERVED5;
  __IO uint16_t SR2;
  uint16_t  RESERVED6;
  __IO uint16_t CCR;
  uint16_t  RESERVED7;
  __IO uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;

// Independent WATCHDOG
typedef struct
{
  __IO uint32_t KR;
  __IO uint32_t PR;
  __IO uint32_t RLR;
  __IO uint32_t SR;
} IWDG_TypeDef;

// Power Control
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CSR;
} PWR_TypeDef;

// Reset and Clock Control
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
} RCC_TypeDef;

// Real-Time Clock
typedef struct
{
  __IO uint16_t CRH;
  uint16_t  RESERVED0;
  __IO uint16_t CRL;
  uint16_t  RESERVED1;
  __IO uint16_t PRLH;
  uint16_t  RESERVED2;
  __IO uint16_t PRLL;
  uint16_t  RESERVED3;
  __IO uint16_t DIVH;
  uint16_t  RESERVED4;
  __IO uint16_t DIVL;
  uint16_t  RESERVED5;
  __IO uint16_t CNTH;
  uint16_t  RESERVED6;
  __IO uint16_t CNTL;
  uint16_t  RESERVED7;
  __IO uint16_t ALRH;
  uint16_t  RESERVED8;
  __IO uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;

// SD host Interface
typedef struct
{
  __IO uint32_t POWER;
  __IO uint32_t CLKCR;
  __IO uint32_t ARG;
  __IO uint32_t CMD;
  __I uint32_t RESPCMD;
  __I uint32_t RESP1;
  __I uint32_t RESP2;
  __I uint32_t RESP3;
  __I uint32_t RESP4;
  __IO uint32_t DTIMER;
  __IO uint32_t DLEN;
  __IO uint32_t DCTRL;
  __I uint32_t DCOUNT;
  __I uint32_t STA;
  __IO uint32_t ICR;
  __IO uint32_t MASK;
  uint32_t  RESERVED0[2];
  __I uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  __IO uint32_t FIFO;
} SDIO_TypeDef;

// Serial Peripheral Interface
typedef struct
{
  __IO uint16_t CR1;
  uint16_t  RESERVED0;
  __IO uint16_t CR2;
  uint16_t  RESERVED1;
  __IO uint16_t SR;
  uint16_t  RESERVED2;
  __IO uint16_t DR;
  uint16_t  RESERVED3;
  __IO uint16_t CRCPR;
  uint16_t  RESERVED4;
  __IO uint16_t RXCRCR;
  uint16_t  RESERVED5;
  __IO uint16_t TXCRCR;
  uint16_t  RESERVED6;
  __IO uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  __IO uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;

// TIM
typedef struct
{
  __IO uint16_t CR1;
  uint16_t  RESERVED0;
  __IO uint16_t CR2;
  uint16_t  RESERVED1;
  __IO uint16_t SMCR;
  uint16_t  RESERVED2;
  __IO uint16_t DIER;
  uint16_t  RESERVED3;
  __IO uint16_t SR;
  uint16_t  RESERVED4;
  __IO uint16_t EGR;
  uint16_t  RESERVED5;
  __IO uint16_t CCMR1;
  uint16_t  RESERVED6;
  __IO uint16_t CCMR2;
  uint16_t  RESERVED7;
  __IO uint16_t CCER;
  uint16_t  RESERVED8;
  __IO uint16_t CNT;
  uint16_t  RESERVED9;
  __IO uint16_t PSC;
  uint16_t  RESERVED10;
  __IO uint16_t ARR;
  uint16_t  RESERVED11;
  __IO uint16_t RCR;
  uint16_t  RESERVED12;
  __IO uint16_t CCR1;
  uint16_t  RESERVED13;
  __IO uint16_t CCR2;
  uint16_t  RESERVED14;
  __IO uint16_t CCR3;
  uint16_t  RESERVED15;
  __IO uint16_t CCR4;
  uint16_t  RESERVED16;
  __IO uint16_t BDTR;
  uint16_t  RESERVED17;
  __IO uint16_t DCR;
  uint16_t  RESERVED18;
  __IO uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;

// Universal Synchronous Asynchronous Receiver Transmitter
typedef struct
{
  __IO uint16_t SR;
  uint16_t  RESERVED0;
  __IO uint16_t DR;
  uint16_t  RESERVED1;
  __IO uint16_t BRR;
  uint16_t  RESERVED2;
  __IO uint16_t CR1;
  uint16_t  RESERVED3;
  __IO uint16_t CR2;
  uint16_t  RESERVED4;
  __IO uint16_t CR3;
  uint16_t  RESERVED5;
  __IO uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;

// Window WATCHDOG
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFR;
  __IO uint32_t SR;
} WWDG_TypeDef;

/************************************************************************************************
 *                                                                                              *
 *                                     Peripheral Registers                                     *
 *                                                                                              *
 ************************************************************************************************/

#define TIM2                     ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                     ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                     ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                     ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                     ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                     ((TIM_TypeDef *) TIM7_BASE)
#define TIM12                    ((TIM_TypeDef *) TIM12_BASE)
#define TIM13                    ((TIM_TypeDef *) TIM13_BASE)
#define TIM14                    ((TIM_TypeDef *) TIM14_BASE)
#define RTC                      ((RTC_TypeDef *) RTC_BASE)
#define WWDG                     ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                     ((IWDG_TypeDef *) IWDG_BASE)
#define SPI2                     ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                     ((SPI_TypeDef *) SPI3_BASE)
#define USART2                   ((USART_TypeDef *) USART2_BASE)
#define USART3                   ((USART_TypeDef *) USART3_BASE)
#define UART4                    ((USART_TypeDef *) UART4_BASE)
#define UART5                    ((USART_TypeDef *) UART5_BASE)
#define I2C1                     ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                     ((I2C_TypeDef *) I2C2_BASE)
#define CAN1                     ((CAN_TypeDef *) CAN1_BASE)
#define CAN2                     ((CAN_TypeDef *) CAN2_BASE)
#define BKP                      ((BKP_TypeDef *) BKP_BASE)
#define PWR                      ((PWR_TypeDef *) PWR_BASE)
#define DAC                      ((DAC_TypeDef *) DAC_BASE)
#define CEC                      ((CEC_TypeDef *) CEC_BASE)
#define AFIO                     ((AFIO_TypeDef *) AFIO_BASE)
#define EXTI                     ((EXTI_TypeDef *) EXTI_BASE)
#define GPIOA                    ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB                    ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC                    ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD                    ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE                    ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF                    ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG                    ((GPIO_TypeDef *) GPIOG_BASE)
#define ADC1                     ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                     ((ADC_TypeDef *) ADC2_BASE)
#define TIM1                     ((TIM_TypeDef *) TIM1_BASE)
#define SPI1                     ((SPI_TypeDef *) SPI1_BASE)
#define TIM8                     ((TIM_TypeDef *) TIM8_BASE)
#define USART1                   ((USART_TypeDef *) USART1_BASE)
#define ADC3                     ((ADC_TypeDef *) ADC3_BASE)
#define TIM15                    ((TIM_TypeDef *) TIM15_BASE)
#define TIM16                    ((TIM_TypeDef *) TIM16_BASE)
#define TIM17                    ((TIM_TypeDef *) TIM17_BASE)
#define TIM9                     ((TIM_TypeDef *) TIM9_BASE)
#define TIM10                    ((TIM_TypeDef *) TIM10_BASE)
#define TIM11                    ((TIM_TypeDef *) TIM11_BASE)
#define SDIO                     ((SDIO_TypeDef *) SDIO_BASE)
#define DMA1                     ((DMA_TypeDef *) DMA1_BASE)
#define DMA2                     ((DMA_TypeDef *) DMA2_BASE)
#define DMA1_Channel1            ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Channel2            ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Channel3            ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Channel4            ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Channel5            ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#define DMA1_Channel6            ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE)
#define DMA1_Channel7            ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE)
#define DMA2_Channel1            ((DMA_Channel_TypeDef *) DMA2_Channel1_BASE)
#define DMA2_Channel2            ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE)
#define DMA2_Channel3            ((DMA_Channel_TypeDef *) DMA2_Channel3_BASE)
#define DMA2_Channel4            ((DMA_Channel_TypeDef *) DMA2_Channel4_BASE)
#define DMA2_Channel5            ((DMA_Channel_TypeDef *) DMA2_Channel5_BASE)
#define RCC                      ((RCC_TypeDef *) RCC_BASE)
#define CRC                      ((CRC_TypeDef *) CRC_BASE)
#define FLASH                    ((FLASH_TypeDef *) FLASH_R_BASE)
#define OB                       ((OB_TypeDef *) OB_BASE) 
#define FSMC_Bank1               ((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE)
#define FSMC_Bank1E              ((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE)
#define FSMC_Bank2               ((FSMC_Bank2_TypeDef *) FSMC_Bank2_R_BASE)
#define FSMC_Bank3               ((FSMC_Bank3_TypeDef *) FSMC_Bank3_R_BASE)
#define FSMC_Bank4               ((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE)
#define DBGMCU                   ((DBGMCU_TypeDef *) DBGMCU_BASE)

/************************************************************************************************
 *                                                                                              *
 *                          Peripheral registers bits definition                                *
 *                                                                                              *
 ************************************************************************************************/

/************************************************************************************************
 *                                                                                              *
 *                                       Power Control                                          *
 *                                                                                              *
 ************************************************************************************************/

/* Bit definition for PWR_CR register
 ************************************************************************************************/
#define  PWR_CR_LPDS             ((uint16_t)0x0001)      ///< Low-Power Deepsleep
#define  PWR_CR_PDDS             ((uint16_t)0x0002)      ///< Power Down Deepsleep
#define  PWR_CR_CWUF             ((uint16_t)0x0004)      ///< Clear Wakeup Flag
#define  PWR_CR_CSBF             ((uint16_t)0x0008)      ///< Clear Standby Flag
#define  PWR_CR_PVDE             ((uint16_t)0x0010)      ///< Power Voltage Detector Enable

#define  PWR_CR_PLS              ((uint16_t)0x00E0)      ///< PLS[2:0] bits (PVD Level Selection)

#define  PWR_CR_PLS_2V2          ((uint16_t)0x0000)      ///< PVD level 2.2V
#define  PWR_CR_PLS_2V3          ((uint16_t)0x0020)      ///< PVD level 2.3V
#define  PWR_CR_PLS_2V4          ((uint16_t)0x0040)      ///< PVD level 2.4V
#define  PWR_CR_PLS_2V5          ((uint16_t)0x0060)      ///< PVD level 2.5V
#define  PWR_CR_PLS_2V6          ((uint16_t)0x0080)      ///< PVD level 2.6V
#define  PWR_CR_PLS_2V7          ((uint16_t)0x00A0)      ///< PVD level 2.7V
#define  PWR_CR_PLS_2V8          ((uint16_t)0x00C0)      ///< PVD level 2.8V
#define  PWR_CR_PLS_2V9          ((uint16_t)0x00E0)      ///< PVD level 2.9V

#define  PWR_CR_DBP              ((uint16_t)0x0100)      ///< Disable Backup Domain write protection

/* Bit definition for PWR_CSR register
 ************************************************************************************************/
#define  PWR_CSR_WUF             ((uint16_t)0x0001)      ///< Wakeup Flag
#define  PWR_CSR_SBF             ((uint16_t)0x0002)      ///< Standby Flag
#define  PWR_CSR_PVDO            ((uint16_t)0x0004)      ///< PVD Output
#define  PWR_CSR_EWUP            ((uint16_t)0x0100)      ///< Enable WKUP pin

/************************************************************************************************
 *                                                                                              *
 *                                   Reset and Clock Control                                    *
 *                                                                                              *
 ************************************************************************************************/
 
/* Bit definition for RCC_CR register
 ************************************************************************************************/
#define  RCC_CR_HSION            ((uint32_t)0x00000001)  ///< Включение внутреннего генератора (HSI)
#define  RCC_CR_HSIRDY           ((uint32_t)0x00000002)  ///< Флаг готовности внутреннего генератора (HSI) к работе после включения
#define  RCC_CR_HSITRIM          ((uint32_t)0x000000F8)  ///< Пользовательское калибровочное значение внутреннего генератора (HSI)
#define  RCC_CR_HSICAL           ((uint32_t)0x0000FF00)  ///< Заводское калибровочное значение внутреннего генератора (HSI)
#define  RCC_CR_HSEON            ((uint32_t)0x00010000)  ///< Включение внешнего генератора (HSE)
#define  RCC_CR_HSERDY           ((uint32_t)0x00020000)  ///< Флаг готовности внешнего генератора (HSE) к работе после включения
#define  RCC_CR_HSEBYP           ((uint32_t)0x00040000)  ///< Источник тактовых сигналов для внешнего генератора (HSE)
#define  RCC_CR_CSSON            ((uint32_t)0x00080000)  ///< Включение системы контроля (CSS) внешнего генератора (HSE)
#define  RCC_CR_PLLON            ((uint32_t)0x01000000)  ///< Включение внутреннего умножителя частоты (PLL)
#define  RCC_CR_PLLRDY           ((uint32_t)0x02000000)  ///< Флаг готовности внутреннего умножителя частоты (PLL) к работе после включения

/* Bit definition for RCC_CFGR register
 ************************************************************************************************/
 
// SW configuration
#define  RCC_CFGR_SW             ((uint32_t)0x00000003)  ///< Переключение источника тактовых импульсов (SW[1:0] bits)

#define  RCC_CFGR_SW_HSI         ((uint32_t)0x00000000)  ///< Источник тактовых импульсов SYSCLK = HSI
#define  RCC_CFGR_SW_HSE         ((uint32_t)0x00000001)  ///< Источник тактовых импульсов SYSCLK = HSE
#define  RCC_CFGR_SW_PLL         ((uint32_t)0x00000002)  ///< Источник тактовых импульсов SYSCLK = PLL

// SWS configuration
#define  RCC_CFGR_SWS            ((uint32_t)0x0000000C)  ///< Флаги используемого генератора (SWS[1:0] bits)

#define  RCC_CFGR_SWS_HSI        ((uint32_t)0x00000000)  ///< Используется генератор HSI
#define  RCC_CFGR_SWS_HSE        ((uint32_t)0x00000004)  ///< Используется генератор HSE
#define  RCC_CFGR_SWS_PLL        ((uint32_t)0x00000008)  ///< Тактирование от блока  PLL

// HPRE configuration
#define  RCC_CFGR_HPRE           ((uint32_t)0x000000F0)  ///< Коэфициент деления частоты SYSCLK для шины AHB (HPRE[3:0] bits)

#define  RCC_CFGR_HPRE_DIV1      ((uint32_t)0x00000000)  ///< HCLK = SYSCLK / 1
#define  RCC_CFGR_HPRE_DIV2      ((uint32_t)0x00000080)  ///< HCLK = SYSCLK / 2
#define  RCC_CFGR_HPRE_DIV4      ((uint32_t)0x00000090)  ///< HCLK = SYSCLK / 4
#define  RCC_CFGR_HPRE_DIV8      ((uint32_t)0x000000A0)  ///< HCLK = SYSCLK / 8
#define  RCC_CFGR_HPRE_DIV16     ((uint32_t)0x000000B0)  ///< HCLK = SYSCLK / 16
#define  RCC_CFGR_HPRE_DIV64     ((uint32_t)0x000000C0)  ///< HCLK = SYSCLK / 64
#define  RCC_CFGR_HPRE_DIV128    ((uint32_t)0x000000D0)  ///< HCLK = SYSCLK / 128
#define  RCC_CFGR_HPRE_DIV256    ((uint32_t)0x000000E0)  ///< HCLK = SYSCLK / 256
#define  RCC_CFGR_HPRE_DIV512    ((uint32_t)0x000000F0)  ///< HCLK = SYSCLK / 512

// PPRE1 configuration
#define  RCC_CFGR_PPRE1          ((uint32_t)0x00000700)  ///< Коэфициент деления частоты HCLK для шины APB1. Частота шины не должна превышать 36 MHz (PRE1[2:0] bits)

#define  RCC_CFGR_PPRE1_DIV1     ((uint32_t)0x00000000)  ///< PCLK1 = HCLK
#define  RCC_CFGR_PPRE1_DIV2     ((uint32_t)0x00000400)  ///< PCLK1 = HCLK / 2
#define  RCC_CFGR_PPRE1_DIV4     ((uint32_t)0x00000500)  ///< PCLK1 = HCLK / 4
#define  RCC_CFGR_PPRE1_DIV8     ((uint32_t)0x00000600)  ///< PCLK1 = HCLK / 8
#define  RCC_CFGR_PPRE1_DIV16    ((uint32_t)0x00000700)  ///< PCLK1 = HCLK / 16

// PPRE2 configuration
#define  RCC_CFGR_PPRE2          ((uint32_t)0x00003800)  ///< Коэфициент деления частоты HCLK для шины APB2 (PRE2[2:0] bits)

#define  RCC_CFGR_PPRE2_DIV1     ((uint32_t)0x00000000)  ///< PCLK2 = HCLK
#define  RCC_CFGR_PPRE2_DIV2     ((uint32_t)0x00002000)  ///< PCLK2 = HCLK / 2
#define  RCC_CFGR_PPRE2_DIV4     ((uint32_t)0x00002800)  ///< PCLK2 = HCLK / 4
#define  RCC_CFGR_PPRE2_DIV8     ((uint32_t)0x00003000)  ///< PCLK2 = HCLK / 8
#define  RCC_CFGR_PPRE2_DIV16    ((uint32_t)0x00003800)  ///< PCLK2 = HCLK / 16

// ADCPPRE configuration
#define  RCC_CFGR_ADCPRE         ((uint32_t)0x0000C000)  ///< Коэфициент деления частоты PCLK2 для ADC. Частота не должна превышать 14 MHz (ADCPRE[1:0] bits)

#define  RCC_CFGR_ADCPRE_DIV2    ((uint32_t)0x00000000)  ///< Частота ADC = PCLK2 / 2
#define  RCC_CFGR_ADCPRE_DIV4    ((uint32_t)0x00004000)  ///< Частота ADC = PCLK2 / 4
#define  RCC_CFGR_ADCPRE_DIV6    ((uint32_t)0x00008000)  ///< Частота ADC = PCLK2 / 6
#define  RCC_CFGR_ADCPRE_DIV8    ((uint32_t)0x0000C000)  ///< Частота ADC = PCLK2 / 8

#define  RCC_CFGR_PLLSRC         ((uint32_t)0x00010000)  ///< Выбор входного источника для PLL

#define  RCC_CFGR_PLLXTPRE       ((uint32_t)0x00020000)  ///< Делитель генератора HSE для PLL

// PLLMUL configuration
#define  RCC_CFGR_PLLMULL        ((uint32_t)0x003C0000)  ///< Коэффициент умножения PLL(PLLMUL[3:0] bits)

#define  RCC_CFGR_PLLSRC_HSI_Div2 ((uint32_t)0x00000000) ///< Входной источник для PLL = HSI / 2
#define  RCC_CFGR_PLLSRC_HSE     ((uint32_t)0x00010000)  ///< Входной источник для PLL = HSE

#define  RCC_CFGR_PLLXTPRE_HSE   ((uint32_t)0x00000000)  ///< Делитель HSE для PLL = / 1
#define  RCC_CFGR_PLLXTPRE_HSE_Div2 ((uint32_t)0x00020000) ///< Делитель HSE для PLL = / 2

#define  RCC_CFGR_PLLMULL2       ((uint32_t)0x00000000)  ///< Коэффициент умножения PLL х2
#define  RCC_CFGR_PLLMULL3       ((uint32_t)0x00040000)  ///< Коэффициент умножения PLL х3
#define  RCC_CFGR_PLLMULL4       ((uint32_t)0x00080000)  ///< Коэффициент умножения PLL х4
#define  RCC_CFGR_PLLMULL5       ((uint32_t)0x000C0000)  ///< Коэффициент умножения PLL х5
#define  RCC_CFGR_PLLMULL6       ((uint32_t)0x00100000)  ///< Коэффициент умножения PLL х6
#define  RCC_CFGR_PLLMULL7       ((uint32_t)0x00140000)  ///< Коэффициент умножения PLL х7
#define  RCC_CFGR_PLLMULL8       ((uint32_t)0x00180000)  ///< Коэффициент умножения PLL х8
#define  RCC_CFGR_PLLMULL9       ((uint32_t)0x001C0000)  ///< Коэффициент умножения PLL х9
#define  RCC_CFGR_PLLMULL10      ((uint32_t)0x00200000)  ///< Коэффициент умножения PLL х10
#define  RCC_CFGR_PLLMULL11      ((uint32_t)0x00240000)  ///< Коэффициент умножения PLL х11
#define  RCC_CFGR_PLLMULL12      ((uint32_t)0x00280000)  ///< Коэффициент умножения PLL х12
#define  RCC_CFGR_PLLMULL13      ((uint32_t)0x002C0000)  ///< Коэффициент умножения PLL х13
#define  RCC_CFGR_PLLMULL14      ((uint32_t)0x00300000)  ///< Коэффициент умножения PLL х14
#define  RCC_CFGR_PLLMULL15      ((uint32_t)0x00340000)  ///< Коэффициент умножения PLL х15
#define  RCC_CFGR_PLLMULL16      ((uint32_t)0x00380000)  ///< Коэффициент умножения PLL х16

#define  RCC_CFGR_USBPRE         ((uint32_t)0x00400000)  ///< Делитель частоты PLL для тактирования модуля USB

// MCO configuration
#define  RCC_CFGR_MCO            ((uint32_t)0x07000000)  ///< Microcontroller Clock Output (MCO[2:0] bits)

#define  RCC_CFGR_MCO_NOCLOCK    ((uint32_t)0x00000000)  ///< No clock
#define  RCC_CFGR_MCO_SYSCLK     ((uint32_t)0x04000000)  ///< System clock selected as MCO source
#define  RCC_CFGR_MCO_HSI        ((uint32_t)0x05000000)  ///< HSI clock selected as MCO source
#define  RCC_CFGR_MCO_HSE        ((uint32_t)0x06000000)  ///< HSE clock selected as MCO source
#define  RCC_CFGR_MCO_PLL        ((uint32_t)0x07000000)  ///< PLL clock divided by 2 selected as MCO source

/* Bit definition for RCC_CIR register
 ************************************************************************************************/
#define  RCC_CIR_LSIRDYF         ((uint32_t)0x00000001)  ///< LSI Ready Interrupt flag
#define  RCC_CIR_LSERDYF         ((uint32_t)0x00000002)  ///< LSE Ready Interrupt flag
#define  RCC_CIR_HSIRDYF         ((uint32_t)0x00000004)  ///< HSI Ready Interrupt flag
#define  RCC_CIR_HSERDYF         ((uint32_t)0x00000008)  ///< HSE Ready Interrupt flag
#define  RCC_CIR_PLLRDYF         ((uint32_t)0x00000010)  ///< PLL Ready Interrupt flag
#define  RCC_CIR_CSSF            ((uint32_t)0x00000080)  ///< Clock Security System Interrupt flag
#define  RCC_CIR_LSIRDYIE        ((uint32_t)0x00000100)  ///< LSI Ready Interrupt Enable
#define  RCC_CIR_LSERDYIE        ((uint32_t)0x00000200)  ///< LSE Ready Interrupt Enable
#define  RCC_CIR_HSIRDYIE        ((uint32_t)0x00000400)  ///< HSI Ready Interrupt Enable
#define  RCC_CIR_HSERDYIE        ((uint32_t)0x00000800)  ///< HSE Ready Interrupt Enable
#define  RCC_CIR_PLLRDYIE        ((uint32_t)0x00001000)  ///< PLL Ready Interrupt Enable
#define  RCC_CIR_LSIRDYC         ((uint32_t)0x00010000)  ///< LSI Ready Interrupt Clear
#define  RCC_CIR_LSERDYC         ((uint32_t)0x00020000)  ///< LSE Ready Interrupt Clear
#define  RCC_CIR_HSIRDYC         ((uint32_t)0x00040000)  ///< HSI Ready Interrupt Clear
#define  RCC_CIR_HSERDYC         ((uint32_t)0x00080000)  ///< HSE Ready Interrupt Clear
#define  RCC_CIR_PLLRDYC         ((uint32_t)0x00100000)  ///< PLL Ready Interrupt Clear
#define  RCC_CIR_CSSC            ((uint32_t)0x00800000)  ///< Clock Security System Interrupt Clear

/* Bit definition for RCC_APB2RSTR register
 ************************************************************************************************/
#define  RCC_APB2RSTR_AFIORST    ((uint32_t)0x00000001)  ///< Alternate Function I/O reset
#define  RCC_APB2RSTR_IOPARST    ((uint32_t)0x00000004)  ///< I/O port A reset
#define  RCC_APB2RSTR_IOPBRST    ((uint32_t)0x00000008)  ///< I/O port B reset
#define  RCC_APB2RSTR_IOPCRST    ((uint32_t)0x00000010)  ///< I/O port C reset
#define  RCC_APB2RSTR_IOPDRST    ((uint32_t)0x00000020)  ///< I/O port D reset
#define  RCC_APB2RSTR_ADC1RST    ((uint32_t)0x00000200)  ///< ADC 1 interface reset
#define  RCC_APB2RSTR_ADC2RST    ((uint32_t)0x00000400)  ///< ADC 2 interface reset
#define  RCC_APB2RSTR_TIM1RST    ((uint32_t)0x00000800)  ///< TIM1 Timer reset
#define  RCC_APB2RSTR_SPI1RST    ((uint32_t)0x00001000)  ///< SPI 1 reset
#define  RCC_APB2RSTR_USART1RST  ((uint32_t)0x00004000)  ///< USART1 reset
#define  RCC_APB2RSTR_IOPERST    ((uint32_t)0x00000040)  ///< I/O port E reset
#define  RCC_APB2RSTR_IOPFRST    ((uint32_t)0x00000080)  ///< I/O port F reset
#define  RCC_APB2RSTR_IOPGRST    ((uint32_t)0x00000100)  ///< I/O port G reset
#define  RCC_APB2RSTR_TIM8RST    ((uint32_t)0x00002000)  ///< TIM8 Timer reset
#define  RCC_APB2RSTR_ADC3RST    ((uint32_t)0x00008000)  ///< ADC3 interface reset

/* Bit definition for RCC_APB1RSTR register
 ************************************************************************************************/
#define  RCC_APB1RSTR_TIM2RST    ((uint32_t)0x00000001)  ///< Timer 2 reset
#define  RCC_APB1RSTR_TIM3RST    ((uint32_t)0x00000002)  ///< Timer 3 reset
#define  RCC_APB1RSTR_WWDGRST    ((uint32_t)0x00000800)  ///< Window Watchdog reset
#define  RCC_APB1RSTR_USART2RST  ((uint32_t)0x00020000)  ///< USART 2 reset
#define  RCC_APB1RSTR_I2C1RST    ((uint32_t)0x00200000)  ///< I2C 1 reset
#define  RCC_APB1RSTR_CAN1RST    ((uint32_t)0x02000000)  ///< CAN1 reset
#define  RCC_APB1RSTR_BKPRST     ((uint32_t)0x08000000)  ///< Backup interface reset
#define  RCC_APB1RSTR_PWRRST     ((uint32_t)0x10000000)  ///< Power interface reset
#define  RCC_APB1RSTR_TIM4RST    ((uint32_t)0x00000004)  ///< Timer 4 reset
#define  RCC_APB1RSTR_SPI2RST    ((uint32_t)0x00004000)  ///< SPI 2 reset
#define  RCC_APB1RSTR_USART3RST  ((uint32_t)0x00040000)  ///< USART 3 reset
#define  RCC_APB1RSTR_I2C2RST    ((uint32_t)0x00400000)  ///< I2C 2 reset
#define  RCC_APB1RSTR_USBRST     ((uint32_t)0x00800000)  ///< USB Device reset
#define  RCC_APB1RSTR_TIM5RST    ((uint32_t)0x00000008)  ///< Timer 5 reset
#define  RCC_APB1RSTR_TIM6RST    ((uint32_t)0x00000010)  ///< Timer 6 reset
#define  RCC_APB1RSTR_TIM7RST    ((uint32_t)0x00000020)  ///< Timer 7 reset
#define  RCC_APB1RSTR_SPI3RST    ((uint32_t)0x00008000)  ///< SPI 3 reset
#define  RCC_APB1RSTR_UART4RST   ((uint32_t)0x00080000)  ///< UART 4 reset
#define  RCC_APB1RSTR_UART5RST   ((uint32_t)0x00100000)  ///< UART 5 reset
#define  RCC_APB1RSTR_DACRST     ((uint32_t)0x20000000)  ///< DAC interface reset

/* Bit definition for RCC_AHBENR register
 ************************************************************************************************/
#define  RCC_AHBENR_DMA1EN       ((uint16_t)0x0001)      ///< DMA1 clock enable
#define  RCC_AHBENR_SRAMEN       ((uint16_t)0x0004)      ///< SRAM interface clock enable
#define  RCC_AHBENR_FLITFEN      ((uint16_t)0x0010)      ///< FLITF clock enable
#define  RCC_AHBENR_CRCEN        ((uint16_t)0x0040)      ///< CRC clock enable
#define  RCC_AHBENR_DMA2EN       ((uint16_t)0x0002)      ///< DMA2 clock enable
#define  RCC_AHBENR_FSMCEN       ((uint16_t)0x0100)      ///< FSMC clock enable
#define  RCC_AHBENR_SDIOEN       ((uint16_t)0x0400)      ///< SDIO clock enable

/* Bit definition for RCC_APB2ENR register
 ************************************************************************************************/
#define  RCC_APB2ENR_AFIOEN      ((uint32_t)0x00000001)  ///< Alternate Function I/O clock enable
#define  RCC_APB2ENR_IOPAEN      ((uint32_t)0x00000004)  ///< I/O port A clock enable
#define  RCC_APB2ENR_IOPBEN      ((uint32_t)0x00000008)  ///< I/O port B clock enable
#define  RCC_APB2ENR_IOPCEN      ((uint32_t)0x00000010)  ///< I/O port C clock enable
#define  RCC_APB2ENR_IOPDEN      ((uint32_t)0x00000020)  ///< I/O port D clock enable
#define  RCC_APB2ENR_ADC1EN      ((uint32_t)0x00000200)  ///< ADC 1 interface clock enable
#define  RCC_APB2ENR_ADC2EN      ((uint32_t)0x00000400)  ///< ADC 2 interface clock enable
#define  RCC_APB2ENR_TIM1EN      ((uint32_t)0x00000800)  ///< TIM1 Timer clock enable
#define  RCC_APB2ENR_SPI1EN      ((uint32_t)0x00001000)  ///< SPI 1 clock enable
#define  RCC_APB2ENR_USART1EN    ((uint32_t)0x00004000)  ///< USART1 clock enable
#define  RCC_APB2ENR_IOPEEN      ((uint32_t)0x00000040)  ///< I/O port E clock enable
#define  RCC_APB2ENR_IOPFEN      ((uint32_t)0x00000080)  ///< I/O port F clock enable
#define  RCC_APB2ENR_IOPGEN      ((uint32_t)0x00000100)  ///< I/O port G clock enable
#define  RCC_APB2ENR_TIM8EN      ((uint32_t)0x00002000)  ///< TIM8 Timer clock enable
#define  RCC_APB2ENR_ADC3EN      ((uint32_t)0x00008000)  ///< DMA1 clock enable

/* Bit definition for RCC_APB1ENR register
 ************************************************************************************************/
#define  RCC_APB1ENR_TIM2EN      ((uint32_t)0x00000001)  ///< Timer 2 clock enabled*/
#define  RCC_APB1ENR_TIM3EN      ((uint32_t)0x00000002)  ///< Timer 3 clock enable
#define  RCC_APB1ENR_WWDGEN      ((uint32_t)0x00000800)  ///< Window Watchdog clock enable
#define  RCC_APB1ENR_USART2EN    ((uint32_t)0x00020000)  ///< USART 2 clock enable
#define  RCC_APB1ENR_I2C1EN      ((uint32_t)0x00200000)  ///< I2C 1 clock enable
#define  RCC_APB1ENR_CAN1EN      ((uint32_t)0x02000000)  ///< CAN1 clock enable
#define  RCC_APB1ENR_BKPEN       ((uint32_t)0x08000000)  ///< Backup interface clock enable
#define  RCC_APB1ENR_PWREN       ((uint32_t)0x10000000)  ///< Power interface clock enable
#define  RCC_APB1ENR_TIM4EN      ((uint32_t)0x00000004)  ///< Timer 4 clock enable
#define  RCC_APB1ENR_SPI2EN      ((uint32_t)0x00004000)  ///< SPI 2 clock enable
#define  RCC_APB1ENR_USART3EN    ((uint32_t)0x00040000)  ///< USART 3 clock enable
#define  RCC_APB1ENR_I2C2EN      ((uint32_t)0x00400000)  ///< I2C 2 clock enable
#define  RCC_APB1ENR_USBEN       ((uint32_t)0x00800000)  ///< USB Device clock enable
#define  RCC_APB1ENR_TIM5EN      ((uint32_t)0x00000008)  ///< Timer 5 clock enable
#define  RCC_APB1ENR_TIM6EN      ((uint32_t)0x00000010)  ///< Timer 6 clock enable
#define  RCC_APB1ENR_TIM7EN      ((uint32_t)0x00000020)  ///< Timer 7 clock enable
#define  RCC_APB1ENR_SPI3EN      ((uint32_t)0x00008000)  ///< SPI 3 clock enable
#define  RCC_APB1ENR_UART4EN     ((uint32_t)0x00080000)  ///< UART 4 clock enable
#define  RCC_APB1ENR_UART5EN     ((uint32_t)0x00100000)  ///< UART 5 clock enable
#define  RCC_APB1ENR_DACEN       ((uint32_t)0x20000000)  ///< DAC interface clock enable

/* Bit definition for RCC_BDCR register
 ************************************************************************************************/ 
#define  RCC_BDCR_LSEON          ((uint32_t)0x00000001)  ///< External Low Speed oscillator enable
#define  RCC_BDCR_LSERDY         ((uint32_t)0x00000002)  ///< External Low Speed oscillator Ready
#define  RCC_BDCR_LSEBYP         ((uint32_t)0x00000004)  ///< External Low Speed oscillator Bypass

#define  RCC_BDCR_RTCSEL         ((uint32_t)0x00000300)  ///< RTCSEL[1:0] bits (RTC clock source selection)

// RTC congiguration
#define  RCC_BDCR_RTCSEL_NOCLOCK ((uint32_t)0x00000000)  ///< No clock
#define  RCC_BDCR_RTCSEL_LSE     ((uint32_t)0x00000100)  ///< LSE oscillator clock used as RTC clock
#define  RCC_BDCR_RTCSEL_LSI     ((uint32_t)0x00000200)  ///< LSI oscillator clock used as RTC clock
#define  RCC_BDCR_RTCSEL_HSE     ((uint32_t)0x00000300)  ///< HSE oscillator clock divided by 128 used as RTC clock

#define  RCC_BDCR_RTCEN          ((uint32_t)0x00008000)  ///< RTC clock enable
#define  RCC_BDCR_BDRST          ((uint32_t)0x00010000)  ///< Backup domain software reset 

/* Bit definition for RCC_CSR register
 ************************************************************************************************/ 
#define  RCC_CSR_LSION           ((uint32_t)0x00000001)  ///< Internal Low Speed oscillator enable
#define  RCC_CSR_LSIRDY          ((uint32_t)0x00000002)  ///< Internal Low Speed oscillator Ready
#define  RCC_CSR_RMVF            ((uint32_t)0x01000000)  ///< Remove reset flag
#define  RCC_CSR_PINRSTF         ((uint32_t)0x04000000)  ///< PIN reset flag
#define  RCC_CSR_PORRSTF         ((uint32_t)0x08000000)  ///< POR/PDR reset flag
#define  RCC_CSR_SFTRSTF         ((uint32_t)0x10000000)  ///< Software Reset flag
#define  RCC_CSR_IWDGRSTF        ((uint32_t)0x20000000)  ///< Independent Watchdog reset flag
#define  RCC_CSR_WWDGRSTF        ((uint32_t)0x40000000)  ///< Window watchdog reset flag
#define  RCC_CSR_LPWRRSTF        ((uint32_t)0x80000000)  ///< Low-Power reset flag

/************************************************************************************************
 *                                                                                              *
 *                                     Backup registers                                         *
 *                                                                                              *
 ************************************************************************************************/

/* Bit definition for BKP_RTCCR register
 ************************************************************************************************/
#define  BKP_RTCCR_CAL           ((uint16_t)0x007F)      ///< Calibration value
#define  BKP_RTCCR_CCO           ((uint16_t)0x0080)      ///< Calibration Clock Output
#define  BKP_RTCCR_ASOE          ((uint16_t)0x0100)      ///< Alarm or Second Output Enable
#define  BKP_RTCCR_ASOS          ((uint16_t)0x0200)      ///< Alarm or Second Output Selection

/* Bit definition for BKP_CR register
 ************************************************************************************************/
#define  BKP_CR_TPE              ((uint8_t)0x01)         ///< TAMPER pin enable
#define  BKP_CR_TPAL             ((uint8_t)0x02)         ///< TAMPER pin active level

/* Bit definition for BKP_CSR register
 ************************************************************************************************/
#define  BKP_CSR_CTE             ((uint16_t)0x0001)      ///< Clear Tamper event
#define  BKP_CSR_CTI             ((uint16_t)0x0002)      ///< Clear Tamper Interrupt
#define  BKP_CSR_TPIE            ((uint16_t)0x0004)      ///< TAMPER Pin interrupt enable
#define  BKP_CSR_TEF             ((uint16_t)0x0100)      ///< Tamper Event Flag
#define  BKP_CSR_TIF             ((uint16_t)0x0200)      ///< Tamper Interrupt Flag

/* Bit definition for BKP_DRx register
 ************************************************************************************************/
#define  BKP_DR1_D               ((uint16_t)0xFFFF)      ///< Backup data BKP_DR1
#define  BKP_DR2_D               ((uint16_t)0xFFFF)      ///< Backup data BKP_DR2
#define  BKP_DR3_D               ((uint16_t)0xFFFF)      ///< Backup data BKP_DR3
#define  BKP_DR4_D               ((uint16_t)0xFFFF)      ///< Backup data BKP_DR4
#define  BKP_DR5_D               ((uint16_t)0xFFFF)      ///< Backup data BKP_DR5
#define  BKP_DR6_D               ((uint16_t)0xFFFF)      ///< Backup data BKP_DR6
#define  BKP_DR7_D               ((uint16_t)0xFFFF)      ///< Backup data BKP_DR7
#define  BKP_DR8_D               ((uint16_t)0xFFFF)      ///< Backup data BKP_DR8
#define  BKP_DR9_D               ((uint16_t)0xFFFF)      ///< Backup data BKP_DR9
#define  BKP_DR10_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR10
#define  BKP_DR11_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR11
#define  BKP_DR12_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR12
#define  BKP_DR13_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR13
#define  BKP_DR14_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR14
#define  BKP_DR15_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR15
#define  BKP_DR16_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR16
#define  BKP_DR17_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR17
#define  BKP_DR18_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR18
#define  BKP_DR19_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR19
#define  BKP_DR20_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR20
#define  BKP_DR21_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR21
#define  BKP_DR22_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR22
#define  BKP_DR23_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR23
#define  BKP_DR24_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR24
#define  BKP_DR25_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR25
#define  BKP_DR26_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR26
#define  BKP_DR27_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR27
#define  BKP_DR28_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR28
#define  BKP_DR29_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR29
#define  BKP_DR30_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR30
#define  BKP_DR31_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR31
#define  BKP_DR32_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR32
#define  BKP_DR33_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR33
#define  BKP_DR34_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR34
#define  BKP_DR35_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR35
#define  BKP_DR36_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR36
#define  BKP_DR37_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR37
#define  BKP_DR38_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR38
#define  BKP_DR39_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR39
#define  BKP_DR40_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR40
#define  BKP_DR41_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR41
#define  BKP_DR42_D              ((uint16_t)0xFFFF)      ///< Backup data BKP_DR42

/************************************************************************************************
 *                                                                                              *
 *                                     Real-Time Clock                                          *
 *                                                                                              *
 ************************************************************************************************/

/* Bit definition for RTC_CRH register
 ************************************************************************************************/
#define  RTC_CRH_SECIE           ((uint8_t)0x01)         ///< Second Interrupt Enable
#define  RTC_CRH_ALRIE           ((uint8_t)0x02)         ///< Alarm Interrupt Enable
#define  RTC_CRH_OWIE            ((uint8_t)0x04)         ///< OverfloW Interrupt Enable

/* Bit definition for RTC_CRL register
 ************************************************************************************************/
#define  RTC_CRL_SECF            ((uint8_t)0x01)         ///< Second Flag
#define  RTC_CRL_ALRF            ((uint8_t)0x02)         ///< Alarm Flag
#define  RTC_CRL_OWF             ((uint8_t)0x04)         ///< OverfloW Flag
#define  RTC_CRL_RSF             ((uint8_t)0x08)         ///< Registers Synchronized Flag
#define  RTC_CRL_CNF             ((uint8_t)0x10)         ///< Configuration Flag
#define  RTC_CRL_RTOFF           ((uint8_t)0x20)         ///< RTC operation OFF

/* Bit definition for RTC_PRLH register
 ************************************************************************************************/
#define  RTC_PRLH_PRL            ((uint16_t)0x000F)      ///< RTC Prescaler Reload Value High

/* Bit definition for RTC_PRLL register
 ************************************************************************************************/
#define  RTC_PRLL_PRL            ((uint16_t)0xFFFF)      ///< RTC Prescaler Reload Value Low

/* Bit definition for RTC_DIVH register
 ************************************************************************************************/
#define  RTC_DIVH_RTC_DIV        ((uint16_t)0x000F)      ///< RTC Clock Divider High

/* Bit definition for RTC_DIVL register
 ************************************************************************************************/
#define  RTC_DIVL_RTC_DIV        ((uint16_t)0xFFFF)      ///< RTC Clock Divider Low

/* Bit definition for RTC_CNTH register
 ************************************************************************************************/
#define  RTC_CNTH_RTC_CNT        ((uint16_t)0xFFFF)      ///< RTC Counter High

/* Bit definition for RTC_CNTL register
 ************************************************************************************************/
#define  RTC_CNTL_RTC_CNT        ((uint16_t)0xFFFF)      ///< RTC Counter Low

/* Bit definition for RTC_ALRH register
 ************************************************************************************************/
#define  RTC_ALRH_RTC_ALR        ((uint16_t)0xFFFF)      ///< RTC Alarm High

/* Bit definition for RTC_ALRL register
 ************************************************************************************************/
#define  RTC_ALRL_RTC_ALR        ((uint16_t)0xFFFF)      ///< RTC Alarm Low

/************************************************************************************************
 *                                                                                              *
 *                         General Purpose and Alternate Function I/O                           *
 *                                                                                              *
 ************************************************************************************************/
 
/* Bit definition for GPIO_CRL register
 ************************************************************************************************/
#define  GPIO_CRL_MODE           ((uint32_t)0x33333333)  ///< Port x mode bits
#define  GPIO_CRL_MODE0          ((uint32_t)0x00000003)  ///< MODE0[1:0] bits (Port x mode bits, pin 0)
#define  GPIO_CRL_MODE1          ((uint32_t)0x00000030)  ///< MODE1[1:0] bits (Port x mode bits, pin 1)
#define  GPIO_CRL_MODE2          ((uint32_t)0x00000300)  ///< MODE2[1:0] bits (Port x mode bits, pin 2)
#define  GPIO_CRL_MODE3          ((uint32_t)0x00003000)  ///< MODE3[1:0] bits (Port x mode bits, pin 3)
#define  GPIO_CRL_MODE4          ((uint32_t)0x00030000)  ///< MODE4[1:0] bits (Port x mode bits, pin 4)
#define  GPIO_CRL_MODE5          ((uint32_t)0x00300000)  ///< MODE5[1:0] bits (Port x mode bits, pin 5)
#define  GPIO_CRL_MODE6          ((uint32_t)0x03000000)  ///< MODE6[1:0] bits (Port x mode bits, pin 6)
#define  GPIO_CRL_MODE7          ((uint32_t)0x30000000)  ///< MODE7[1:0] bits (Port x mode bits, pin 7)

#define  GPIO_CRL_CNF            ((uint32_t)0xCCCCCCCC)  ///< Port x configuration bits
#define  GPIO_CRL_CNF0           ((uint32_t)0x0000000C)  ///< CNF0[1:0] bits (Port x configuration bits, pin 0)
#define  GPIO_CRL_CNF1           ((uint32_t)0x000000C0)  ///< CNF1[1:0] bits (Port x configuration bits, pin 1)
#define  GPIO_CRL_CNF2           ((uint32_t)0x00000C00)  ///< CNF2[1:0] bits (Port x configuration bits, pin 2)
#define  GPIO_CRL_CNF3           ((uint32_t)0x0000C000)  ///< CNF3[1:0] bits (Port x configuration bits, pin 3)
#define  GPIO_CRL_CNF4           ((uint32_t)0x000C0000)  ///< CNF4[1:0] bits (Port x configuration bits, pin 4)
#define  GPIO_CRL_CNF5           ((uint32_t)0x00C00000)  ///< CNF5[1:0] bits (Port x configuration bits, pin 5)
#define  GPIO_CRL_CNF6           ((uint32_t)0x0C000000)  ///< CNF6[1:0] bits (Port x configuration bits, pin 6)
#define  GPIO_CRL_CNF7           ((uint32_t)0xC0000000)  ///< CNF7[1:0] bits (Port x configuration bits, pin 7)

/* Bit definition for GPIO_CRH register
 ************************************************************************************************/
#define  GPIO_CRH_MODE           ((uint32_t)0x33333333)  ///< Port x mode bits
#define  GPIO_CRH_MODE8          ((uint32_t)0x00000003)  ///< MODE8[1:0] bits (Port x mode bits, pin 8)
#define  GPIO_CRH_MODE9          ((uint32_t)0x00000030)  ///< MODE9[1:0] bits (Port x mode bits, pin 9)
#define  GPIO_CRH_MODE10         ((uint32_t)0x00000300)  ///< MODE10[1:0] bits (Port x mode bits, pin 10)
#define  GPIO_CRH_MODE11         ((uint32_t)0x00003000)  ///< MODE11[1:0] bits (Port x mode bits, pin 11)
#define  GPIO_CRH_MODE12         ((uint32_t)0x00030000)  ///< MODE12[1:0] bits (Port x mode bits, pin 12)
#define  GPIO_CRH_MODE13         ((uint32_t)0x00300000)  ///< MODE13[1:0] bits (Port x mode bits, pin 13)
#define  GPIO_CRH_MODE14         ((uint32_t)0x03000000)  ///< MODE14[1:0] bits (Port x mode bits, pin 14)
#define  GPIO_CRH_MODE15         ((uint32_t)0x30000000)  ///< MODE15[1:0] bits (Port x mode bits, pin 15)

#define  GPIO_CRH_CNF            ((uint32_t)0xCCCCCCCC)  ///< Port x configuration bits
#define  GPIO_CRH_CNF8           ((uint32_t)0x0000000C)  ///< CNF8[1:0] bits (Port x configuration bits, pin 8)
#define  GPIO_CRH_CNF9           ((uint32_t)0x000000C0)  ///< CNF9[1:0] bits (Port x configuration bits, pin 9)
#define  GPIO_CRH_CNF10          ((uint32_t)0x00000C00)  ///< CNF10[1:0] bits (Port x configuration bits, pin 10)
#define  GPIO_CRH_CNF11          ((uint32_t)0x0000C000)  ///< CNF11[1:0] bits (Port x configuration bits, pin 11)
#define  GPIO_CRH_CNF12          ((uint32_t)0x000C0000)  ///< CNF12[1:0] bits (Port x configuration bits, pin 12)
#define  GPIO_CRH_CNF13          ((uint32_t)0x00C00000)  ///< CNF13[1:0] bits (Port x configuration bits, pin 13)
#define  GPIO_CRH_CNF14          ((uint32_t)0x0C000000)  ///< CNF14[1:0] bits (Port x configuration bits, pin 14)
#define  GPIO_CRH_CNF15          ((uint32_t)0xC0000000)  ///< CNF15[1:0] bits (Port x configuration bits, pin 15)

// GPIO configuration for GPIO_CRL/GPIO_CRL
#define GPIO_MODE_INPUT_ANALOG                    0x00   ///< вход аналоговый
#define GPIO_MODE_INPUT_FLOATING                  0x04   ///< вход HiZ без подтяжки
#define GPIO_MODE_INPUT_PULL_UP_DOWN              0x08   ///< вход с подтяжкой
#define GPIO_MODE_OUTPUT2_PUSH_PULL               0x02   ///< выход двухтактный 2МГц
#define GPIO_MODE_OUTPUT10_PUSH_PULL              0x01   ///< выход двухтактный 10МГц
#define GPIO_MODE_OUTPUT50_PUSH_PULL              0x03   ///< выход двухтактный 50МГц
#define GPIO_MODE_OUTPUT2_OPEN_DRAIN              0x06   ///< выход открытый коллектор 2МГц
#define GPIO_MODE_OUTPUT10_OPEN_DRAIN             0x05   ///< выход открытый коллектор 10МГц
#define GPIO_MODE_OUTPUT50_OPEN_DRAIN             0x07   ///< выход открытый коллектор 50МГц
#define GPIO_MODE_OUTPUT2_ALT_PUSH_PULL           0x0A   ///< выход альтерн. двухтактный 2МГц
#define GPIO_MODE_OUTPUT10_ALT_PUSH_PULL          0x09   ///< выход альтерн. двухтактный 10МГц
#define GPIO_MODE_OUTPUT50_ALT_PUSH_PULL          0x0B   ///< выход альтерн. двухтактный 50МГц
#define GPIO_MODE_OUTPUT2_ALT_OPEN_DRAIN          0x0E   ///< выход альтерн. открытый коллектор 2МГц
#define GPIO_MODE_OUTPUT10_ALT_OPEN_DRAIN         0x0D   ///< выход альтерн. открытый коллектор 10МГц
#define GPIO_MODE_OUTPUT50_ALT_OPEN_DRAIN         0x0F   ///< выход альтерн. открытый коллектор 50МГц

/* Bit definition for GPIO_IDR register
 ************************************************************************************************/
#define GPIO_IDR_IDR0            ((uint16_t)0x0001)      ///< Входные данные, бит 0
#define GPIO_IDR_IDR1            ((uint16_t)0x0002)      ///< Входные данные, бит 1
#define GPIO_IDR_IDR2            ((uint16_t)0x0004)      ///< Входные данные, бит 2
#define GPIO_IDR_IDR3            ((uint16_t)0x0008)      ///< Входные данные, бит 3
#define GPIO_IDR_IDR4            ((uint16_t)0x0010)      ///< Входные данные, бит 4
#define GPIO_IDR_IDR5            ((uint16_t)0x0020)      ///< Входные данные, бит 5
#define GPIO_IDR_IDR6            ((uint16_t)0x0040)      ///< Входные данные, бит 6
#define GPIO_IDR_IDR7            ((uint16_t)0x0080)      ///< Входные данные, бит 7
#define GPIO_IDR_IDR8            ((uint16_t)0x0100)      ///< Входные данные, бит 8
#define GPIO_IDR_IDR9            ((uint16_t)0x0200)      ///< Входные данные, бит 9
#define GPIO_IDR_IDR10           ((uint16_t)0x0400)      ///< Входные данные, бит 10
#define GPIO_IDR_IDR11           ((uint16_t)0x0800)      ///< Входные данные, бит 11
#define GPIO_IDR_IDR12           ((uint16_t)0x1000)      ///< Входные данные, бит 12
#define GPIO_IDR_IDR13           ((uint16_t)0x2000)      ///< Входные данные, бит 13
#define GPIO_IDR_IDR14           ((uint16_t)0x4000)      ///< Входные данные, бит 14
#define GPIO_IDR_IDR15           ((uint16_t)0x8000)      ///< Входные данные, бит 15

/* Bit definition for GPIO_ODR register
 ************************************************************************************************/
#define GPIO_ODR_ODR0            ((uint16_t)0x0001)      ///< Выходные данные, бит 0
#define GPIO_ODR_ODR1            ((uint16_t)0x0002)      ///< Выходные данные, бит 1
#define GPIO_ODR_ODR2            ((uint16_t)0x0004)      ///< Выходные данные, бит 2
#define GPIO_ODR_ODR3            ((uint16_t)0x0008)      ///< Выходные данные, бит 3
#define GPIO_ODR_ODR4            ((uint16_t)0x0010)      ///< Выходные данные, бит 4
#define GPIO_ODR_ODR5            ((uint16_t)0x0020)      ///< Выходные данные, бит 5
#define GPIO_ODR_ODR6            ((uint16_t)0x0040)      ///< Выходные данные, бит 6
#define GPIO_ODR_ODR7            ((uint16_t)0x0080)      ///< Выходные данные, бит 7
#define GPIO_ODR_ODR8            ((uint16_t)0x0100)      ///< Выходные данные, бит 8
#define GPIO_ODR_ODR9            ((uint16_t)0x0200)      ///< Выходные данные, бит 9
#define GPIO_ODR_ODR10           ((uint16_t)0x0400)      ///< Выходные данные, бит 10
#define GPIO_ODR_ODR11           ((uint16_t)0x0800)      ///< Выходные данные, бит 11
#define GPIO_ODR_ODR12           ((uint16_t)0x1000)      ///< Выходные данные, бит 12
#define GPIO_ODR_ODR13           ((uint16_t)0x2000)      ///< Выходные данные, бит 13
#define GPIO_ODR_ODR14           ((uint16_t)0x4000)      ///< Выходные данные, бит 14
#define GPIO_ODR_ODR15           ((uint16_t)0x8000)      ///< Выходные данные, бит 15

/* Bit definition for GPIO_BSRR register
 ************************************************************************************************/
#define GPIO_BSRR_BS0            ((uint32_t)0x00000001)  ///< Установка выходного бита 0
#define GPIO_BSRR_BS1            ((uint32_t)0x00000002)  ///< Установка выходного бита 1
#define GPIO_BSRR_BS2            ((uint32_t)0x00000004)  ///< Установка выходного бита 2
#define GPIO_BSRR_BS3            ((uint32_t)0x00000008)  ///< Установка выходного бита 3
#define GPIO_BSRR_BS4            ((uint32_t)0x00000010)  ///< Установка выходного бита 4
#define GPIO_BSRR_BS5            ((uint32_t)0x00000020)  ///< Установка выходного бита 5
#define GPIO_BSRR_BS6            ((uint32_t)0x00000040)  ///< Установка выходного бита 6
#define GPIO_BSRR_BS7            ((uint32_t)0x00000080)  ///< Установка выходного бита 7
#define GPIO_BSRR_BS8            ((uint32_t)0x00000100)  ///< Установка выходного бита 8
#define GPIO_BSRR_BS9            ((uint32_t)0x00000200)  ///< Установка выходного бита 9
#define GPIO_BSRR_BS10           ((uint32_t)0x00000400)  ///< Установка выходного бита 10
#define GPIO_BSRR_BS11           ((uint32_t)0x00000800)  ///< Установка выходного бита 11
#define GPIO_BSRR_BS12           ((uint32_t)0x00001000)  ///< Установка выходного бита 12
#define GPIO_BSRR_BS13           ((uint32_t)0x00002000)  ///< Установка выходного бита 13
#define GPIO_BSRR_BS14           ((uint32_t)0x00004000)  ///< Установка выходного бита 14
#define GPIO_BSRR_BS15           ((uint32_t)0x00008000)  ///< Установка выходного бита 15
#define GPIO_BSRR_BR0            ((uint32_t)0x00010000)  ///< Сброс выходного бита 0
#define GPIO_BSRR_BR1            ((uint32_t)0x00020000)  ///< Сброс выходного бита 1
#define GPIO_BSRR_BR2            ((uint32_t)0x00040000)  ///< Сброс выходного бита 2
#define GPIO_BSRR_BR3            ((uint32_t)0x00080000)  ///< Сброс выходного бита 3
#define GPIO_BSRR_BR4            ((uint32_t)0x00100000)  ///< Сброс выходного бита 4
#define GPIO_BSRR_BR5            ((uint32_t)0x00200000)  ///< Сброс выходного бита 5
#define GPIO_BSRR_BR6            ((uint32_t)0x00400000)  ///< Сброс выходного бита 6
#define GPIO_BSRR_BR7            ((uint32_t)0x00800000)  ///< Сброс выходного бита 7
#define GPIO_BSRR_BR8            ((uint32_t)0x01000000)  ///< Сброс выходного бита 8
#define GPIO_BSRR_BR9            ((uint32_t)0x02000000)  ///< Сброс выходного бита 9
#define GPIO_BSRR_BR10           ((uint32_t)0x04000000)  ///< Сброс выходного бита 10
#define GPIO_BSRR_BR11           ((uint32_t)0x08000000)  ///< Сброс выходного бита 11
#define GPIO_BSRR_BR12           ((uint32_t)0x10000000)  ///< Сброс выходного бита 12
#define GPIO_BSRR_BR13           ((uint32_t)0x20000000)  ///< Сброс выходного бита 13
#define GPIO_BSRR_BR14           ((uint32_t)0x40000000)  ///< Сброс выходного бита 14
#define GPIO_BSRR_BR15           ((uint32_t)0x80000000)  ///< Сброс выходного бита 15

/* Bit definition for GPIO_BRR registe
 ************************************************************************************************/
#define GPIO_BRR_BR0             ((uint16_t)0x0001)      ///< Сброс выходного бита 0
#define GPIO_BRR_BR1             ((uint16_t)0x0002)      ///< Сброс выходного бита 1
#define GPIO_BRR_BR2             ((uint16_t)0x0004)      ///< Сброс выходного бита 2
#define GPIO_BRR_BR3             ((uint16_t)0x0008)      ///< Сброс выходного бита 3
#define GPIO_BRR_BR4             ((uint16_t)0x0010)      ///< Сброс выходного бита 4
#define GPIO_BRR_BR5             ((uint16_t)0x0020)      ///< Сброс выходного бита 5
#define GPIO_BRR_BR6             ((uint16_t)0x0040)      ///< Сброс выходного бита 6
#define GPIO_BRR_BR7             ((uint16_t)0x0080)      ///< Сброс выходного бита 7
#define GPIO_BRR_BR8             ((uint16_t)0x0100)      ///< Сброс выходного бита 8
#define GPIO_BRR_BR9             ((uint16_t)0x0200)      ///< Сброс выходного бита 9
#define GPIO_BRR_BR10            ((uint16_t)0x0400)      ///< Сброс выходного бита 10
#define GPIO_BRR_BR11            ((uint16_t)0x0800)      ///< Сброс выходного бита 11
#define GPIO_BRR_BR12            ((uint16_t)0x1000)      ///< Сброс выходного бита 12
#define GPIO_BRR_BR13            ((uint16_t)0x2000)      ///< Сброс выходного бита 13
#define GPIO_BRR_BR14            ((uint16_t)0x4000)      ///< Сброс выходного бита 14
#define GPIO_BRR_BR15            ((uint16_t)0x8000)      ///< Сброс выходного бита 15

/* Bit definition for GPIO_LCKR register
 ************************************************************************************************/
#define GPIO_LCKR_LCK0           ((uint32_t)0x00000001)  ///< Бит блокировки пина 0
#define GPIO_LCKR_LCK1           ((uint32_t)0x00000002)  ///< Бит блокировки пина 1
#define GPIO_LCKR_LCK2           ((uint32_t)0x00000004)  ///< Бит блокировки пина 2
#define GPIO_LCKR_LCK3           ((uint32_t)0x00000008)  ///< Бит блокировки пина 3
#define GPIO_LCKR_LCK4           ((uint32_t)0x00000010)  ///< Бит блокировки пина 4
#define GPIO_LCKR_LCK5           ((uint32_t)0x00000020)  ///< Бит блокировки пина 5
#define GPIO_LCKR_LCK6           ((uint32_t)0x00000040)  ///< Бит блокировки пина 6
#define GPIO_LCKR_LCK7           ((uint32_t)0x00000080)  ///< Бит блокировки пина 7
#define GPIO_LCKR_LCK8           ((uint32_t)0x00000100)  ///< Бит блокировки пина 8
#define GPIO_LCKR_LCK9           ((uint32_t)0x00000200)  ///< Бит блокировки пина 9
#define GPIO_LCKR_LCK10          ((uint32_t)0x00000400)  ///< Бит блокировки пина 10
#define GPIO_LCKR_LCK11          ((uint32_t)0x00000800)  ///< Бит блокировки пина 11
#define GPIO_LCKR_LCK12          ((uint32_t)0x00001000)  ///< Бит блокировки пина 12
#define GPIO_LCKR_LCK13          ((uint32_t)0x00002000)  ///< Бит блокировки пина 13
#define GPIO_LCKR_LCK14          ((uint32_t)0x00004000)  ///< Бит блокировки пина 14
#define GPIO_LCKR_LCK15          ((uint32_t)0x00008000)  ///< Бит блокировки пина 15
#define GPIO_LCKR_LCKK           ((uint32_t)0x00010000)  ///< Бит ключа блокировки

/******************  Bit definition for AFIO_EVCR register  *******************/
#define AFIO_EVCR_PIN                        ((uint8_t)0x0F)               /* PIN[3:0] bits (Pin selection) */
#define AFIO_EVCR_PIN_0                      ((uint8_t)0x01)               /* Bit 0 */
#define AFIO_EVCR_PIN_1                      ((uint8_t)0x02)               /* Bit 1 */
#define AFIO_EVCR_PIN_2                      ((uint8_t)0x04)               /* Bit 2 */
#define AFIO_EVCR_PIN_3                      ((uint8_t)0x08)               /* Bit 3 */

/* PIN configuration */
#define AFIO_EVCR_PIN_PX0                    ((uint8_t)0x00)               /* Pin 0 selected */
#define AFIO_EVCR_PIN_PX1                    ((uint8_t)0x01)               /* Pin 1 selected */
#define AFIO_EVCR_PIN_PX2                    ((uint8_t)0x02)               /* Pin 2 selected */
#define AFIO_EVCR_PIN_PX3                    ((uint8_t)0x03)               /* Pin 3 selected */
#define AFIO_EVCR_PIN_PX4                    ((uint8_t)0x04)               /* Pin 4 selected */
#define AFIO_EVCR_PIN_PX5                    ((uint8_t)0x05)               /* Pin 5 selected */
#define AFIO_EVCR_PIN_PX6                    ((uint8_t)0x06)               /* Pin 6 selected */
#define AFIO_EVCR_PIN_PX7                    ((uint8_t)0x07)               /* Pin 7 selected */
#define AFIO_EVCR_PIN_PX8                    ((uint8_t)0x08)               /* Pin 8 selected */
#define AFIO_EVCR_PIN_PX9                    ((uint8_t)0x09)               /* Pin 9 selected */
#define AFIO_EVCR_PIN_PX10                   ((uint8_t)0x0A)               /* Pin 10 selected */
#define AFIO_EVCR_PIN_PX11                   ((uint8_t)0x0B)               /* Pin 11 selected */
#define AFIO_EVCR_PIN_PX12                   ((uint8_t)0x0C)               /* Pin 12 selected */
#define AFIO_EVCR_PIN_PX13                   ((uint8_t)0x0D)               /* Pin 13 selected */
#define AFIO_EVCR_PIN_PX14                   ((uint8_t)0x0E)               /* Pin 14 selected */
#define AFIO_EVCR_PIN_PX15                   ((uint8_t)0x0F)               /* Pin 15 selected */

#define AFIO_EVCR_PORT                       ((uint8_t)0x70)               /* PORT[2:0] bits (Port selection) */
#define AFIO_EVCR_PORT_0                     ((uint8_t)0x10)               /* Bit 0 */
#define AFIO_EVCR_PORT_1                     ((uint8_t)0x20)               /* Bit 1 */
#define AFIO_EVCR_PORT_2                     ((uint8_t)0x40)               /* Bit 2 */

/* PORT configuration */
#define AFIO_EVCR_PORT_PA                    ((uint8_t)0x00)               /* Port A selected */
#define AFIO_EVCR_PORT_PB                    ((uint8_t)0x10)               /* Port B selected */
#define AFIO_EVCR_PORT_PC                    ((uint8_t)0x20)               /* Port C selected */
#define AFIO_EVCR_PORT_PD                    ((uint8_t)0x30)               /* Port D selected */
#define AFIO_EVCR_PORT_PE                    ((uint8_t)0x40)               /* Port E selected */

#define AFIO_EVCR_EVOE                       ((uint8_t)0x80)               /* Event Output Enable */

/******************  Bit definition for AFIO_MAPR register  *******************/
#define AFIO_MAPR_SPI1_REMAP                 ((uint32_t)0x00000001)        /* SPI1 remapping */
#define AFIO_MAPR_I2C1_REMAP                 ((uint32_t)0x00000002)        /* I2C1 remapping */
#define AFIO_MAPR_USART1_REMAP               ((uint32_t)0x00000004)        /* USART1 remapping */
#define AFIO_MAPR_USART2_REMAP               ((uint32_t)0x00000008)        /* USART2 remapping */

#define AFIO_MAPR_USART3_REMAP               ((uint32_t)0x00000030)        /* USART3_REMAP[1:0] bits (USART3 remapping) */
#define AFIO_MAPR_USART3_REMAP_0             ((uint32_t)0x00000010)        /* Bit 0 */
#define AFIO_MAPR_USART3_REMAP_1             ((uint32_t)0x00000020)        /* Bit 1 */

/* USART3_REMAP configuration */
#define AFIO_MAPR_USART3_REMAP_NOREMAP       ((uint32_t)0x00000000)        /* No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP  ((uint32_t)0x00000010)        /* Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_FULLREMAP     ((uint32_t)0x00000030)        /* Full remap (TX/PD8, RX/PD9, CK/PD10, CTS/PD11, RTS/PD12) */

#define AFIO_MAPR_TIM1_REMAP                 ((uint32_t)0x000000C0)        /* TIM1_REMAP[1:0] bits (TIM1 remapping) */
#define AFIO_MAPR_TIM1_REMAP_0               ((uint32_t)0x00000040)        /* Bit 0 */
#define AFIO_MAPR_TIM1_REMAP_1               ((uint32_t)0x00000080)        /* Bit 1 */

/* TIM1_REMAP configuration */
#define AFIO_MAPR_TIM1_REMAP_NOREMAP         ((uint32_t)0x00000000)        /* No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15) */
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP    ((uint32_t)0x00000040)        /* Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1) */
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP       ((uint32_t)0x000000C0)        /* Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12) */

#define AFIO_MAPR_TIM2_REMAP                 ((uint32_t)0x00000300)        /* TIM2_REMAP[1:0] bits (TIM2 remapping) */
#define AFIO_MAPR_TIM2_REMAP_0               ((uint32_t)0x00000100)        /* Bit 0 */
#define AFIO_MAPR_TIM2_REMAP_1               ((uint32_t)0x00000200)        /* Bit 1 */

/* TIM2_REMAP configuration */
#define AFIO_MAPR_TIM2_REMAP_NOREMAP         ((uint32_t)0x00000000)        /* No remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1   ((uint32_t)0x00000100)        /* Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2   ((uint32_t)0x00000200)        /* Partial remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11) */
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP       ((uint32_t)0x00000300)        /* Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11) */

#define AFIO_MAPR_TIM3_REMAP                 ((uint32_t)0x00000C00)        /* TIM3_REMAP[1:0] bits (TIM3 remapping) */
#define AFIO_MAPR_TIM3_REMAP_0               ((uint32_t)0x00000400)        /* Bit 0 */
#define AFIO_MAPR_TIM3_REMAP_1               ((uint32_t)0x00000800)        /* Bit 1 */

/* TIM3_REMAP configuration */
#define AFIO_MAPR_TIM3_REMAP_NOREMAP         ((uint32_t)0x00000000)        /* No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP    ((uint32_t)0x00000800)        /* Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP       ((uint32_t)0x00000C00)        /* Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9) */

#define AFIO_MAPR_TIM4_REMAP                 ((uint32_t)0x00001000)        /* TIM4_REMAP bit (TIM4 remapping) */

#define AFIO_MAPR_CAN_REMAP                  ((uint32_t)0x00006000)        /* CAN_REMAP[1:0] bits (CAN Alternate function remapping) */
#define AFIO_MAPR_CAN_REMAP_0                ((uint32_t)0x00002000)        /* Bit 0 */
#define AFIO_MAPR_CAN_REMAP_1                ((uint32_t)0x00004000)        /* Bit 1 */

/* CAN_REMAP configuration */
#define AFIO_MAPR_CAN_REMAP_REMAP1           ((uint32_t)0x00000000)        /* CANRX mapped to PA11, CANTX mapped to PA12 */
#define AFIO_MAPR_CAN_REMAP_REMAP2           ((uint32_t)0x00004000)        /* CANRX mapped to PB8, CANTX mapped to PB9 */
#define AFIO_MAPR_CAN_REMAP_REMAP3           ((uint32_t)0x00006000)        /* CANRX mapped to PD0, CANTX mapped to PD1 */

#define AFIO_MAPR_PD01_REMAP                 ((uint32_t)0x00008000)        /* Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
#define AFIO_MAPR_TIM5CH4_IREMAP             ((uint32_t)0x00010000)        /* TIM5 Channel4 Internal Remap */
#define AFIO_MAPR_ADC1_ETRGINJ_REMAP         ((uint32_t)0x00020000)        /* ADC 1 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC1_ETRGREG_REMAP         ((uint32_t)0x00040000)        /* ADC 1 External Trigger Regular Conversion remapping */
#define AFIO_MAPR_ADC2_ETRGINJ_REMAP         ((uint32_t)0x00080000)        /* ADC 2 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC2_ETRGREG_REMAP         ((uint32_t)0x00100000)        /* ADC 2 External Trigger Regular Conversion remapping */

/* SWJ_CFG configuration */
#define AFIO_MAPR_SWJ_CFG                    ((uint32_t)0x07000000)        /* SWJ_CFG[2:0] bits (Serial Wire JTAG configuration) */
#define AFIO_MAPR_SWJ_CFG_0                  ((uint32_t)0x01000000)        /* Bit 0 */
#define AFIO_MAPR_SWJ_CFG_1                  ((uint32_t)0x02000000)        /* Bit 1 */
#define AFIO_MAPR_SWJ_CFG_2                  ((uint32_t)0x04000000)        /* Bit 2 */

#define AFIO_MAPR_SWJ_CFG_RESET              ((uint32_t)0x00000000)        /* Full SWJ (JTAG-DP + SW-DP) : Reset State */
#define AFIO_MAPR_SWJ_CFG_NOJNTRST           ((uint32_t)0x01000000)        /* Full SWJ (JTAG-DP + SW-DP) but without JNTRST */
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE        ((uint32_t)0x02000000)        /* JTAG-DP Disabled and SW-DP Enabled */
#define AFIO_MAPR_SWJ_CFG_DISABLE            ((uint32_t)0x04000000)        /* JTAG-DP Disabled and SW-DP Disabled */

/*****************  Bit definition for AFIO_EXTICR1 register  *****************/
#define AFIO_EXTICR1_EXTI0                   ((uint16_t)0x000F)            /* EXTI 0 configuration */
#define AFIO_EXTICR1_EXTI1                   ((uint16_t)0x00F0)            /* EXTI 1 configuration */
#define AFIO_EXTICR1_EXTI2                   ((uint16_t)0x0F00)            /* EXTI 2 configuration */
#define AFIO_EXTICR1_EXTI3                   ((uint16_t)0xF000)            /* EXTI 3 configuration */

/* EXTI0 configuration */
#define AFIO_EXTICR1_EXTI0_PA                ((uint16_t)0x0000)            /* PA[0] pin */
#define AFIO_EXTICR1_EXTI0_PB                ((uint16_t)0x0001)            /* PB[0] pin */
#define AFIO_EXTICR1_EXTI0_PC                ((uint16_t)0x0002)            /* PC[0] pin */
#define AFIO_EXTICR1_EXTI0_PD                ((uint16_t)0x0003)            /* PD[0] pin */
#define AFIO_EXTICR1_EXTI0_PE                ((uint16_t)0x0004)            /* PE[0] pin */
#define AFIO_EXTICR1_EXTI0_PF                ((uint16_t)0x0005)            /* PF[0] pin */
#define AFIO_EXTICR1_EXTI0_PG                ((uint16_t)0x0006)            /* PG[0] pin */

/* EXTI1 configuration */
#define AFIO_EXTICR1_EXTI1_PA                ((uint16_t)0x0000)            /* PA[1] pin */
#define AFIO_EXTICR1_EXTI1_PB                ((uint16_t)0x0010)            /* PB[1] pin */
#define AFIO_EXTICR1_EXTI1_PC                ((uint16_t)0x0020)            /* PC[1] pin */
#define AFIO_EXTICR1_EXTI1_PD                ((uint16_t)0x0030)            /* PD[1] pin */
#define AFIO_EXTICR1_EXTI1_PE                ((uint16_t)0x0040)            /* PE[1] pin */
#define AFIO_EXTICR1_EXTI1_PF                ((uint16_t)0x0050)            /* PF[1] pin */
#define AFIO_EXTICR1_EXTI1_PG                ((uint16_t)0x0060)            /* PG[1] pin */

/* EXTI2 configuration */  
#define AFIO_EXTICR1_EXTI2_PA                ((uint16_t)0x0000)            /* PA[2] pin */
#define AFIO_EXTICR1_EXTI2_PB                ((uint16_t)0x0100)            /* PB[2] pin */
#define AFIO_EXTICR1_EXTI2_PC                ((uint16_t)0x0200)            /* PC[2] pin */
#define AFIO_EXTICR1_EXTI2_PD                ((uint16_t)0x0300)            /* PD[2] pin */
#define AFIO_EXTICR1_EXTI2_PE                ((uint16_t)0x0400)            /* PE[2] pin */
#define AFIO_EXTICR1_EXTI2_PF                ((uint16_t)0x0500)            /* PF[2] pin */
#define AFIO_EXTICR1_EXTI2_PG                ((uint16_t)0x0600)            /* PG[2] pin */

/* EXTI3 configuration */
#define AFIO_EXTICR1_EXTI3_PA                ((uint16_t)0x0000)            /* PA[3] pin */
#define AFIO_EXTICR1_EXTI3_PB                ((uint16_t)0x1000)            /* PB[3] pin */
#define AFIO_EXTICR1_EXTI3_PC                ((uint16_t)0x2000)            /* PC[3] pin */
#define AFIO_EXTICR1_EXTI3_PD                ((uint16_t)0x3000)            /* PD[3] pin */
#define AFIO_EXTICR1_EXTI3_PE                ((uint16_t)0x4000)            /* PE[3] pin */
#define AFIO_EXTICR1_EXTI3_PF                ((uint16_t)0x5000)            /* PF[3] pin */
#define AFIO_EXTICR1_EXTI3_PG                ((uint16_t)0x6000)            /* PG[3] pin */

/*****************  Bit definition for AFIO_EXTICR2 register  *****************/
#define AFIO_EXTICR2_EXTI4                   ((uint16_t)0x000F)            /* EXTI 4 configuration */
#define AFIO_EXTICR2_EXTI5                   ((uint16_t)0x00F0)            /* EXTI 5 configuration */
#define AFIO_EXTICR2_EXTI6                   ((uint16_t)0x0F00)            /* EXTI 6 configuration */
#define AFIO_EXTICR2_EXTI7                   ((uint16_t)0xF000)            /* EXTI 7 configuration */

/* EXTI4 configuration */
#define AFIO_EXTICR2_EXTI4_PA                ((uint16_t)0x0000)            /* PA[4] pin */
#define AFIO_EXTICR2_EXTI4_PB                ((uint16_t)0x0001)            /* PB[4] pin */
#define AFIO_EXTICR2_EXTI4_PC                ((uint16_t)0x0002)            /* PC[4] pin */
#define AFIO_EXTICR2_EXTI4_PD                ((uint16_t)0x0003)            /* PD[4] pin */
#define AFIO_EXTICR2_EXTI4_PE                ((uint16_t)0x0004)            /* PE[4] pin */
#define AFIO_EXTICR2_EXTI4_PF                ((uint16_t)0x0005)            /* PF[4] pin */
#define AFIO_EXTICR2_EXTI4_PG                ((uint16_t)0x0006)            /* PG[4] pin */

/* EXTI5 configuration */
#define AFIO_EXTICR2_EXTI5_PA                ((uint16_t)0x0000)            /* PA[5] pin */
#define AFIO_EXTICR2_EXTI5_PB                ((uint16_t)0x0010)            /* PB[5] pin */
#define AFIO_EXTICR2_EXTI5_PC                ((uint16_t)0x0020)            /* PC[5] pin */
#define AFIO_EXTICR2_EXTI5_PD                ((uint16_t)0x0030)            /* PD[5] pin */
#define AFIO_EXTICR2_EXTI5_PE                ((uint16_t)0x0040)            /* PE[5] pin */
#define AFIO_EXTICR2_EXTI5_PF                ((uint16_t)0x0050)            /* PF[5] pin */
#define AFIO_EXTICR2_EXTI5_PG                ((uint16_t)0x0060)            /* PG[5] pin */

/* EXTI6 configuration */  
#define AFIO_EXTICR2_EXTI6_PA                ((uint16_t)0x0000)            /* PA[6] pin */
#define AFIO_EXTICR2_EXTI6_PB                ((uint16_t)0x0100)            /* PB[6] pin */
#define AFIO_EXTICR2_EXTI6_PC                ((uint16_t)0x0200)            /* PC[6] pin */
#define AFIO_EXTICR2_EXTI6_PD                ((uint16_t)0x0300)            /* PD[6] pin */
#define AFIO_EXTICR2_EXTI6_PE                ((uint16_t)0x0400)            /* PE[6] pin */
#define AFIO_EXTICR2_EXTI6_PF                ((uint16_t)0x0500)            /* PF[6] pin */
#define AFIO_EXTICR2_EXTI6_PG                ((uint16_t)0x0600)            /* PG[6] pin */

/* EXTI7 configuration */
#define AFIO_EXTICR2_EXTI7_PA                ((uint16_t)0x0000)            /* PA[7] pin */
#define AFIO_EXTICR2_EXTI7_PB                ((uint16_t)0x1000)            /* PB[7] pin */
#define AFIO_EXTICR2_EXTI7_PC                ((uint16_t)0x2000)            /* PC[7] pin */
#define AFIO_EXTICR2_EXTI7_PD                ((uint16_t)0x3000)            /* PD[7] pin */
#define AFIO_EXTICR2_EXTI7_PE                ((uint16_t)0x4000)            /* PE[7] pin */
#define AFIO_EXTICR2_EXTI7_PF                ((uint16_t)0x5000)            /* PF[7] pin */
#define AFIO_EXTICR2_EXTI7_PG                ((uint16_t)0x6000)            /* PG[7] pin */

/*****************  Bit definition for AFIO_EXTICR3 register  *****************/
#define AFIO_EXTICR3_EXTI8                   ((uint16_t)0x000F)            /* EXTI 8 configuration */
#define AFIO_EXTICR3_EXTI9                   ((uint16_t)0x00F0)            /* EXTI 9 configuration */
#define AFIO_EXTICR3_EXTI10                  ((uint16_t)0x0F00)            /* EXTI 10 configuration */
#define AFIO_EXTICR3_EXTI11                  ((uint16_t)0xF000)            /* EXTI 11 configuration */

/* EXTI8 configuration */
#define AFIO_EXTICR3_EXTI8_PA                ((uint16_t)0x0000)            /* PA[8] pin */
#define AFIO_EXTICR3_EXTI8_PB                ((uint16_t)0x0001)            /* PB[8] pin */
#define AFIO_EXTICR3_EXTI8_PC                ((uint16_t)0x0002)            /* PC[8] pin */
#define AFIO_EXTICR3_EXTI8_PD                ((uint16_t)0x0003)            /* PD[8] pin */
#define AFIO_EXTICR3_EXTI8_PE                ((uint16_t)0x0004)            /* PE[8] pin */
#define AFIO_EXTICR3_EXTI8_PF                ((uint16_t)0x0005)            /* PF[8] pin */
#define AFIO_EXTICR3_EXTI8_PG                ((uint16_t)0x0006)            /* PG[8] pin */

/* EXTI9 configuration */
#define AFIO_EXTICR3_EXTI9_PA                ((uint16_t)0x0000)            /* PA[9] pin */
#define AFIO_EXTICR3_EXTI9_PB                ((uint16_t)0x0010)            /* PB[9] pin */
#define AFIO_EXTICR3_EXTI9_PC                ((uint16_t)0x0020)            /* PC[9] pin */
#define AFIO_EXTICR3_EXTI9_PD                ((uint16_t)0x0030)            /* PD[9] pin */
#define AFIO_EXTICR3_EXTI9_PE                ((uint16_t)0x0040)            /* PE[9] pin */
#define AFIO_EXTICR3_EXTI9_PF                ((uint16_t)0x0050)            /* PF[9] pin */
#define AFIO_EXTICR3_EXTI9_PG                ((uint16_t)0x0060)            /* PG[9] pin */

/* EXTI10 configuration */  
#define AFIO_EXTICR3_EXTI10_PA               ((uint16_t)0x0000)            /* PA[10] pin */
#define AFIO_EXTICR3_EXTI10_PB               ((uint16_t)0x0100)            /* PB[10] pin */
#define AFIO_EXTICR3_EXTI10_PC               ((uint16_t)0x0200)            /* PC[10] pin */
#define AFIO_EXTICR3_EXTI10_PD               ((uint16_t)0x0300)            /* PD[10] pin */
#define AFIO_EXTICR3_EXTI10_PE               ((uint16_t)0x0400)            /* PE[10] pin */
#define AFIO_EXTICR3_EXTI10_PF               ((uint16_t)0x0500)            /* PF[10] pin */
#define AFIO_EXTICR3_EXTI10_PG               ((uint16_t)0x0600)            /* PG[10] pin */

/* EXTI11 configuration */
#define AFIO_EXTICR3_EXTI11_PA               ((uint16_t)0x0000)            /* PA[11] pin */
#define AFIO_EXTICR3_EXTI11_PB               ((uint16_t)0x1000)            /* PB[11] pin */
#define AFIO_EXTICR3_EXTI11_PC               ((uint16_t)0x2000)            /* PC[11] pin */
#define AFIO_EXTICR3_EXTI11_PD               ((uint16_t)0x3000)            /* PD[11] pin */
#define AFIO_EXTICR3_EXTI11_PE               ((uint16_t)0x4000)            /* PE[11] pin */
#define AFIO_EXTICR3_EXTI11_PF               ((uint16_t)0x5000)            /* PF[11] pin */
#define AFIO_EXTICR3_EXTI11_PG               ((uint16_t)0x6000)            /* PG[11] pin */

/*****************  Bit definition for AFIO_EXTICR4 register  *****************/
#define AFIO_EXTICR4_EXTI12                  ((uint16_t)0x000F)            /* EXTI 12 configuration */
#define AFIO_EXTICR4_EXTI13                  ((uint16_t)0x00F0)            /* EXTI 13 configuration */
#define AFIO_EXTICR4_EXTI14                  ((uint16_t)0x0F00)            /* EXTI 14 configuration */
#define AFIO_EXTICR4_EXTI15                  ((uint16_t)0xF000)            /* EXTI 15 configuration */

/* EXTI12 configuration */
#define AFIO_EXTICR4_EXTI12_PA               ((uint16_t)0x0000)            /* PA[12] pin */
#define AFIO_EXTICR4_EXTI12_PB               ((uint16_t)0x0001)            /* PB[12] pin */
#define AFIO_EXTICR4_EXTI12_PC               ((uint16_t)0x0002)            /* PC[12] pin */
#define AFIO_EXTICR4_EXTI12_PD               ((uint16_t)0x0003)            /* PD[12] pin */
#define AFIO_EXTICR4_EXTI12_PE               ((uint16_t)0x0004)            /* PE[12] pin */
#define AFIO_EXTICR4_EXTI12_PF               ((uint16_t)0x0005)            /* PF[12] pin */
#define AFIO_EXTICR4_EXTI12_PG               ((uint16_t)0x0006)            /* PG[12] pin */

/* EXTI13 configuration */
#define AFIO_EXTICR4_EXTI13_PA               ((uint16_t)0x0000)            /* PA[13] pin */
#define AFIO_EXTICR4_EXTI13_PB               ((uint16_t)0x0010)            /* PB[13] pin */
#define AFIO_EXTICR4_EXTI13_PC               ((uint16_t)0x0020)            /* PC[13] pin */
#define AFIO_EXTICR4_EXTI13_PD               ((uint16_t)0x0030)            /* PD[13] pin */
#define AFIO_EXTICR4_EXTI13_PE               ((uint16_t)0x0040)            /* PE[13] pin */
#define AFIO_EXTICR4_EXTI13_PF               ((uint16_t)0x0050)            /* PF[13] pin */
#define AFIO_EXTICR4_EXTI13_PG               ((uint16_t)0x0060)            /* PG[13] pin */

/* EXTI14 configuration */  
#define AFIO_EXTICR4_EXTI14_PA               ((uint16_t)0x0000)            /* PA[14] pin */
#define AFIO_EXTICR4_EXTI14_PB               ((uint16_t)0x0100)            /* PB[14] pin */
#define AFIO_EXTICR4_EXTI14_PC               ((uint16_t)0x0200)            /* PC[14] pin */
#define AFIO_EXTICR4_EXTI14_PD               ((uint16_t)0x0300)            /* PD[14] pin */
#define AFIO_EXTICR4_EXTI14_PE               ((uint16_t)0x0400)            /* PE[14] pin */
#define AFIO_EXTICR4_EXTI14_PF               ((uint16_t)0x0500)            /* PF[14] pin */
#define AFIO_EXTICR4_EXTI14_PG               ((uint16_t)0x0600)            /* PG[14] pin */

/* EXTI15 configuration */
#define AFIO_EXTICR4_EXTI15_PA               ((uint16_t)0x0000)            /* PA[15] pin */
#define AFIO_EXTICR4_EXTI15_PB               ((uint16_t)0x1000)            /* PB[15] pin */
#define AFIO_EXTICR4_EXTI15_PC               ((uint16_t)0x2000)            /* PC[15] pin */
#define AFIO_EXTICR4_EXTI15_PD               ((uint16_t)0x3000)            /* PD[15] pin */
#define AFIO_EXTICR4_EXTI15_PE               ((uint16_t)0x4000)            /* PE[15] pin */
#define AFIO_EXTICR4_EXTI15_PF               ((uint16_t)0x5000)            /* PF[15] pin */
#define AFIO_EXTICR4_EXTI15_PG               ((uint16_t)0x6000)            /* PG[15] pin */

/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for EXTI_IMR register  *******************/
#define  EXTI_IMR_MR0                        ((uint32_t)0x00000001)        /* Interrupt Mask on line 0 */
#define  EXTI_IMR_MR1                        ((uint32_t)0x00000002)        /* Interrupt Mask on line 1 */
#define  EXTI_IMR_MR2                        ((uint32_t)0x00000004)        /* Interrupt Mask on line 2 */
#define  EXTI_IMR_MR3                        ((uint32_t)0x00000008)        /* Interrupt Mask on line 3 */
#define  EXTI_IMR_MR4                        ((uint32_t)0x00000010)        /* Interrupt Mask on line 4 */
#define  EXTI_IMR_MR5                        ((uint32_t)0x00000020)        /* Interrupt Mask on line 5 */
#define  EXTI_IMR_MR6                        ((uint32_t)0x00000040)        /* Interrupt Mask on line 6 */
#define  EXTI_IMR_MR7                        ((uint32_t)0x00000080)        /* Interrupt Mask on line 7 */
#define  EXTI_IMR_MR8                        ((uint32_t)0x00000100)        /* Interrupt Mask on line 8 */
#define  EXTI_IMR_MR9                        ((uint32_t)0x00000200)        /* Interrupt Mask on line 9 */
#define  EXTI_IMR_MR10                       ((uint32_t)0x00000400)        /* Interrupt Mask on line 10 */
#define  EXTI_IMR_MR11                       ((uint32_t)0x00000800)        /* Interrupt Mask on line 11 */
#define  EXTI_IMR_MR12                       ((uint32_t)0x00001000)        /* Interrupt Mask on line 12 */
#define  EXTI_IMR_MR13                       ((uint32_t)0x00002000)        /* Interrupt Mask on line 13 */
#define  EXTI_IMR_MR14                       ((uint32_t)0x00004000)        /* Interrupt Mask on line 14 */
#define  EXTI_IMR_MR15                       ((uint32_t)0x00008000)        /* Interrupt Mask on line 15 */
#define  EXTI_IMR_MR16                       ((uint32_t)0x00010000)        /* Interrupt Mask on line 16 */
#define  EXTI_IMR_MR17                       ((uint32_t)0x00020000)        /* Interrupt Mask on line 17 */
#define  EXTI_IMR_MR18                       ((uint32_t)0x00040000)        /* Interrupt Mask on line 18 */
#define  EXTI_IMR_MR19                       ((uint32_t)0x00080000)        /* Interrupt Mask on line 19 */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define  EXTI_EMR_MR0                        ((uint32_t)0x00000001)        /* Event Mask on line 0 */
#define  EXTI_EMR_MR1                        ((uint32_t)0x00000002)        /* Event Mask on line 1 */
#define  EXTI_EMR_MR2                        ((uint32_t)0x00000004)        /* Event Mask on line 2 */
#define  EXTI_EMR_MR3                        ((uint32_t)0x00000008)        /* Event Mask on line 3 */
#define  EXTI_EMR_MR4                        ((uint32_t)0x00000010)        /* Event Mask on line 4 */
#define  EXTI_EMR_MR5                        ((uint32_t)0x00000020)        /* Event Mask on line 5 */
#define  EXTI_EMR_MR6                        ((uint32_t)0x00000040)        /* Event Mask on line 6 */
#define  EXTI_EMR_MR7                        ((uint32_t)0x00000080)        /* Event Mask on line 7 */
#define  EXTI_EMR_MR8                        ((uint32_t)0x00000100)        /* Event Mask on line 8 */
#define  EXTI_EMR_MR9                        ((uint32_t)0x00000200)        /* Event Mask on line 9 */
#define  EXTI_EMR_MR10                       ((uint32_t)0x00000400)        /* Event Mask on line 10 */
#define  EXTI_EMR_MR11                       ((uint32_t)0x00000800)        /* Event Mask on line 11 */
#define  EXTI_EMR_MR12                       ((uint32_t)0x00001000)        /* Event Mask on line 12 */
#define  EXTI_EMR_MR13                       ((uint32_t)0x00002000)        /* Event Mask on line 13 */
#define  EXTI_EMR_MR14                       ((uint32_t)0x00004000)        /* Event Mask on line 14 */
#define  EXTI_EMR_MR15                       ((uint32_t)0x00008000)        /* Event Mask on line 15 */
#define  EXTI_EMR_MR16                       ((uint32_t)0x00010000)        /* Event Mask on line 16 */
#define  EXTI_EMR_MR17                       ((uint32_t)0x00020000)        /* Event Mask on line 17 */
#define  EXTI_EMR_MR18                       ((uint32_t)0x00040000)        /* Event Mask on line 18 */
#define  EXTI_EMR_MR19                       ((uint32_t)0x00080000)        /* Event Mask on line 19 */

/******************  Bit definition for EXTI_RTSR register  *******************/
#define  EXTI_RTSR_TR0                       ((uint32_t)0x00000001)        /* Rising trigger event configuration bit of line 0 */
#define  EXTI_RTSR_TR1                       ((uint32_t)0x00000002)        /* Rising trigger event configuration bit of line 1 */
#define  EXTI_RTSR_TR2                       ((uint32_t)0x00000004)        /* Rising trigger event configuration bit of line 2 */
#define  EXTI_RTSR_TR3                       ((uint32_t)0x00000008)        /* Rising trigger event configuration bit of line 3 */
#define  EXTI_RTSR_TR4                       ((uint32_t)0x00000010)        /* Rising trigger event configuration bit of line 4 */
#define  EXTI_RTSR_TR5                       ((uint32_t)0x00000020)        /* Rising trigger event configuration bit of line 5 */
#define  EXTI_RTSR_TR6                       ((uint32_t)0x00000040)        /* Rising trigger event configuration bit of line 6 */
#define  EXTI_RTSR_TR7                       ((uint32_t)0x00000080)        /* Rising trigger event configuration bit of line 7 */
#define  EXTI_RTSR_TR8                       ((uint32_t)0x00000100)        /* Rising trigger event configuration bit of line 8 */
#define  EXTI_RTSR_TR9                       ((uint32_t)0x00000200)        /* Rising trigger event configuration bit of line 9 */
#define  EXTI_RTSR_TR10                      ((uint32_t)0x00000400)        /* Rising trigger event configuration bit of line 10 */
#define  EXTI_RTSR_TR11                      ((uint32_t)0x00000800)        /* Rising trigger event configuration bit of line 11 */
#define  EXTI_RTSR_TR12                      ((uint32_t)0x00001000)        /* Rising trigger event configuration bit of line 12 */
#define  EXTI_RTSR_TR13                      ((uint32_t)0x00002000)        /* Rising trigger event configuration bit of line 13 */
#define  EXTI_RTSR_TR14                      ((uint32_t)0x00004000)        /* Rising trigger event configuration bit of line 14 */
#define  EXTI_RTSR_TR15                      ((uint32_t)0x00008000)        /* Rising trigger event configuration bit of line 15 */
#define  EXTI_RTSR_TR16                      ((uint32_t)0x00010000)        /* Rising trigger event configuration bit of line 16 */
#define  EXTI_RTSR_TR17                      ((uint32_t)0x00020000)        /* Rising trigger event configuration bit of line 17 */
#define  EXTI_RTSR_TR18                      ((uint32_t)0x00040000)        /* Rising trigger event configuration bit of line 18 */
#define  EXTI_RTSR_TR19                      ((uint32_t)0x00080000)        /* Rising trigger event configuration bit of line 19 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define  EXTI_FTSR_TR0                       ((uint32_t)0x00000001)        /* Falling trigger event configuration bit of line 0 */
#define  EXTI_FTSR_TR1                       ((uint32_t)0x00000002)        /* Falling trigger event configuration bit of line 1 */
#define  EXTI_FTSR_TR2                       ((uint32_t)0x00000004)        /* Falling trigger event configuration bit of line 2 */
#define  EXTI_FTSR_TR3                       ((uint32_t)0x00000008)        /* Falling trigger event configuration bit of line 3 */
#define  EXTI_FTSR_TR4                       ((uint32_t)0x00000010)        /* Falling trigger event configuration bit of line 4 */
#define  EXTI_FTSR_TR5                       ((uint32_t)0x00000020)        /* Falling trigger event configuration bit of line 5 */
#define  EXTI_FTSR_TR6                       ((uint32_t)0x00000040)        /* Falling trigger event configuration bit of line 6 */
#define  EXTI_FTSR_TR7                       ((uint32_t)0x00000080)        /* Falling trigger event configuration bit of line 7 */
#define  EXTI_FTSR_TR8                       ((uint32_t)0x00000100)        /* Falling trigger event configuration bit of line 8 */
#define  EXTI_FTSR_TR9                       ((uint32_t)0x00000200)        /* Falling trigger event configuration bit of line 9 */
#define  EXTI_FTSR_TR10                      ((uint32_t)0x00000400)        /* Falling trigger event configuration bit of line 10 */
#define  EXTI_FTSR_TR11                      ((uint32_t)0x00000800)        /* Falling trigger event configuration bit of line 11 */
#define  EXTI_FTSR_TR12                      ((uint32_t)0x00001000)        /* Falling trigger event configuration bit of line 12 */
#define  EXTI_FTSR_TR13                      ((uint32_t)0x00002000)        /* Falling trigger event configuration bit of line 13 */
#define  EXTI_FTSR_TR14                      ((uint32_t)0x00004000)        /* Falling trigger event configuration bit of line 14 */
#define  EXTI_FTSR_TR15                      ((uint32_t)0x00008000)        /* Falling trigger event configuration bit of line 15 */
#define  EXTI_FTSR_TR16                      ((uint32_t)0x00010000)        /* Falling trigger event configuration bit of line 16 */
#define  EXTI_FTSR_TR17                      ((uint32_t)0x00020000)        /* Falling trigger event configuration bit of line 17 */
#define  EXTI_FTSR_TR18                      ((uint32_t)0x00040000)        /* Falling trigger event configuration bit of line 18 */
#define  EXTI_FTSR_TR19                      ((uint32_t)0x00080000)        /* Falling trigger event configuration bit of line 19 */

/******************  Bit definition for EXTI_SWIER register  ******************/
#define  EXTI_SWIER_SWIER0                   ((uint32_t)0x00000001)        /* Software Interrupt on line 0 */
#define  EXTI_SWIER_SWIER1                   ((uint32_t)0x00000002)        /* Software Interrupt on line 1 */
#define  EXTI_SWIER_SWIER2                   ((uint32_t)0x00000004)        /* Software Interrupt on line 2 */
#define  EXTI_SWIER_SWIER3                   ((uint32_t)0x00000008)        /* Software Interrupt on line 3 */
#define  EXTI_SWIER_SWIER4                   ((uint32_t)0x00000010)        /* Software Interrupt on line 4 */
#define  EXTI_SWIER_SWIER5                   ((uint32_t)0x00000020)        /* Software Interrupt on line 5 */
#define  EXTI_SWIER_SWIER6                   ((uint32_t)0x00000040)        /* Software Interrupt on line 6 */
#define  EXTI_SWIER_SWIER7                   ((uint32_t)0x00000080)        /* Software Interrupt on line 7 */
#define  EXTI_SWIER_SWIER8                   ((uint32_t)0x00000100)        /* Software Interrupt on line 8 */
#define  EXTI_SWIER_SWIER9                   ((uint32_t)0x00000200)        /* Software Interrupt on line 9 */
#define  EXTI_SWIER_SWIER10                  ((uint32_t)0x00000400)        /* Software Interrupt on line 10 */
#define  EXTI_SWIER_SWIER11                  ((uint32_t)0x00000800)        /* Software Interrupt on line 11 */
#define  EXTI_SWIER_SWIER12                  ((uint32_t)0x00001000)        /* Software Interrupt on line 12 */
#define  EXTI_SWIER_SWIER13                  ((uint32_t)0x00002000)        /* Software Interrupt on line 13 */
#define  EXTI_SWIER_SWIER14                  ((uint32_t)0x00004000)        /* Software Interrupt on line 14 */
#define  EXTI_SWIER_SWIER15                  ((uint32_t)0x00008000)        /* Software Interrupt on line 15 */
#define  EXTI_SWIER_SWIER16                  ((uint32_t)0x00010000)        /* Software Interrupt on line 16 */
#define  EXTI_SWIER_SWIER17                  ((uint32_t)0x00020000)        /* Software Interrupt on line 17 */
#define  EXTI_SWIER_SWIER18                  ((uint32_t)0x00040000)        /* Software Interrupt on line 18 */
#define  EXTI_SWIER_SWIER19                  ((uint32_t)0x00080000)        /* Software Interrupt on line 19 */

/*******************  Bit definition for EXTI_PR register  ********************/
#define  EXTI_PR_PR0                         ((uint32_t)0x00000001)        /* Pending bit for line 0 */
#define  EXTI_PR_PR1                         ((uint32_t)0x00000002)        /* Pending bit for line 1 */
#define  EXTI_PR_PR2                         ((uint32_t)0x00000004)        /* Pending bit for line 2 */
#define  EXTI_PR_PR3                         ((uint32_t)0x00000008)        /* Pending bit for line 3 */
#define  EXTI_PR_PR4                         ((uint32_t)0x00000010)        /* Pending bit for line 4 */
#define  EXTI_PR_PR5                         ((uint32_t)0x00000020)        /* Pending bit for line 5 */
#define  EXTI_PR_PR6                         ((uint32_t)0x00000040)        /* Pending bit for line 6 */
#define  EXTI_PR_PR7                         ((uint32_t)0x00000080)        /* Pending bit for line 7 */
#define  EXTI_PR_PR8                         ((uint32_t)0x00000100)        /* Pending bit for line 8 */
#define  EXTI_PR_PR9                         ((uint32_t)0x00000200)        /* Pending bit for line 9 */
#define  EXTI_PR_PR10                        ((uint32_t)0x00000400)        /* Pending bit for line 10 */
#define  EXTI_PR_PR11                        ((uint32_t)0x00000800)        /* Pending bit for line 11 */
#define  EXTI_PR_PR12                        ((uint32_t)0x00001000)        /* Pending bit for line 12 */
#define  EXTI_PR_PR13                        ((uint32_t)0x00002000)        /* Pending bit for line 13 */
#define  EXTI_PR_PR14                        ((uint32_t)0x00004000)        /* Pending bit for line 14 */
#define  EXTI_PR_PR15                        ((uint32_t)0x00008000)        /* Pending bit for line 15 */
#define  EXTI_PR_PR16                        ((uint32_t)0x00010000)        /* Pending bit for line 16 */
#define  EXTI_PR_PR17                        ((uint32_t)0x00020000)        /* Pending bit for line 17 */
#define  EXTI_PR_PR18                        ((uint32_t)0x00040000)        /* Pending bit for line 18 */
#define  EXTI_PR_PR19                        ((uint32_t)0x00080000)        /* Pending bit for line 19 */

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for DMA_ISR register  ********************/
#define  DMA_ISR_GIF1                        ((uint32_t)0x00000001)        /* Channel 1 Global interrupt flag */
#define  DMA_ISR_TCIF1                       ((uint32_t)0x00000002)        /* Channel 1 Transfer Complete flag */
#define  DMA_ISR_HTIF1                       ((uint32_t)0x00000004)        /* Channel 1 Half Transfer flag */
#define  DMA_ISR_TEIF1                       ((uint32_t)0x00000008)        /* Channel 1 Transfer Error flag */
#define  DMA_ISR_GIF2                        ((uint32_t)0x00000010)        /* Channel 2 Global interrupt flag */
#define  DMA_ISR_TCIF2                       ((uint32_t)0x00000020)        /* Channel 2 Transfer Complete flag */
#define  DMA_ISR_HTIF2                       ((uint32_t)0x00000040)        /* Channel 2 Half Transfer flag */
#define  DMA_ISR_TEIF2                       ((uint32_t)0x00000080)        /* Channel 2 Transfer Error flag */
#define  DMA_ISR_GIF3                        ((uint32_t)0x00000100)        /* Channel 3 Global interrupt flag */
#define  DMA_ISR_TCIF3                       ((uint32_t)0x00000200)        /* Channel 3 Transfer Complete flag */
#define  DMA_ISR_HTIF3                       ((uint32_t)0x00000400)        /* Channel 3 Half Transfer flag */
#define  DMA_ISR_TEIF3                       ((uint32_t)0x00000800)        /* Channel 3 Transfer Error flag */
#define  DMA_ISR_GIF4                        ((uint32_t)0x00001000)        /* Channel 4 Global interrupt flag */
#define  DMA_ISR_TCIF4                       ((uint32_t)0x00002000)        /* Channel 4 Transfer Complete flag */
#define  DMA_ISR_HTIF4                       ((uint32_t)0x00004000)        /* Channel 4 Half Transfer flag */
#define  DMA_ISR_TEIF4                       ((uint32_t)0x00008000)        /* Channel 4 Transfer Error flag */
#define  DMA_ISR_GIF5                        ((uint32_t)0x00010000)        /* Channel 5 Global interrupt flag */
#define  DMA_ISR_TCIF5                       ((uint32_t)0x00020000)        /* Channel 5 Transfer Complete flag */
#define  DMA_ISR_HTIF5                       ((uint32_t)0x00040000)        /* Channel 5 Half Transfer flag */
#define  DMA_ISR_TEIF5                       ((uint32_t)0x00080000)        /* Channel 5 Transfer Error flag */
#define  DMA_ISR_GIF6                        ((uint32_t)0x00100000)        /* Channel 6 Global interrupt flag */
#define  DMA_ISR_TCIF6                       ((uint32_t)0x00200000)        /* Channel 6 Transfer Complete flag */
#define  DMA_ISR_HTIF6                       ((uint32_t)0x00400000)        /* Channel 6 Half Transfer flag */
#define  DMA_ISR_TEIF6                       ((uint32_t)0x00800000)        /* Channel 6 Transfer Error flag */
#define  DMA_ISR_GIF7                        ((uint32_t)0x01000000)        /* Channel 7 Global interrupt flag */
#define  DMA_ISR_TCIF7                       ((uint32_t)0x02000000)        /* Channel 7 Transfer Complete flag */
#define  DMA_ISR_HTIF7                       ((uint32_t)0x04000000)        /* Channel 7 Half Transfer flag */
#define  DMA_ISR_TEIF7                       ((uint32_t)0x08000000)        /* Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define  DMA_IFCR_CGIF1                      ((uint32_t)0x00000001)        /* Channel 1 Global interrupt clear */
#define  DMA_IFCR_CTCIF1                     ((uint32_t)0x00000002)        /* Channel 1 Transfer Complete clear */
#define  DMA_IFCR_CHTIF1                     ((uint32_t)0x00000004)        /* Channel 1 Half Transfer clear */
#define  DMA_IFCR_CTEIF1                     ((uint32_t)0x00000008)        /* Channel 1 Transfer Error clear */
#define  DMA_IFCR_CGIF2                      ((uint32_t)0x00000010)        /* Channel 2 Global interrupt clear */
#define  DMA_IFCR_CTCIF2                     ((uint32_t)0x00000020)        /* Channel 2 Transfer Complete clear */
#define  DMA_IFCR_CHTIF2                     ((uint32_t)0x00000040)        /* Channel 2 Half Transfer clear */
#define  DMA_IFCR_CTEIF2                     ((uint32_t)0x00000080)        /* Channel 2 Transfer Error clear */
#define  DMA_IFCR_CGIF3                      ((uint32_t)0x00000100)        /* Channel 3 Global interrupt clear */
#define  DMA_IFCR_CTCIF3                     ((uint32_t)0x00000200)        /* Channel 3 Transfer Complete clear */
#define  DMA_IFCR_CHTIF3                     ((uint32_t)0x00000400)        /* Channel 3 Half Transfer clear */
#define  DMA_IFCR_CTEIF3                     ((uint32_t)0x00000800)        /* Channel 3 Transfer Error clear */
#define  DMA_IFCR_CGIF4                      ((uint32_t)0x00001000)        /* Channel 4 Global interrupt clear */
#define  DMA_IFCR_CTCIF4                     ((uint32_t)0x00002000)        /* Channel 4 Transfer Complete clear */
#define  DMA_IFCR_CHTIF4                     ((uint32_t)0x00004000)        /* Channel 4 Half Transfer clear */
#define  DMA_IFCR_CTEIF4                     ((uint32_t)0x00008000)        /* Channel 4 Transfer Error clear */
#define  DMA_IFCR_CGIF5                      ((uint32_t)0x00010000)        /* Channel 5 Global interrupt clear */
#define  DMA_IFCR_CTCIF5                     ((uint32_t)0x00020000)        /* Channel 5 Transfer Complete clear */
#define  DMA_IFCR_CHTIF5                     ((uint32_t)0x00040000)        /* Channel 5 Half Transfer clear */
#define  DMA_IFCR_CTEIF5                     ((uint32_t)0x00080000)        /* Channel 5 Transfer Error clear */
#define  DMA_IFCR_CGIF6                      ((uint32_t)0x00100000)        /* Channel 6 Global interrupt clear */
#define  DMA_IFCR_CTCIF6                     ((uint32_t)0x00200000)        /* Channel 6 Transfer Complete clear */
#define  DMA_IFCR_CHTIF6                     ((uint32_t)0x00400000)        /* Channel 6 Half Transfer clear */
#define  DMA_IFCR_CTEIF6                     ((uint32_t)0x00800000)        /* Channel 6 Transfer Error clear */
#define  DMA_IFCR_CGIF7                      ((uint32_t)0x01000000)        /* Channel 7 Global interrupt clear */
#define  DMA_IFCR_CTCIF7                     ((uint32_t)0x02000000)        /* Channel 7 Transfer Complete clear */
#define  DMA_IFCR_CHTIF7                     ((uint32_t)0x04000000)        /* Channel 7 Half Transfer clear */
#define  DMA_IFCR_CTEIF7                     ((uint32_t)0x08000000)        /* Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCR1 register  *******************/
#define  DMA_CCR1_EN                         ((uint16_t)0x0001)            /* Channel enable*/
#define  DMA_CCR1_TCIE                       ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR1_HTIE                       ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR1_TEIE                       ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR1_DIR                        ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CCR1_CIRC                       ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CCR1_PINC                       ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR1_MINC                       ((uint16_t)0x0080)            /* Memory increment mode */

#define  DMA_CCR1_PSIZE                      ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR1_PSIZE_0                    ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CCR1_PSIZE_1                    ((uint16_t)0x0200)            /* Bit 1 */

#define  DMA_CCR1_MSIZE                      ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR1_MSIZE_0                    ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CCR1_MSIZE_1                    ((uint16_t)0x0800)            /* Bit 1 */

#define  DMA_CCR1_PL                         ((uint16_t)0x3000)            /* PL[1:0] bits(Channel Priority level) */
#define  DMA_CCR1_PL_0                       ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CCR1_PL_1                       ((uint16_t)0x2000)            /* Bit 1 */

#define  DMA_CCR1_MEM2MEM                    ((uint16_t)0x4000)            /* Memory to memory mode */

/*******************  Bit definition for DMA_CCR2 register  *******************/
#define  DMA_CCR2_EN                         ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CCR2_TCIE                       ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR2_HTIE                       ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR2_TEIE                       ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR2_DIR                        ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CCR2_CIRC                       ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CCR2_PINC                       ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR2_MINC                       ((uint16_t)0x0080)            /* Memory increment mode */

#define  DMA_CCR2_PSIZE                      ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR2_PSIZE_0                    ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CCR2_PSIZE_1                    ((uint16_t)0x0200)            /* Bit 1 */

#define  DMA_CCR2_MSIZE                      ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR2_MSIZE_0                    ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CCR2_MSIZE_1                    ((uint16_t)0x0800)            /* Bit 1 */

#define  DMA_CCR2_PL                         ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR2_PL_0                       ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CCR2_PL_1                       ((uint16_t)0x2000)            /* Bit 1 */

#define  DMA_CCR2_MEM2MEM                    ((uint16_t)0x4000)            /* Memory to memory mode */

/*******************  Bit definition for DMA_CCR3 register  *******************/
#define  DMA_CCR3_EN                         ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CCR3_TCIE                       ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR3_HTIE                       ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR3_TEIE                       ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR3_DIR                        ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CCR3_CIRC                       ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CCR3_PINC                       ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR3_MINC                       ((uint16_t)0x0080)            /* Memory increment mode */

#define  DMA_CCR3_PSIZE                      ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR3_PSIZE_0                    ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CCR3_PSIZE_1                    ((uint16_t)0x0200)            /* Bit 1 */

#define  DMA_CCR3_MSIZE                      ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR3_MSIZE_0                    ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CCR3_MSIZE_1                    ((uint16_t)0x0800)            /* Bit 1 */

#define  DMA_CCR3_PL                         ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR3_PL_0                       ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CCR3_PL_1                       ((uint16_t)0x2000)            /* Bit 1 */

#define  DMA_CCR3_MEM2MEM                    ((uint16_t)0x4000)            /* Memory to memory mode */

/*******************  Bit definition for DMA_CCR4 register  *******************/
#define  DMA_CCR4_EN                         ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CCR4_TCIE                       ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR4_HTIE                       ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR4_TEIE                       ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR4_DIR                        ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CCR4_CIRC                       ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CCR4_PINC                       ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR4_MINC                       ((uint16_t)0x0080)            /* Memory increment mode */

#define  DMA_CCR4_PSIZE                      ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR4_PSIZE_0                    ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CCR4_PSIZE_1                    ((uint16_t)0x0200)            /* Bit 1 */

#define  DMA_CCR4_MSIZE                      ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR4_MSIZE_0                    ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CCR4_MSIZE_1                    ((uint16_t)0x0800)            /* Bit 1 */

#define  DMA_CCR4_PL                         ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR4_PL_0                       ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CCR4_PL_1                       ((uint16_t)0x2000)            /* Bit 1 */

#define  DMA_CCR4_MEM2MEM                    ((uint16_t)0x4000)            /* Memory to memory mode */

/******************  Bit definition for DMA_CCR5 register  *******************/
#define  DMA_CCR5_EN                         ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CCR5_TCIE                       ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR5_HTIE                       ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR5_TEIE                       ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR5_DIR                        ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CCR5_CIRC                       ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CCR5_PINC                       ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR5_MINC                       ((uint16_t)0x0080)            /* Memory increment mode */

#define  DMA_CCR5_PSIZE                      ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR5_PSIZE_0                    ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CCR5_PSIZE_1                    ((uint16_t)0x0200)            /* Bit 1 */

#define  DMA_CCR5_MSIZE                      ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR5_MSIZE_0                    ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CCR5_MSIZE_1                    ((uint16_t)0x0800)            /* Bit 1 */

#define  DMA_CCR5_PL                         ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR5_PL_0                       ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CCR5_PL_1                       ((uint16_t)0x2000)            /* Bit 1 */

#define  DMA_CCR5_MEM2MEM                    ((uint16_t)0x4000)            /* Memory to memory mode enable */

/*******************  Bit definition for DMA_CCR6 register  *******************/
#define  DMA_CCR6_EN                         ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CCR6_TCIE                       ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR6_HTIE                       ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR6_TEIE                       ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR6_DIR                        ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CCR6_CIRC                       ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CCR6_PINC                       ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR6_MINC                       ((uint16_t)0x0080)            /* Memory increment mode */

#define  DMA_CCR6_PSIZE                      ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR6_PSIZE_0                    ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CCR6_PSIZE_1                    ((uint16_t)0x0200)            /* Bit 1 */

#define  DMA_CCR6_MSIZE                      ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR6_MSIZE_0                    ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CCR6_MSIZE_1                    ((uint16_t)0x0800)            /* Bit 1 */

#define  DMA_CCR6_PL                         ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR6_PL_0                       ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CCR6_PL_1                       ((uint16_t)0x2000)            /* Bit 1 */

#define  DMA_CCR6_MEM2MEM                    ((uint16_t)0x4000)            /* Memory to memory mode */

/*******************  Bit definition for DMA_CCR7 register  *******************/
#define  DMA_CCR7_EN                         ((uint16_t)0x0001)            /* Channel enable */
#define  DMA_CCR7_TCIE                       ((uint16_t)0x0002)            /* Transfer complete interrupt enable */
#define  DMA_CCR7_HTIE                       ((uint16_t)0x0004)            /* Half Transfer interrupt enable */
#define  DMA_CCR7_TEIE                       ((uint16_t)0x0008)            /* Transfer error interrupt enable */
#define  DMA_CCR7_DIR                        ((uint16_t)0x0010)            /* Data transfer direction */
#define  DMA_CCR7_CIRC                       ((uint16_t)0x0020)            /* Circular mode */
#define  DMA_CCR7_PINC                       ((uint16_t)0x0040)            /* Peripheral increment mode */
#define  DMA_CCR7_MINC                       ((uint16_t)0x0080)            /* Memory increment mode */

#define  DMA_CCR7_PSIZE            ,         ((uint16_t)0x0300)            /* PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR7_PSIZE_0                    ((uint16_t)0x0100)            /* Bit 0 */
#define  DMA_CCR7_PSIZE_1                    ((uint16_t)0x0200)            /* Bit 1 */

#define  DMA_CCR7_MSIZE                      ((uint16_t)0x0C00)            /* MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR7_MSIZE_0                    ((uint16_t)0x0400)            /* Bit 0 */
#define  DMA_CCR7_MSIZE_1                    ((uint16_t)0x0800)            /* Bit 1 */

#define  DMA_CCR7_PL                         ((uint16_t)0x3000)            /* PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR7_PL_0                       ((uint16_t)0x1000)            /* Bit 0 */
#define  DMA_CCR7_PL_1                       ((uint16_t)0x2000)            /* Bit 1 */

#define  DMA_CCR7_MEM2MEM                    ((uint16_t)0x4000)            /* Memory to memory mode enable */

/******************  Bit definition for DMA_CNDTR1 register  ******************/
#define  DMA_CNDTR1_NDT                      ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR2 register  ******************/
#define  DMA_CNDTR2_NDT                      ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR3 register  ******************/
#define  DMA_CNDTR3_NDT                      ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR4 register  ******************/
#define  DMA_CNDTR4_NDT                      ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR5 register  ******************/
#define  DMA_CNDTR5_NDT                      ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR6 register  ******************/
#define  DMA_CNDTR6_NDT                      ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR7 register  ******************/
#define  DMA_CNDTR7_NDT                      ((uint16_t)0xFFFF)            /* Number of data to Transfer */

/******************  Bit definition for DMA_CPAR1 register  *******************/
#define  DMA_CPAR1_PA                        ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_CPAR2 register  *******************/
#define  DMA_CPAR2_PA                        ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_CPAR3 register  *******************/
#define  DMA_CPAR3_PA                        ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */


/******************  Bit definition for DMA_CPAR4 register  *******************/
#define  DMA_CPAR4_PA                        ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_CPAR5 register  *******************/
#define  DMA_CPAR5_PA                        ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_CPAR6 register  *******************/
#define  DMA_CPAR6_PA                        ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */


/******************  Bit definition for DMA_CPAR7 register  *******************/
#define  DMA_CPAR7_PA                        ((uint32_t)0xFFFFFFFF)        /* Peripheral Address */

/******************  Bit definition for DMA_CMAR1 register  *******************/
#define  DMA_CMAR1_MA                        ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_CMAR2 register  *******************/
#define  DMA_CMAR2_MA                        ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_CMAR3 register  *******************/
#define  DMA_CMAR3_MA                        ((uint32_t)0xFFFFFFFF)        /* Memory Address */


/******************  Bit definition for DMA_CMAR4 register  *******************/
#define  DMA_CMAR4_MA                        ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_CMAR5 register  *******************/
#define  DMA_CMAR5_MA                        ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_CMAR6 register  *******************/
#define  DMA_CMAR6_MA                        ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************  Bit definition for DMA_CMAR7 register  *******************/
#define  DMA_CMAR7_MA                        ((uint32_t)0xFFFFFFFF)        /* Memory Address */

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for ADC_SR register  ********************/
#define  ADC_SR_AWD                          ((uint8_t)0x01)               /* Analog watchdog flag */
#define  ADC_SR_EOC                          ((uint8_t)0x02)               /* End of conversion */
#define  ADC_SR_JEOC                         ((uint8_t)0x04)               /* Injected channel end of conversion */
#define  ADC_SR_JSTRT                        ((uint8_t)0x08)               /* Injected channel Start flag */
#define  ADC_SR_STRT                         ((uint8_t)0x10)               /* Regular channel Start flag */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define  ADC_CR1_AWDCH                       ((uint32_t)0x0000001F)        /* AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_CR1_AWDCH_0                     ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_CR1_AWDCH_1                     ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_CR1_AWDCH_2                     ((uint32_t)0x00000004)        /* Bit 2 */
#define  ADC_CR1_AWDCH_3                     ((uint32_t)0x00000008)        /* Bit 3 */
#define  ADC_CR1_AWDCH_4                     ((uint32_t)0x00000010)        /* Bit 4 */

#define  ADC_CR1_EOCIE                       ((uint32_t)0x00000020)        /* Interrupt enable for EOC */
#define  ADC_CR1_AWDIE                       ((uint32_t)0x00000040)        /* Analog Watchdog interrupt enable */
#define  ADC_CR1_JEOCIE                      ((uint32_t)0x00000080)        /* Interrupt enable for injected channels */
#define  ADC_CR1_SCAN                        ((uint32_t)0x00000100)        /* Scan mode */
#define  ADC_CR1_AWDSGL                      ((uint32_t)0x00000200)        /* Enable the watchdog on a single channel in scan mode */
#define  ADC_CR1_JAUTO                       ((uint32_t)0x00000400)        /* Automatic injected group conversion */
#define  ADC_CR1_DISCEN                      ((uint32_t)0x00000800)        /* Discontinuous mode on regular channels */
#define  ADC_CR1_JDISCEN                     ((uint32_t)0x00001000)        /* Discontinuous mode on injected channels */

#define  ADC_CR1_DISCNUM                     ((uint32_t)0x0000E000)        /* DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  ADC_CR1_DISCNUM_0                   ((uint32_t)0x00002000)        /* Bit 0 */
#define  ADC_CR1_DISCNUM_1                   ((uint32_t)0x00004000)        /* Bit 1 */
#define  ADC_CR1_DISCNUM_2                   ((uint32_t)0x00008000)        /* Bit 2 */

#define  ADC_CR1_DUALMOD                     ((uint32_t)0x000F0000)        /* DUALMOD[3:0] bits (Dual mode selection) */
#define  ADC_CR1_DUALMOD_0                   ((uint32_t)0x00010000)        /* Bit 0 */
#define  ADC_CR1_DUALMOD_1                   ((uint32_t)0x00020000)        /* Bit 1 */
#define  ADC_CR1_DUALMOD_2                   ((uint32_t)0x00040000)        /* Bit 2 */
#define  ADC_CR1_DUALMOD_3                   ((uint32_t)0x00080000)        /* Bit 3 */

#define  ADC_CR1_JAWDEN                      ((uint32_t)0x00400000)        /* Analog watchdog enable on injected channels */
#define  ADC_CR1_AWDEN                       ((uint32_t)0x00800000)        /* Analog watchdog enable on regular channels */

  
/*******************  Bit definition for ADC_CR2 register  ********************/
#define  ADC_CR2_ADON                        ((uint32_t)0x00000001)        /* A/D Converter ON / OFF */
#define  ADC_CR2_CONT                        ((uint32_t)0x00000002)        /* Continuous Conversion */
#define  ADC_CR2_CAL                         ((uint32_t)0x00000004)        /* A/D Calibration */
#define  ADC_CR2_RSTCAL                      ((uint32_t)0x00000008)        /* Reset Calibration */
#define  ADC_CR2_DMA                         ((uint32_t)0x00000100)        /* Direct Memory access mode */
#define  ADC_CR2_ALIGN                       ((uint32_t)0x00000800)        /* Data Alignment */

#define  ADC_CR2_JEXTSEL                     ((uint32_t)0x00007000)        /* JEXTSEL[2:0] bits (External event select for injected group) */
#define  ADC_CR2_JEXTSEL_0                   ((uint32_t)0x00001000)        /* Bit 0 */
#define  ADC_CR2_JEXTSEL_1                   ((uint32_t)0x00002000)        /* Bit 1 */
#define  ADC_CR2_JEXTSEL_2                   ((uint32_t)0x00004000)        /* Bit 2 */

#define  ADC_CR2_JEXTTRIG                    ((uint32_t)0x00008000)        /* External Trigger Conversion mode for injected channels */

#define  ADC_CR2_EXTSEL                      ((uint32_t)0x000E0000)        /* EXTSEL[2:0] bits (External Event Select for regular group) */
#define  ADC_CR2_EXTSEL_0                    ((uint32_t)0x00020000)        /* Bit 0 */
#define  ADC_CR2_EXTSEL_1                    ((uint32_t)0x00040000)        /* Bit 1 */
#define  ADC_CR2_EXTSEL_2                    ((uint32_t)0x00080000)        /* Bit 2 */

#define  ADC_CR2_EXTTRIG                     ((uint32_t)0x00100000)        /* External Trigger Conversion mode for regular channels */
#define  ADC_CR2_JSWSTART                    ((uint32_t)0x00200000)        /* Start Conversion of injected channels */
#define  ADC_CR2_SWSTART                     ((uint32_t)0x00400000)        /* Start Conversion of regular channels */
#define  ADC_CR2_TSVREFE                     ((uint32_t)0x00800000)        /* Temperature Sensor and VREFINT Enable */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define  ADC_SMPR1_SMP10                     ((uint32_t)0x00000007)        /* SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  ADC_SMPR1_SMP10_0                   ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_SMPR1_SMP10_1                   ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_SMPR1_SMP10_2                   ((uint32_t)0x00000004)        /* Bit 2 */

#define  ADC_SMPR1_SMP11                     ((uint32_t)0x00000038)        /* SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  ADC_SMPR1_SMP11_0                   ((uint32_t)0x00000008)        /* Bit 0 */
#define  ADC_SMPR1_SMP11_1                   ((uint32_t)0x00000010)        /* Bit 1 */
#define  ADC_SMPR1_SMP11_2                   ((uint32_t)0x00000020)        /* Bit 2 */

#define  ADC_SMPR1_SMP12                     ((uint32_t)0x000001C0)        /* SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  ADC_SMPR1_SMP12_0                   ((uint32_t)0x00000040)        /* Bit 0 */
#define  ADC_SMPR1_SMP12_1                   ((uint32_t)0x00000080)        /* Bit 1 */
#define  ADC_SMPR1_SMP12_2                   ((uint32_t)0x00000100)        /* Bit 2 */

#define  ADC_SMPR1_SMP13                     ((uint32_t)0x00000E00)        /* SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  ADC_SMPR1_SMP13_0                   ((uint32_t)0x00000200)        /* Bit 0 */
#define  ADC_SMPR1_SMP13_1                   ((uint32_t)0x00000400)        /* Bit 1 */
#define  ADC_SMPR1_SMP13_2                   ((uint32_t)0x00000800)        /* Bit 2 */

#define  ADC_SMPR1_SMP14                     ((uint32_t)0x00007000)        /* SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  ADC_SMPR1_SMP14_0                   ((uint32_t)0x00001000)        /* Bit 0 */
#define  ADC_SMPR1_SMP14_1                   ((uint32_t)0x00002000)        /* Bit 1 */
#define  ADC_SMPR1_SMP14_2                   ((uint32_t)0x00004000)        /* Bit 2 */

#define  ADC_SMPR1_SMP15                     ((uint32_t)0x00038000)        /* SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  ADC_SMPR1_SMP15_0                   ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_SMPR1_SMP15_1                   ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_SMPR1_SMP15_2                   ((uint32_t)0x00020000)        /* Bit 2 */

#define  ADC_SMPR1_SMP16                     ((uint32_t)0x001C0000)        /* SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  ADC_SMPR1_SMP16_0                   ((uint32_t)0x00040000)        /* Bit 0 */
#define  ADC_SMPR1_SMP16_1                   ((uint32_t)0x00080000)        /* Bit 1 */
#define  ADC_SMPR1_SMP16_2                   ((uint32_t)0x00100000)        /* Bit 2 */

#define  ADC_SMPR1_SMP17                     ((uint32_t)0x00E00000)        /* SMP17[2:0] bits (Channel 17 Sample time selection) */
#define  ADC_SMPR1_SMP17_0                   ((uint32_t)0x00200000)        /* Bit 0 */
#define  ADC_SMPR1_SMP17_1                   ((uint32_t)0x00400000)        /* Bit 1 */
#define  ADC_SMPR1_SMP17_2                   ((uint32_t)0x00800000)        /* Bit 2 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define  ADC_SMPR2_SMP0                      ((uint32_t)0x00000007)        /* SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  ADC_SMPR2_SMP0_0                    ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_SMPR2_SMP0_1                    ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_SMPR2_SMP0_2                    ((uint32_t)0x00000004)        /* Bit 2 */

#define  ADC_SMPR2_SMP1                      ((uint32_t)0x00000038)        /* SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  ADC_SMPR2_SMP1_0                    ((uint32_t)0x00000008)        /* Bit 0 */
#define  ADC_SMPR2_SMP1_1                    ((uint32_t)0x00000010)        /* Bit 1 */
#define  ADC_SMPR2_SMP1_2                    ((uint32_t)0x00000020)        /* Bit 2 */

#define  ADC_SMPR2_SMP2                      ((uint32_t)0x000001C0)        /* SMP2[2:0] bits (Channel 2 Sample time selection) */
#define  ADC_SMPR2_SMP2_0                    ((uint32_t)0x00000040)        /* Bit 0 */
#define  ADC_SMPR2_SMP2_1                    ((uint32_t)0x00000080)        /* Bit 1 */
#define  ADC_SMPR2_SMP2_2                    ((uint32_t)0x00000100)        /* Bit 2 */

#define  ADC_SMPR2_SMP3                      ((uint32_t)0x00000E00)        /* SMP3[2:0] bits (Channel 3 Sample time selection) */
#define  ADC_SMPR2_SMP3_0                    ((uint32_t)0x00000200)        /* Bit 0 */
#define  ADC_SMPR2_SMP3_1                    ((uint32_t)0x00000400)        /* Bit 1 */
#define  ADC_SMPR2_SMP3_2                    ((uint32_t)0x00000800)        /* Bit 2 */

#define  ADC_SMPR2_SMP4                      ((uint32_t)0x00007000)        /* SMP4[2:0] bits (Channel 4 Sample time selection) */
#define  ADC_SMPR2_SMP4_0                    ((uint32_t)0x00001000)        /* Bit 0 */
#define  ADC_SMPR2_SMP4_1                    ((uint32_t)0x00002000)        /* Bit 1 */
#define  ADC_SMPR2_SMP4_2                    ((uint32_t)0x00004000)        /* Bit 2 */

#define  ADC_SMPR2_SMP5                      ((uint32_t)0x00038000)        /* SMP5[2:0] bits (Channel 5 Sample time selection) */
#define  ADC_SMPR2_SMP5_0                    ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_SMPR2_SMP5_1                    ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_SMPR2_SMP5_2                    ((uint32_t)0x00020000)        /* Bit 2 */

#define  ADC_SMPR2_SMP6                      ((uint32_t)0x001C0000)        /* SMP6[2:0] bits (Channel 6 Sample time selection) */
#define  ADC_SMPR2_SMP6_0                    ((uint32_t)0x00040000)        /* Bit 0 */
#define  ADC_SMPR2_SMP6_1                    ((uint32_t)0x00080000)        /* Bit 1 */
#define  ADC_SMPR2_SMP6_2                    ((uint32_t)0x00100000)        /* Bit 2 */

#define  ADC_SMPR2_SMP7                      ((uint32_t)0x00E00000)        /* SMP7[2:0] bits (Channel 7 Sample time selection) */
#define  ADC_SMPR2_SMP7_0                    ((uint32_t)0x00200000)        /* Bit 0 */
#define  ADC_SMPR2_SMP7_1                    ((uint32_t)0x00400000)        /* Bit 1 */
#define  ADC_SMPR2_SMP7_2                    ((uint32_t)0x00800000)        /* Bit 2 */

#define  ADC_SMPR2_SMP8                      ((uint32_t)0x07000000)        /* SMP8[2:0] bits (Channel 8 Sample time selection) */
#define  ADC_SMPR2_SMP8_0                    ((uint32_t)0x01000000)        /* Bit 0 */
#define  ADC_SMPR2_SMP8_1                    ((uint32_t)0x02000000)        /* Bit 1 */
#define  ADC_SMPR2_SMP8_2                    ((uint32_t)0x04000000)        /* Bit 2 */

#define  ADC_SMPR2_SMP9                      ((uint32_t)0x38000000)        /* SMP9[2:0] bits (Channel 9 Sample time selection) */
#define  ADC_SMPR2_SMP9_0                    ((uint32_t)0x08000000)        /* Bit 0 */
#define  ADC_SMPR2_SMP9_1                    ((uint32_t)0x10000000)        /* Bit 1 */
#define  ADC_SMPR2_SMP9_2                    ((uint32_t)0x20000000)        /* Bit 2 */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define  ADC_JOFR1_JOFFSET1                  ((uint16_t)0x0FFF)            /* Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define  ADC_JOFR2_JOFFSET2                  ((uint16_t)0x0FFF)            /* Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define  ADC_JOFR3_JOFFSET3                  ((uint16_t)0x0FFF)            /* Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define  ADC_JOFR4_JOFFSET4                  ((uint16_t)0x0FFF)            /* Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define  ADC_HTR_HT                          ((uint16_t)0x0FFF)            /* Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define  ADC_LTR_LT                          ((uint16_t)0x0FFF)            /* Analog watchdog low threshold */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define  ADC_SQR1_SQ13                       ((uint32_t)0x0000001F)        /* SQ13[4:0] bits (13th conversion in regular sequence) */
#define  ADC_SQR1_SQ13_0                     ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_SQR1_SQ13_1                     ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_SQR1_SQ13_2                     ((uint32_t)0x00000004)        /* Bit 2 */
#define  ADC_SQR1_SQ13_3                     ((uint32_t)0x00000008)        /* Bit 3 */
#define  ADC_SQR1_SQ13_4                     ((uint32_t)0x00000010)        /* Bit 4 */

#define  ADC_SQR1_SQ14                       ((uint32_t)0x000003E0)        /* SQ14[4:0] bits (14th conversion in regular sequence) */
#define  ADC_SQR1_SQ14_0                     ((uint32_t)0x00000020)        /* Bit 0 */
#define  ADC_SQR1_SQ14_1                     ((uint32_t)0x00000040)        /* Bit 1 */
#define  ADC_SQR1_SQ14_2                     ((uint32_t)0x00000080)        /* Bit 2 */
#define  ADC_SQR1_SQ14_3                     ((uint32_t)0x00000100)        /* Bit 3 */
#define  ADC_SQR1_SQ14_4                     ((uint32_t)0x00000200)        /* Bit 4 */

#define  ADC_SQR1_SQ15                       ((uint32_t)0x00007C00)        /* SQ15[4:0] bits (15th conversion in regular sequence) */
#define  ADC_SQR1_SQ15_0                     ((uint32_t)0x00000400)        /* Bit 0 */
#define  ADC_SQR1_SQ15_1                     ((uint32_t)0x00000800)        /* Bit 1 */
#define  ADC_SQR1_SQ15_2                     ((uint32_t)0x00001000)        /* Bit 2 */
#define  ADC_SQR1_SQ15_3                     ((uint32_t)0x00002000)        /* Bit 3 */
#define  ADC_SQR1_SQ15_4                     ((uint32_t)0x00004000)        /* Bit 4 */

#define  ADC_SQR1_SQ16                       ((uint32_t)0x000F8000)        /* SQ16[4:0] bits (16th conversion in regular sequence) */
#define  ADC_SQR1_SQ16_0                     ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_SQR1_SQ16_1                     ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_SQR1_SQ16_2                     ((uint32_t)0x00020000)        /* Bit 2 */
#define  ADC_SQR1_SQ16_3                     ((uint32_t)0x00040000)        /* Bit 3 */
#define  ADC_SQR1_SQ16_4                     ((uint32_t)0x00080000)        /* Bit 4 */

#define  ADC_SQR1_L                          ((uint32_t)0x00F00000)        /* L[3:0] bits (Regular channel sequence length) */
#define  ADC_SQR1_L_0                        ((uint32_t)0x00100000)        /* Bit 0 */
#define  ADC_SQR1_L_1                        ((uint32_t)0x00200000)        /* Bit 1 */
#define  ADC_SQR1_L_2                        ((uint32_t)0x00400000)        /* Bit 2 */
#define  ADC_SQR1_L_3                        ((uint32_t)0x00800000)        /* Bit 3 */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define  ADC_SQR2_SQ7                        ((uint32_t)0x0000001F)        /* SQ7[4:0] bits (7th conversion in regular sequence) */
#define  ADC_SQR2_SQ7_0                      ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_SQR2_SQ7_1                      ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_SQR2_SQ7_2                      ((uint32_t)0x00000004)        /* Bit 2 */
#define  ADC_SQR2_SQ7_3                      ((uint32_t)0x00000008)        /* Bit 3 */
#define  ADC_SQR2_SQ7_4                      ((uint32_t)0x00000010)        /* Bit 4 */

#define  ADC_SQR2_SQ8                        ((uint32_t)0x000003E0)        /* SQ8[4:0] bits (8th conversion in regular sequence) */
#define  ADC_SQR2_SQ8_0                      ((uint32_t)0x00000020)        /* Bit 0 */
#define  ADC_SQR2_SQ8_1                      ((uint32_t)0x00000040)        /* Bit 1 */
#define  ADC_SQR2_SQ8_2                      ((uint32_t)0x00000080)        /* Bit 2 */
#define  ADC_SQR2_SQ8_3                      ((uint32_t)0x00000100)        /* Bit 3 */
#define  ADC_SQR2_SQ8_4                      ((uint32_t)0x00000200)        /* Bit 4 */

#define  ADC_SQR2_SQ9                        ((uint32_t)0x00007C00)        /* SQ9[4:0] bits (9th conversion in regular sequence) */
#define  ADC_SQR2_SQ9_0                      ((uint32_t)0x00000400)        /* Bit 0 */
#define  ADC_SQR2_SQ9_1                      ((uint32_t)0x00000800)        /* Bit 1 */
#define  ADC_SQR2_SQ9_2                      ((uint32_t)0x00001000)        /* Bit 2 */
#define  ADC_SQR2_SQ9_3                      ((uint32_t)0x00002000)        /* Bit 3 */
#define  ADC_SQR2_SQ9_4                      ((uint32_t)0x00004000)        /* Bit 4 */

#define  ADC_SQR2_SQ10                       ((uint32_t)0x000F8000)        /* SQ10[4:0] bits (10th conversion in regular sequence) */
#define  ADC_SQR2_SQ10_0                     ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_SQR2_SQ10_1                     ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_SQR2_SQ10_2                     ((uint32_t)0x00020000)        /* Bit 2 */
#define  ADC_SQR2_SQ10_3                     ((uint32_t)0x00040000)        /* Bit 3 */
#define  ADC_SQR2_SQ10_4                     ((uint32_t)0x00080000)        /* Bit 4 */

#define  ADC_SQR2_SQ11                       ((uint32_t)0x01F00000)        /* SQ11[4:0] bits (11th conversion in regular sequence) */
#define  ADC_SQR2_SQ11_0                     ((uint32_t)0x00100000)        /* Bit 0 */
#define  ADC_SQR2_SQ11_1                     ((uint32_t)0x00200000)        /* Bit 1 */
#define  ADC_SQR2_SQ11_2                     ((uint32_t)0x00400000)        /* Bit 2 */
#define  ADC_SQR2_SQ11_3                     ((uint32_t)0x00800000)        /* Bit 3 */
#define  ADC_SQR2_SQ11_4                     ((uint32_t)0x01000000)        /* Bit 4 */

#define  ADC_SQR2_SQ12                       ((uint32_t)0x3E000000)        /* SQ12[4:0] bits (12th conversion in regular sequence) */
#define  ADC_SQR2_SQ12_0                     ((uint32_t)0x02000000)        /* Bit 0 */
#define  ADC_SQR2_SQ12_1                     ((uint32_t)0x04000000)        /* Bit 1 */
#define  ADC_SQR2_SQ12_2                     ((uint32_t)0x08000000)        /* Bit 2 */
#define  ADC_SQR2_SQ12_3                     ((uint32_t)0x10000000)        /* Bit 3 */
#define  ADC_SQR2_SQ12_4                     ((uint32_t)0x20000000)        /* Bit 4 */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define  ADC_SQR3_SQ1                        ((uint32_t)0x0000001F)        /* SQ1[4:0] bits (1st conversion in regular sequence) */
#define  ADC_SQR3_SQ1_0                      ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_SQR3_SQ1_1                      ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_SQR3_SQ1_2                      ((uint32_t)0x00000004)        /* Bit 2 */
#define  ADC_SQR3_SQ1_3                      ((uint32_t)0x00000008)        /* Bit 3 */
#define  ADC_SQR3_SQ1_4                      ((uint32_t)0x00000010)        /* Bit 4 */

#define  ADC_SQR3_SQ2                        ((uint32_t)0x000003E0)        /* SQ2[4:0] bits (2nd conversion in regular sequence) */
#define  ADC_SQR3_SQ2_0                      ((uint32_t)0x00000020)        /* Bit 0 */
#define  ADC_SQR3_SQ2_1                      ((uint32_t)0x00000040)        /* Bit 1 */
#define  ADC_SQR3_SQ2_2                      ((uint32_t)0x00000080)        /* Bit 2 */
#define  ADC_SQR3_SQ2_3                      ((uint32_t)0x00000100)        /* Bit 3 */
#define  ADC_SQR3_SQ2_4                      ((uint32_t)0x00000200)        /* Bit 4 */

#define  ADC_SQR3_SQ3                        ((uint32_t)0x00007C00)        /* SQ3[4:0] bits (3rd conversion in regular sequence) */
#define  ADC_SQR3_SQ3_0                      ((uint32_t)0x00000400)        /* Bit 0 */
#define  ADC_SQR3_SQ3_1                      ((uint32_t)0x00000800)        /* Bit 1 */
#define  ADC_SQR3_SQ3_2                      ((uint32_t)0x00001000)        /* Bit 2 */
#define  ADC_SQR3_SQ3_3                      ((uint32_t)0x00002000)        /* Bit 3 */
#define  ADC_SQR3_SQ3_4                      ((uint32_t)0x00004000)        /* Bit 4 */

#define  ADC_SQR3_SQ4                        ((uint32_t)0x000F8000)        /* SQ4[4:0] bits (4th conversion in regular sequence) */
#define  ADC_SQR3_SQ4_0                      ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_SQR3_SQ4_1                      ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_SQR3_SQ4_2                      ((uint32_t)0x00020000)        /* Bit 2 */
#define  ADC_SQR3_SQ4_3                      ((uint32_t)0x00040000)        /* Bit 3 */
#define  ADC_SQR3_SQ4_4                      ((uint32_t)0x00080000)        /* Bit 4 */

#define  ADC_SQR3_SQ5                        ((uint32_t)0x01F00000)        /* SQ5[4:0] bits (5th conversion in regular sequence) */
#define  ADC_SQR3_SQ5_0                      ((uint32_t)0x00100000)        /* Bit 0 */
#define  ADC_SQR3_SQ5_1                      ((uint32_t)0x00200000)        /* Bit 1 */
#define  ADC_SQR3_SQ5_2                      ((uint32_t)0x00400000)        /* Bit 2 */
#define  ADC_SQR3_SQ5_3                      ((uint32_t)0x00800000)        /* Bit 3 */
#define  ADC_SQR3_SQ5_4                      ((uint32_t)0x01000000)        /* Bit 4 */

#define  ADC_SQR3_SQ6                        ((uint32_t)0x3E000000)        /* SQ6[4:0] bits (6th conversion in regular sequence) */
#define  ADC_SQR3_SQ6_0                      ((uint32_t)0x02000000)        /* Bit 0 */
#define  ADC_SQR3_SQ6_1                      ((uint32_t)0x04000000)        /* Bit 1 */
#define  ADC_SQR3_SQ6_2                      ((uint32_t)0x08000000)        /* Bit 2 */
#define  ADC_SQR3_SQ6_3                      ((uint32_t)0x10000000)        /* Bit 3 */
#define  ADC_SQR3_SQ6_4                      ((uint32_t)0x20000000)        /* Bit 4 */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define  ADC_JSQR_JSQ1                       ((uint32_t)0x0000001F)        /* JSQ1[4:0] bits (1st conversion in injected sequence) */  
#define  ADC_JSQR_JSQ1_0                     ((uint32_t)0x00000001)        /* Bit 0 */
#define  ADC_JSQR_JSQ1_1                     ((uint32_t)0x00000002)        /* Bit 1 */
#define  ADC_JSQR_JSQ1_2                     ((uint32_t)0x00000004)        /* Bit 2 */
#define  ADC_JSQR_JSQ1_3                     ((uint32_t)0x00000008)        /* Bit 3 */
#define  ADC_JSQR_JSQ1_4                     ((uint32_t)0x00000010)        /* Bit 4 */

#define  ADC_JSQR_JSQ2                       ((uint32_t)0x000003E0)        /* JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define  ADC_JSQR_JSQ2_0                     ((uint32_t)0x00000020)        /* Bit 0 */
#define  ADC_JSQR_JSQ2_1                     ((uint32_t)0x00000040)        /* Bit 1 */
#define  ADC_JSQR_JSQ2_2                     ((uint32_t)0x00000080)        /* Bit 2 */
#define  ADC_JSQR_JSQ2_3                     ((uint32_t)0x00000100)        /* Bit 3 */
#define  ADC_JSQR_JSQ2_4                     ((uint32_t)0x00000200)        /* Bit 4 */

#define  ADC_JSQR_JSQ3                       ((uint32_t)0x00007C00)        /* JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define  ADC_JSQR_JSQ3_0                     ((uint32_t)0x00000400)        /* Bit 0 */
#define  ADC_JSQR_JSQ3_1                     ((uint32_t)0x00000800)        /* Bit 1 */
#define  ADC_JSQR_JSQ3_2                     ((uint32_t)0x00001000)        /* Bit 2 */
#define  ADC_JSQR_JSQ3_3                     ((uint32_t)0x00002000)        /* Bit 3 */
#define  ADC_JSQR_JSQ3_4                     ((uint32_t)0x00004000)        /* Bit 4 */

#define  ADC_JSQR_JSQ4                       ((uint32_t)0x000F8000)        /* JSQ4[4:0] bits (4th conversion in injected sequence) */
#define  ADC_JSQR_JSQ4_0                     ((uint32_t)0x00008000)        /* Bit 0 */
#define  ADC_JSQR_JSQ4_1                     ((uint32_t)0x00010000)        /* Bit 1 */
#define  ADC_JSQR_JSQ4_2                     ((uint32_t)0x00020000)        /* Bit 2 */
#define  ADC_JSQR_JSQ4_3                     ((uint32_t)0x00040000)        /* Bit 3 */
#define  ADC_JSQR_JSQ4_4                     ((uint32_t)0x00080000)        /* Bit 4 */

#define  ADC_JSQR_JL                         ((uint32_t)0x00300000)        /* JL[1:0] bits (Injected Sequence length) */
#define  ADC_JSQR_JL_0                       ((uint32_t)0x00100000)        /* Bit 0 */
#define  ADC_JSQR_JL_1                       ((uint32_t)0x00200000)        /* Bit 1 */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define  ADC_JDR1_JDATA                      ((uint16_t)0xFFFF)            /* Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define  ADC_JDR2_JDATA                      ((uint16_t)0xFFFF)            /* Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define  ADC_JDR3_JDATA                      ((uint16_t)0xFFFF)            /* Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define  ADC_JDR4_JDATA                      ((uint16_t)0xFFFF)            /* Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define  ADC_DR_DATA                         ((uint32_t)0x0000FFFF)        /* Regular data */
#define  ADC_DR_ADC2DATA                     ((uint32_t)0xFFFF0000)        /* ADC2 data */

/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for DAC_CR register  ********************/
#define  DAC_CR_EN1                          ((uint32_t)0x00000001)        /* DAC channel1 enable */
#define  DAC_CR_BOFF1                        ((uint32_t)0x00000002)        /* DAC channel1 output buffer disable */
#define  DAC_CR_TEN1                         ((uint32_t)0x00000004)        /* DAC channel1 Trigger enable */

#define  DAC_CR_TSEL1                        ((uint32_t)0x00000038)        /* TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_CR_TSEL1_0                      ((uint32_t)0x00000008)        /* Bit 0 */
#define  DAC_CR_TSEL1_1                      ((uint32_t)0x00000010)        /* Bit 1 */
#define  DAC_CR_TSEL1_2                      ((uint32_t)0x00000020)        /* Bit 2 */

#define  DAC_CR_WAVE1                        ((uint32_t)0x000000C0)        /* WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE1_0                      ((uint32_t)0x00000040)        /* Bit 0 */
#define  DAC_CR_WAVE1_1                      ((uint32_t)0x00000080)        /* Bit 1 */

#define  DAC_CR_MAMP1                        ((uint32_t)0x00000F00)        /* MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_CR_MAMP1_0                      ((uint32_t)0x00000100)        /* Bit 0 */
#define  DAC_CR_MAMP1_1                      ((uint32_t)0x00000200)        /* Bit 1 */
#define  DAC_CR_MAMP1_2                      ((uint32_t)0x00000400)        /* Bit 2 */
#define  DAC_CR_MAMP1_3                      ((uint32_t)0x00000800)        /* Bit 3 */

#define  DAC_CR_DMAEN1                       ((uint32_t)0x00001000)        /* DAC channel1 DMA enable */
#define  DAC_CR_EN2                          ((uint32_t)0x00010000)        /* DAC channel2 enable */
#define  DAC_CR_BOFF2                        ((uint32_t)0x00020000)        /* DAC channel2 output buffer disable */
#define  DAC_CR_TEN2                         ((uint32_t)0x00040000)        /* DAC channel2 Trigger enable */

#define  DAC_CR_TSEL2                        ((uint32_t)0x00380000)        /* TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  DAC_CR_TSEL2_0                      ((uint32_t)0x00080000)        /* Bit 0 */
#define  DAC_CR_TSEL2_1                      ((uint32_t)0x00100000)        /* Bit 1 */
#define  DAC_CR_TSEL2_2                      ((uint32_t)0x00200000)        /* Bit 2 */

#define  DAC_CR_WAVE2                        ((uint32_t)0x00C00000)        /* WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE2_0                      ((uint32_t)0x00400000)        /* Bit 0 */
#define  DAC_CR_WAVE2_1                      ((uint32_t)0x00800000)        /* Bit 1 */

#define  DAC_CR_MAMP2                        ((uint32_t)0x0F000000)        /* MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define  DAC_CR_MAMP2_0                      ((uint32_t)0x01000000)        /* Bit 0 */
#define  DAC_CR_MAMP2_1                      ((uint32_t)0x02000000)        /* Bit 1 */
#define  DAC_CR_MAMP2_2                      ((uint32_t)0x04000000)        /* Bit 2 */
#define  DAC_CR_MAMP2_3                      ((uint32_t)0x08000000)        /* Bit 3 */

#define  DAC_CR_DMAEN2                       ((uint32_t)0x10000000)        /* DAC channel2 DMA enabled */

/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define  DAC_SWTRIGR_SWTRIG1                 ((uint8_t)0x01)               /* DAC channel1 software trigger */
#define  DAC_SWTRIGR_SWTRIG2                 ((uint8_t)0x02)               /* DAC channel2 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define  DAC_DHR12R1_DACC1DHR                ((uint16_t)0x0FFF)            /* DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define  DAC_DHR12L1_DACC1DHR                ((uint16_t)0xFFF0)            /* DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define  DAC_DHR8R1_DACC1DHR                 ((uint8_t)0xFF)               /* DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12R2 register  ******************/
#define  DAC_DHR12R2_DACC2DHR                ((uint16_t)0x0FFF)            /* DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L2 register  ******************/
#define  DAC_DHR12L2_DACC2DHR                ((uint16_t)0xFFF0)            /* DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R2 register  ******************/
#define  DAC_DHR8R2_DACC2DHR                 ((uint8_t)0xFF)               /* DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define  DAC_DHR12RD_DACC1DHR                ((uint32_t)0x00000FFF)        /* DAC channel1 12-bit Right aligned data */
#define  DAC_DHR12RD_DACC2DHR                ((uint32_t)0x0FFF0000)        /* DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define  DAC_DHR12LD_DACC1DHR                ((uint32_t)0x0000FFF0)        /* DAC channel1 12-bit Left aligned data */
#define  DAC_DHR12LD_DACC2DHR                ((uint32_t)0xFFF00000)        /* DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define  DAC_DHR8RD_DACC1DHR                 ((uint16_t)0x00FF)            /* DAC channel1 8-bit Right aligned data */
#define  DAC_DHR8RD_DACC2DHR                 ((uint16_t)0xFF00)            /* DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define  DAC_DOR1_DACC1DOR                   ((uint16_t)0x0FFF)            /* DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
#define  DAC_DOR2_DACC2DOR                   ((uint16_t)0x0FFF)            /* DAC channel2 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define  DAC_SR_DMAUDR1                      ((uint32_t)0x00002000)        /* DAC channel1 DMA underrun flag */
#define  DAC_SR_DMAUDR2                      ((uint32_t)0x20000000)        /* DAC channel2 DMA underrun flag */

/************************************************************************************************
 *                                                                                              *
 *                   Universal Synchronous Asynchronous Receiver Transmitter                    *
 *                                                                                              *
 ************************************************************************************************/

/* Bit definition for USART_SR register
 ************************************************************************************************/
#define  USART_SR_PE             ((uint16_t)0x0001)      ///< Parity Error
#define  USART_SR_FE             ((uint16_t)0x0002)      ///< Framing Error
#define  USART_SR_NE             ((uint16_t)0x0004)      ///< Noise Error Flag
#define  USART_SR_ORE            ((uint16_t)0x0008)      ///< OverRun Error
#define  USART_SR_IDLE           ((uint16_t)0x0010)      ///< IDLE line detected
#define  USART_SR_RXNE           ((uint16_t)0x0020)      ///< Read Data Register Not Empty
#define  USART_SR_TC             ((uint16_t)0x0040)      ///< Transmission Complete
#define  USART_SR_TXE            ((uint16_t)0x0080)      ///< Transmit Data Register Empty
#define  USART_SR_LBD            ((uint16_t)0x0100)      ///< LIN Break Detection Flag
#define  USART_SR_CTS            ((uint16_t)0x0200)      ///< CTS Flag

/* Bit definition for USART_DR register
 ************************************************************************************************/
#define  USART_DR_DR             ((uint16_t)0x01FF)      ///< Data value

#define  USART_BRR_DIV_Fraction  ((uint16_t)0x000F)      ///< Fraction of USARTDIV
#define  USART_BRR_DIV_Mantissa  ((uint16_t)0xFFF0)      ///< Mantissa of USARTDIV

#define  USART_BAUDRATE1200      ((uint32_t)1200)        ///< BaudRate 1200
#define  USART_BAUDRATE2400      ((uint32_t)2400)        ///< BaudRate 2400
#define  USART_BAUDRATE4800      ((uint32_t)4800)        ///< BaudRate 4800
#define  USART_BAUDRATE9600      ((uint32_t)9600)        ///< BaudRate 9600
#define  USART_BAUDRATE14400     ((uint32_t)14400)       ///< BaudRate 14400
#define  USART_BAUDRATE19200     ((uint32_t)19200)       ///< BaudRate 19200
#define  USART_BAUDRATE28800     ((uint32_t)28800)       ///< BaudRate 28800
#define  USART_BAUDRATE38400     ((uint32_t)38400)       ///< BaudRate 38400
#define  USART_BAUDRATE56000     ((uint32_t)56000)       ///< BaudRate 56000
#define  USART_BAUDRATE57600     ((uint32_t)57600)       ///< BaudRate 57600
#define  USART_BAUDRATE115200    ((uint32_t)115200)      ///< BaudRate 115200
#define  USART_BAUDRATE230400    ((uint32_t)230400)      ///< BaudRate 230400
#define  USART_BAUDRATE460800    ((uint32_t)460800)      ///< BaudRate 460800
#define  USART_BAUDRATE921600    ((uint32_t)921600)      ///< BaudRate 921600

/* Bit definition for USART_CR1 register
 ************************************************************************************************/
#define  USART_CR1_SBK           ((uint16_t)0x0001)      ///< Send Break
#define  USART_CR1_RWU           ((uint16_t)0x0002)      ///< Receiver wakeup
#define  USART_CR1_RE            ((uint16_t)0x0004)      ///< Receiver Enable
#define  USART_CR1_TE            ((uint16_t)0x0008)      ///< Transmitter Enable
#define  USART_CR1_IDLEIE        ((uint16_t)0x0010)      ///< IDLE Interrupt Enable
#define  USART_CR1_RXNEIE        ((uint16_t)0x0020)      ///< RXNE Interrupt Enable
#define  USART_CR1_TCIE          ((uint16_t)0x0040)      ///< Transmission Complete Interrupt Enable
#define  USART_CR1_TXEIE         ((uint16_t)0x0080)      ///< PE Interrupt Enable
#define  USART_CR1_PEIE          ((uint16_t)0x0100)      ///< PE Interrupt Enable
#define  USART_CR1_PS            ((uint16_t)0x0200)      ///< Parity Selection
#define  USART_CR1_PCE           ((uint16_t)0x0400)      ///< Parity Control Enable
#define  USART_CR1_WAKE          ((uint16_t)0x0800)      ///< Wakeup method
#define  USART_CR1_M             ((uint16_t)0x1000)      ///< Word length
#define  USART_CR1_UE            ((uint16_t)0x2000)      ///< USART Enable
#define  USART_CR1_OVER8         ((uint16_t)0x8000)      ///< USART Oversmapling 8-bits 

/* Bit definition for USART_CR2 register
 ************************************************************************************************/
#define  USART_CR2_ADD           ((uint16_t)0x000F)      ///< Address of the USART node
#define  USART_CR2_LBDL          ((uint16_t)0x0020)      ///< LIN Break Detection Length
#define  USART_CR2_LBDIE         ((uint16_t)0x0040)      ///< LIN Break Detection Interrupt Enable
#define  USART_CR2_LBCL          ((uint16_t)0x0100)      ///< Last Bit Clock pulse
#define  USART_CR2_CPHA          ((uint16_t)0x0200)      ///< Clock Phase
#define  USART_CR2_CPOL          ((uint16_t)0x0400)      ///< Clock Polarity
#define  USART_CR2_CLKEN         ((uint16_t)0x0800)      ///< Clock Enable
#define  USART_CR2_STOP          ((uint16_t)0x3000)      ///< STOP[1:0] bits (STOP bits)
#define  USART_CR2_LINEN         ((uint16_t)0x4000)      ///< LIN mode enable

/* Bit definition for USART_CR3 register
 ************************************************************************************************/
#define  USART_CR3_EIE           ((uint16_t)0x0001)      ///< Error Interrupt Enable
#define  USART_CR3_IREN          ((uint16_t)0x0002)      ///< IrDA mode Enable
#define  USART_CR3_IRLP          ((uint16_t)0x0004)      ///< IrDA Low-Power
#define  USART_CR3_HDSEL         ((uint16_t)0x0008)      ///< Half-Duplex Selection
#define  USART_CR3_NACK          ((uint16_t)0x0010)      ///< Smartcard NACK enable
#define  USART_CR3_SCEN          ((uint16_t)0x0020)      ///< Smartcard mode enable
#define  USART_CR3_DMAR          ((uint16_t)0x0040)      ///< DMA Enable Receiver
#define  USART_CR3_DMAT          ((uint16_t)0x0080)      ///< DMA Enable Transmitter
#define  USART_CR3_RTSE          ((uint16_t)0x0100)      ///< RTS Enable
#define  USART_CR3_CTSE          ((uint16_t)0x0200)      ///< CTS Enable
#define  USART_CR3_CTSIE         ((uint16_t)0x0400)      ///< CTS Interrupt Enable
#define  USART_CR3_ONEBIT        ((uint16_t)0x0800)      ///< One Bit method

/* Bit definition for USART_GTPR register
 ************************************************************************************************/
#define  USART_GTPR_PSC          ((uint16_t)0x00FF)      ///< PSC[7:0] bits (Prescaler value)
#define  USART_GTPR_GT           ((uint16_t)0xFF00)      ///< Guard time value

/************************************************************************************************
 *                                                                                              *
 *                                Serial Peripheral Interface                                   *
 *                                                                                              *
 ************************************************************************************************/

/* Bit definition for SPI_CR1 register
 ************************************************************************************************/
#define  SPI_CR1_CPHA            ((uint16_t)0x0001)      ///< Clock Phase
#define  SPI_CR1_CPOL            ((uint16_t)0x0002)      ///< Clock Polarity
#define  SPI_CR1_MSTR            ((uint16_t)0x0004)      ///< Master Selection
#define  SPI_CR1_BR              ((uint16_t)0x0038)      ///< BR[2:0] bits (Baud Rate Control)
#define  SPI_CR1_SPE             ((uint16_t)0x0040)      ///< SPI Enable
#define  SPI_CR1_LSBFIRST        ((uint16_t)0x0080)      ///< Frame Format
#define  SPI_CR1_SSI             ((uint16_t)0x0100)      ///< Internal slave select
#define  SPI_CR1_SSM             ((uint16_t)0x0200)      ///< Software slave management
#define  SPI_CR1_RXONLY          ((uint16_t)0x0400)      ///< Receive only
#define  SPI_CR1_DFF             ((uint16_t)0x0800)      ///< Data Frame Format
#define  SPI_CR1_CRCNEXT         ((uint16_t)0x1000)      ///< Transmit CRC next
#define  SPI_CR1_CRCEN           ((uint16_t)0x2000)      ///< Hardware CRC calculation enable
#define  SPI_CR1_BIDIOE          ((uint16_t)0x4000)      ///< Output enable in bidirectional mode
#define  SPI_CR1_BIDIMODE        ((uint16_t)0x8000)      ///< Bidirectional data mode enable

/* Bit definition for SPI_CR2 register
 ************************************************************************************************/
#define  SPI_CR2_RXDMAEN         ((uint8_t)0x01)         ///< Rx Buffer DMA Enable
#define  SPI_CR2_TXDMAEN         ((uint8_t)0x02)         ///< Tx Buffer DMA Enable
#define  SPI_CR2_SSOE            ((uint8_t)0x04)         ///< SS Output Enable
#define  SPI_CR2_ERRIE           ((uint8_t)0x20)         ///< Error Interrupt Enable
#define  SPI_CR2_RXNEIE          ((uint8_t)0x40)         ///< RX buffer Not Empty Interrupt Enable
#define  SPI_CR2_TXEIE           ((uint8_t)0x80)         ///< Tx buffer Empty Interrupt Enable

/* Bit definition for SPI_SR register
 ************************************************************************************************/
#define  SPI_SR_RXNE             ((uint8_t)0x01)         ///< Receive buffer Not Empty
#define  SPI_SR_TXE              ((uint8_t)0x02)         ///< Transmit buffer Empty
#define  SPI_SR_CHSIDE           ((uint8_t)0x04)         ///< Channel side
#define  SPI_SR_UDR              ((uint8_t)0x08)         ///< Underrun flag
#define  SPI_SR_CRCERR           ((uint8_t)0x10)         ///< CRC Error flag
#define  SPI_SR_MODF             ((uint8_t)0x20)         ///< Mode fault
#define  SPI_SR_OVR              ((uint8_t)0x40)         ///< Overrun flag
#define  SPI_SR_BSY              ((uint8_t)0x80)         ///< Busy flag

/* Bit definition for SPI_DR register
 ************************************************************************************************/
#define  SPI_DR_DR               ((uint16_t)0xFFFF)      ///< Data Register

/* Bit definition for SPI_CRCPR register
 ************************************************************************************************/
#define  SPI_CRCPR_CRCPOLY       ((uint16_t)0xFFFF)      ///< CRC polynomial register

/* Bit definition for SPI_RXCRCR register
 ************************************************************************************************/
#define  SPI_RXCRCR_RXCRC        ((uint16_t)0xFFFF)      ///< Rx CRC Register

/* Bit definition for SPI_TXCRCR register
 ************************************************************************************************/
#define  SPI_TXCRCR_TXCRC        ((uint16_t)0xFFFF)      ///< Tx CRC Register

/* Bit definition for SPI_I2SCFGR register
 ************************************************************************************************/
#define  SPI_I2SCFGR_CHLEN       ((uint16_t)0x0001)      ///< Channel length (number of bits per audio channel)
#define  SPI_I2SCFGR_DATLEN      ((uint16_t)0x0006)      ///< DATLEN[1:0] bits (Data length to be transferred)
#define  SPI_I2SCFGR_CKPOL       ((uint16_t)0x0008)      ///< steady state clock polarity
#define  SPI_I2SCFGR_I2SSTD      ((uint16_t)0x0030)      ///< I2SSTD[1:0] bits (I2S standard selection)
#define  SPI_I2SCFGR_PCMSYNC     ((uint16_t)0x0080)      ///< PCM frame synchronization */
#define  SPI_I2SCFGR_I2SCFG      ((uint16_t)0x0300)      ///< I2SCFG[1:0] bits (I2S configuration mode)
#define  SPI_I2SCFGR_I2SE        ((uint16_t)0x0400)      ///< I2S Enable
#define  SPI_I2SCFGR_I2SMOD      ((uint16_t)0x0800)      ///< I2S mode selection

/* Bit definition for SPI_I2SPR register
 ************************************************************************************************/
#define  SPI_I2SPR_I2SDIV        ((uint16_t)0x00FF)      ///< I2S Linear prescaler
#define  SPI_I2SPR_ODD           ((uint16_t)0x0100)      ///< Odd factor for the prescaler
#define  SPI_I2SPR_MCKOE         ((uint16_t)0x0200)      ///< Master Clock Output Enable

/************************************************************************************************
 *                                                                                              *
 *                            Inter-integrated Circuit Interface                                *
 *                                                                                              *
 ************************************************************************************************/

/* Bit definition for I2C_CR1 register
 ************************************************************************************************/
#define  I2C_CR1_PE              ((uint16_t)0x0001)      ///< Peripheral Enable
#define  I2C_CR1_SMBUS           ((uint16_t)0x0002)      ///< SMBus Mode
#define  I2C_CR1_SMBTYPE         ((uint16_t)0x0008)      ///< SMBus Type
#define  I2C_CR1_ENARP           ((uint16_t)0x0010)      ///< ARP Enable
#define  I2C_CR1_ENPEC           ((uint16_t)0x0020)      ///< PEC Enable
#define  I2C_CR1_ENGC            ((uint16_t)0x0040)      ///< General Call Enable
#define  I2C_CR1_NOSTRETCH       ((uint16_t)0x0080)      ///< Clock Stretching Disable (Slave mode)
#define  I2C_CR1_START           ((uint16_t)0x0100)      ///< Start Generation
#define  I2C_CR1_STOP            ((uint16_t)0x0200)      ///< Stop Generation
#define  I2C_CR1_ACK             ((uint16_t)0x0400)      ///< Acknowledge Enable
#define  I2C_CR1_POS             ((uint16_t)0x0800)      ///< Acknowledge/PEC Position (for data reception)
#define  I2C_CR1_PEC             ((uint16_t)0x1000)      ///< Packet Error Checking
#define  I2C_CR1_ALERT           ((uint16_t)0x2000)      ///< SMBus Alert
#define  I2C_CR1_SWRST           ((uint16_t)0x8000)      ///< Software Reset

/* Bit definition for I2C_CR2 register
 ************************************************************************************************/
#define  I2C_CR2_FREQ            ((uint16_t)0x003F)      ///< FREQ[5:0] bits (Peripheral Clock Frequency)
#define  I2C_CR2_ITERREN         ((uint16_t)0x0100)      ///< Error Interrupt Enable
#define  I2C_CR2_ITEVTEN         ((uint16_t)0x0200)      ///< Event Interrupt Enable
#define  I2C_CR2_ITBUFEN         ((uint16_t)0x0400)      ///< Buffer Interrupt Enable
#define  I2C_CR2_DMAEN           ((uint16_t)0x0800)      ///< DMA Requests Enable
#define  I2C_CR2_LAST            ((uint16_t)0x1000)      ///< DMA Last Transfer

/* Bit definition for I2C_OAR1 register
 ************************************************************************************************/
#define  I2C_OAR1_ADD1_7         ((uint16_t)0x00FE)      ///< Interface Address
#define  I2C_OAR1_ADD8_9         ((uint16_t)0x0300)      ///< Interface Address
#define  I2C_OAR1_ADDMODE        ((uint16_t)0x8000)      ///< Addressing Mode (Slave mode)

/* Bit definition for I2C_OAR2 register
 ************************************************************************************************/
#define  I2C_OAR2_ENDUAL         ((uint8_t)0x01)         ///< Dual addressing mode enable
#define  I2C_OAR2_ADD2           ((uint8_t)0xFE)         ///< Interface address

/* Bit definition for I2C_DR register
 ************************************************************************************************/
#define  I2C_DR_DR               ((uint8_t)0xFF)         ///< 8-bit Data Register

/* Bit definition for I2C_SR1 register
 ************************************************************************************************/
#define  I2C_SR1_SB              ((uint16_t)0x0001)      ///< Start Bit (Master mode)
#define  I2C_SR1_ADDR            ((uint16_t)0x0002)      ///< Address sent (master mode)/matched (slave mode)
#define  I2C_SR1_BTF             ((uint16_t)0x0004)      ///< Byte Transfer Finished
#define  I2C_SR1_ADD10           ((uint16_t)0x0008)      ///< 10-bit header sent (Master mode)
#define  I2C_SR1_STOPF           ((uint16_t)0x0010)      ///< Stop detection (Slave mode)
#define  I2C_SR1_RXNE            ((uint16_t)0x0040)      ///< Data Register not Empty (receivers)
#define  I2C_SR1_TXE             ((uint16_t)0x0080)      ///< Data Register Empty (transmitters)
#define  I2C_SR1_BERR            ((uint16_t)0x0100)      ///< Bus Error
#define  I2C_SR1_ARLO            ((uint16_t)0x0200)      ///< Arbitration Lost (master mode)
#define  I2C_SR1_AF              ((uint16_t)0x0400)      ///< Acknowledge Failure
#define  I2C_SR1_OVR             ((uint16_t)0x0800)      ///< Overrun/Underrun
#define  I2C_SR1_PECERR          ((uint16_t)0x1000)      ///< PEC Error in reception
#define  I2C_SR1_TIMEOUT         ((uint16_t)0x4000)      ///< Timeout or Tlow Error
#define  I2C_SR1_SMBALERT        ((uint16_t)0x8000)      ///< SMBus Alert

/* Bit definition for I2C_SR2 register
 ************************************************************************************************/
#define  I2C_SR2_MSL             ((uint16_t)0x0001)      ///< Master/Slave
#define  I2C_SR2_BUSY            ((uint16_t)0x0002)      ///< Bus Busy
#define  I2C_SR2_TRA             ((uint16_t)0x0004)      ///< Transmitter/Receiver
#define  I2C_SR2_GENCALL         ((uint16_t)0x0010)      ///< General Call Address (Slave mode)
#define  I2C_SR2_SMBDEFAULT      ((uint16_t)0x0020)      ///< SMBus Device Default Address (Slave mode)
#define  I2C_SR2_SMBHOST         ((uint16_t)0x0040)      ///< SMBus Host Header (Slave mode)
#define  I2C_SR2_DUALF           ((uint16_t)0x0080)      ///< Dual Flag (Slave mode)
#define  I2C_SR2_PEC             ((uint16_t)0xFF00)      ///< Packet Error Checking Register

/* Bit definition for I2C_CCR register
 ************************************************************************************************/
#define  I2C_CCR_CCR             ((uint16_t)0x0FFF)      ///< Clock Control Register in Fast/Standard mode (Master mode)
#define  I2C_CCR_DUTY            ((uint16_t)0x4000)      ///< Fast Mode Duty Cycle
#define  I2C_CCR_FS              ((uint16_t)0x8000)      ///< I2C Master Mode Selection

/* Bit definition for I2C_TRISE register
 ************************************************************************************************/
#define  I2C_TRISE_TRISE         ((uint8_t)0x3F)         ///< Maximum Rise Time in Fast/Standard mode (Master mode)


/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for TIM_CR1 register  ********************/
#define  TIM_CR1_CEN                         ((uint16_t)0x0001)            /* Counter enable */
#define  TIM_CR1_UDIS                        ((uint16_t)0x0002)            /* Update disable */
#define  TIM_CR1_URS                         ((uint16_t)0x0004)            /* Update request source */
#define  TIM_CR1_OPM                         ((uint16_t)0x0008)            /* One pulse mode */
#define  TIM_CR1_DIR                         ((uint16_t)0x0010)            /* Direction */

#define  TIM_CR1_CMS                         ((uint16_t)0x0060)            /* CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CR1_CMS_0                       ((uint16_t)0x0020)            /* Bit 0 */
#define  TIM_CR1_CMS_1                       ((uint16_t)0x0040)            /* Bit 1 */

#define  TIM_CR1_ARPE                        ((uint16_t)0x0080)            /* Auto-reload preload enable */

#define  TIM_CR1_CKD                         ((uint16_t)0x0300)            /* CKD[1:0] bits (clock division) */
#define  TIM_CR1_CKD_0                       ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_CR1_CKD_1                       ((uint16_t)0x0200)            /* Bit 1 */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define  TIM_CR2_CCPC                        ((uint16_t)0x0001)            /* Capture/Compare Preloaded Control */
#define  TIM_CR2_CCUS                        ((uint16_t)0x0004)            /* Capture/Compare Control Update Selection */
#define  TIM_CR2_CCDS                        ((uint16_t)0x0008)            /* Capture/Compare DMA Selection */

#define  TIM_CR2_MMS                         ((uint16_t)0x0070)            /* MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS_0                       ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_CR2_MMS_1                       ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_CR2_MMS_2                       ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_CR2_TI1S                        ((uint16_t)0x0080)            /* TI1 Selection */
#define  TIM_CR2_OIS1                        ((uint16_t)0x0100)            /* Output Idle state 1 (OC1 output) */
#define  TIM_CR2_OIS1N                       ((uint16_t)0x0200)            /* Output Idle state 1 (OC1N output) */
#define  TIM_CR2_OIS2                        ((uint16_t)0x0400)            /* Output Idle state 2 (OC2 output) */
#define  TIM_CR2_OIS2N                       ((uint16_t)0x0800)            /* Output Idle state 2 (OC2N output) */
#define  TIM_CR2_OIS3                        ((uint16_t)0x1000)            /* Output Idle state 3 (OC3 output) */
#define  TIM_CR2_OIS3N                       ((uint16_t)0x2000)            /* Output Idle state 3 (OC3N output) */
#define  TIM_CR2_OIS4                        ((uint16_t)0x4000)            /* Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define  TIM_SMCR_SMS                        ((uint16_t)0x0007)            /* SMS[2:0] bits (Slave mode selection) */
#define  TIM_SMCR_SMS_0                      ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_SMCR_SMS_1                      ((uint16_t)0x0002)            /* Bit 1 */
#define  TIM_SMCR_SMS_2                      ((uint16_t)0x0004)            /* Bit 2 */

#define  TIM_SMCR_TS                         ((uint16_t)0x0070)            /* TS[2:0] bits (Trigger selection) */
#define  TIM_SMCR_TS_0                       ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_SMCR_TS_1                       ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_SMCR_TS_2                       ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_SMCR_MSM                        ((uint16_t)0x0080)            /* Master/slave mode */

#define  TIM_SMCR_ETF                        ((uint16_t)0x0F00)            /* ETF[3:0] bits (External trigger filter) */
#define  TIM_SMCR_ETF_0                      ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_SMCR_ETF_1                      ((uint16_t)0x0200)            /* Bit 1 */
#define  TIM_SMCR_ETF_2                      ((uint16_t)0x0400)            /* Bit 2 */
#define  TIM_SMCR_ETF_3                      ((uint16_t)0x0800)            /* Bit 3 */

#define  TIM_SMCR_ETPS                       ((uint16_t)0x3000)            /* ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_SMCR_ETPS_0                     ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_SMCR_ETPS_1                     ((uint16_t)0x2000)            /* Bit 1 */

#define  TIM_SMCR_ECE                        ((uint16_t)0x4000)            /* External clock enable */
#define  TIM_SMCR_ETP                        ((uint16_t)0x8000)            /* External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define  TIM_DIER_UIE                        ((uint16_t)0x0001)            /* Update interrupt enable */
#define  TIM_DIER_CC1IE                      ((uint16_t)0x0002)            /* Capture/Compare 1 interrupt enable */
#define  TIM_DIER_CC2IE                      ((uint16_t)0x0004)            /* Capture/Compare 2 interrupt enable */
#define  TIM_DIER_CC3IE                      ((uint16_t)0x0008)            /* Capture/Compare 3 interrupt enable */
#define  TIM_DIER_CC4IE                      ((uint16_t)0x0010)            /* Capture/Compare 4 interrupt enable */
#define  TIM_DIER_COMIE                      ((uint16_t)0x0020)            /* COM interrupt enable */
#define  TIM_DIER_TIE                        ((uint16_t)0x0040)            /* Trigger interrupt enable */
#define  TIM_DIER_BIE                        ((uint16_t)0x0080)            /* Break interrupt enable */
#define  TIM_DIER_UDE                        ((uint16_t)0x0100)            /* Update DMA request enable */
#define  TIM_DIER_CC1DE                      ((uint16_t)0x0200)            /* Capture/Compare 1 DMA request enable */
#define  TIM_DIER_CC2DE                      ((uint16_t)0x0400)            /* Capture/Compare 2 DMA request enable */
#define  TIM_DIER_CC3DE                      ((uint16_t)0x0800)            /* Capture/Compare 3 DMA request enable */
#define  TIM_DIER_CC4DE                      ((uint16_t)0x1000)            /* Capture/Compare 4 DMA request enable */
#define  TIM_DIER_COMDE                      ((uint16_t)0x2000)            /* COM DMA request enable */
#define  TIM_DIER_TDE                        ((uint16_t)0x4000)            /* Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  ********************/
#define  TIM_SR_UIF                          ((uint16_t)0x0001)            /* Update interrupt Flag */
#define  TIM_SR_CC1IF                        ((uint16_t)0x0002)            /* Capture/Compare 1 interrupt Flag */
#define  TIM_SR_CC2IF                        ((uint16_t)0x0004)            /* Capture/Compare 2 interrupt Flag */
#define  TIM_SR_CC3IF                        ((uint16_t)0x0008)            /* Capture/Compare 3 interrupt Flag */
#define  TIM_SR_CC4IF                        ((uint16_t)0x0010)            /* Capture/Compare 4 interrupt Flag */
#define  TIM_SR_COMIF                        ((uint16_t)0x0020)            /* COM interrupt Flag */
#define  TIM_SR_TIF                          ((uint16_t)0x0040)            /* Trigger interrupt Flag */
#define  TIM_SR_BIF                          ((uint16_t)0x0080)            /* Break interrupt Flag */
#define  TIM_SR_CC1OF                        ((uint16_t)0x0200)            /* Capture/Compare 1 Overcapture Flag */
#define  TIM_SR_CC2OF                        ((uint16_t)0x0400)            /* Capture/Compare 2 Overcapture Flag */
#define  TIM_SR_CC3OF                        ((uint16_t)0x0800)            /* Capture/Compare 3 Overcapture Flag */
#define  TIM_SR_CC4OF                        ((uint16_t)0x1000)            /* Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define  TIM_EGR_UG                          ((uint8_t)0x01)               /* Update Generation */
#define  TIM_EGR_CC1G                        ((uint8_t)0x02)               /* Capture/Compare 1 Generation */
#define  TIM_EGR_CC2G                        ((uint8_t)0x04)               /* Capture/Compare 2 Generation */
#define  TIM_EGR_CC3G                        ((uint8_t)0x08)               /* Capture/Compare 3 Generation */
#define  TIM_EGR_CC4G                        ((uint8_t)0x10)               /* Capture/Compare 4 Generation */
#define  TIM_EGR_COMG                        ((uint8_t)0x20)               /* Capture/Compare Control Update Generation */
#define  TIM_EGR_TG                          ((uint8_t)0x40)               /* Trigger Generation */
#define  TIM_EGR_BG                          ((uint8_t)0x80)               /* Break Generation */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  TIM_CCMR1_CC1S                      ((uint16_t)0x0003)            /* CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CCMR1_CC1S_0                    ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_CCMR1_CC1S_1                    ((uint16_t)0x0002)            /* Bit 1 */

#define  TIM_CCMR1_OC1FE                     ((uint16_t)0x0004)            /* Output Compare 1 Fast enable */
#define  TIM_CCMR1_OC1PE                     ((uint16_t)0x0008)            /* Output Compare 1 Preload enable */

#define  TIM_CCMR1_OC1M                      ((uint16_t)0x0070)            /* OC1M[2:0] bits (Output Compare 1 Mode) */
#define  TIM_CCMR1_OC1M_0                    ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_CCMR1_OC1M_1                    ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_CCMR1_OC1M_2                    ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_CCMR1_OC1CE                     ((uint16_t)0x0080)            /* Output Compare 1Clear Enable */

#define  TIM_CCMR1_CC2S                      ((uint16_t)0x0300)            /* CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CCMR1_CC2S_0                    ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_CCMR1_CC2S_1                    ((uint16_t)0x0200)            /* Bit 1 */

#define  TIM_CCMR1_OC2FE                     ((uint16_t)0x0400)            /* Output Compare 2 Fast enable */
#define  TIM_CCMR1_OC2PE                     ((uint16_t)0x0800)            /* Output Compare 2 Preload enable */

#define  TIM_CCMR1_OC2M                      ((uint16_t)0x7000)            /* OC2M[2:0] bits (Output Compare 2 Mode) */
#define  TIM_CCMR1_OC2M_0                    ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_CCMR1_OC2M_1                    ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_CCMR1_OC2M_2                    ((uint16_t)0x4000)            /* Bit 2 */

#define  TIM_CCMR1_OC2CE                     ((uint16_t)0x8000)            /* Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR1_IC1PSC                    ((uint16_t)0x000C)            /* IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_CCMR1_IC1PSC_0                  ((uint16_t)0x0004)            /* Bit 0 */
#define  TIM_CCMR1_IC1PSC_1                  ((uint16_t)0x0008)            /* Bit 1 */

#define  TIM_CCMR1_IC1F                      ((uint16_t)0x00F0)            /* IC1F[3:0] bits (Input Capture 1 Filter) */
#define  TIM_CCMR1_IC1F_0                    ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_CCMR1_IC1F_1                    ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_CCMR1_IC1F_2                    ((uint16_t)0x0040)            /* Bit 2 */
#define  TIM_CCMR1_IC1F_3                    ((uint16_t)0x0080)            /* Bit 3 */

#define  TIM_CCMR1_IC2PSC                    ((uint16_t)0x0C00)            /* IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define  TIM_CCMR1_IC2PSC_0                  ((uint16_t)0x0400)            /* Bit 0 */
#define  TIM_CCMR1_IC2PSC_1                  ((uint16_t)0x0800)            /* Bit 1 */

#define  TIM_CCMR1_IC2F                      ((uint16_t)0xF000)            /* IC2F[3:0] bits (Input Capture 2 Filter) */
#define  TIM_CCMR1_IC2F_0                    ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_CCMR1_IC2F_1                    ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_CCMR1_IC2F_2                    ((uint16_t)0x4000)            /* Bit 2 */
#define  TIM_CCMR1_IC2F_3                    ((uint16_t)0x8000)            /* Bit 3 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  TIM_CCMR2_CC3S                      ((uint16_t)0x0003)            /* CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define  TIM_CCMR2_CC3S_0                    ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_CCMR2_CC3S_1                    ((uint16_t)0x0002)            /* Bit 1 */

#define  TIM_CCMR2_OC3FE                     ((uint16_t)0x0004)            /* Output Compare 3 Fast enable */
#define  TIM_CCMR2_OC3PE                     ((uint16_t)0x0008)            /* Output Compare 3 Preload enable */

#define  TIM_CCMR2_OC3M                      ((uint16_t)0x0070)            /* OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_CCMR2_OC3M_0                    ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_CCMR2_OC3M_1                    ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_CCMR2_OC3M_2                    ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_CCMR2_OC3CE                     ((uint16_t)0x0080)            /* Output Compare 3 Clear Enable */

#define  TIM_CCMR2_CC4S                      ((uint16_t)0x0300)            /* CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CCMR2_CC4S_0                    ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_CCMR2_CC4S_1                    ((uint16_t)0x0200)            /* Bit 1 */

#define  TIM_CCMR2_OC4FE                     ((uint16_t)0x0400)            /* Output Compare 4 Fast enable */
#define  TIM_CCMR2_OC4PE                     ((uint16_t)0x0800)            /* Output Compare 4 Preload enable */

#define  TIM_CCMR2_OC4M                      ((uint16_t)0x7000)            /* OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR2_OC4M_0                    ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_CCMR2_OC4M_1                    ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_CCMR2_OC4M_2                    ((uint16_t)0x4000)            /* Bit 2 */

#define  TIM_CCMR2_OC4CE                     ((uint16_t)0x8000)            /* Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR2_IC3PSC                    ((uint16_t)0x000C)            /* IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_CCMR2_IC3PSC_0                  ((uint16_t)0x0004)            /* Bit 0 */
#define  TIM_CCMR2_IC3PSC_1                  ((uint16_t)0x0008)            /* Bit 1 */

#define  TIM_CCMR2_IC3F                      ((uint16_t)0x00F0)            /* IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_CCMR2_IC3F_0                    ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_CCMR2_IC3F_1                    ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_CCMR2_IC3F_2                    ((uint16_t)0x0040)            /* Bit 2 */
#define  TIM_CCMR2_IC3F_3                    ((uint16_t)0x0080)            /* Bit 3 */

#define  TIM_CCMR2_IC4PSC                    ((uint16_t)0x0C00)            /* IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_CCMR2_IC4PSC_0                  ((uint16_t)0x0400)            /* Bit 0 */
#define  TIM_CCMR2_IC4PSC_1                  ((uint16_t)0x0800)            /* Bit 1 */

#define  TIM_CCMR2_IC4F                      ((uint16_t)0xF000)            /* IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_CCMR2_IC4F_0                    ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_CCMR2_IC4F_1                    ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_CCMR2_IC4F_2                    ((uint16_t)0x4000)            /* Bit 2 */
#define  TIM_CCMR2_IC4F_3                    ((uint16_t)0x8000)            /* Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CCER_CC1E                       ((uint16_t)0x0001)            /* Capture/Compare 1 output enable */
#define  TIM_CCER_CC1P                       ((uint16_t)0x0002)            /* Capture/Compare 1 output Polarity */
#define  TIM_CCER_CC1NE                      ((uint16_t)0x0004)            /* Capture/Compare 1 Complementary output enable */
#define  TIM_CCER_CC1NP                      ((uint16_t)0x0008)            /* Capture/Compare 1 Complementary output Polarity */
#define  TIM_CCER_CC2E                       ((uint16_t)0x0010)            /* Capture/Compare 2 output enable */
#define  TIM_CCER_CC2P                       ((uint16_t)0x0020)            /* Capture/Compare 2 output Polarity */
#define  TIM_CCER_CC2NE                      ((uint16_t)0x0040)            /* Capture/Compare 2 Complementary output enable */
#define  TIM_CCER_CC2NP                      ((uint16_t)0x0080)            /* Capture/Compare 2 Complementary output Polarity */
#define  TIM_CCER_CC3E                       ((uint16_t)0x0100)            /* Capture/Compare 3 output enable */
#define  TIM_CCER_CC3P                       ((uint16_t)0x0200)            /* Capture/Compare 3 output Polarity */
#define  TIM_CCER_CC3NE                      ((uint16_t)0x0400)            /* Capture/Compare 3 Complementary output enable */
#define  TIM_CCER_CC3NP                      ((uint16_t)0x0800)            /* Capture/Compare 3 Complementary output Polarity */
#define  TIM_CCER_CC4E                       ((uint16_t)0x1000)            /* Capture/Compare 4 output enable */
#define  TIM_CCER_CC4P                       ((uint16_t)0x2000)            /* Capture/Compare 4 output Polarity */
#define  TIM_CCER_CC4NP                      ((uint16_t)0x8000)            /* Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT_CNT                         ((uint16_t)0xFFFF)            /* Counter Value */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC_PSC                         ((uint16_t)0xFFFF)            /* Prescaler Value */

/*******************  Bit definition for TIM_ARR register  ********************/
#define  TIM_ARR_ARR                         ((uint16_t)0xFFFF)            /* actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define  TIM_RCR_REP                         ((uint8_t)0xFF)               /* Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define  TIM_CCR1_CCR1                       ((uint16_t)0xFFFF)            /* Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define  TIM_CCR2_CCR2                       ((uint16_t)0xFFFF)            /* Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define  TIM_CCR3_CCR3                       ((uint16_t)0xFFFF)            /* Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define  TIM_CCR4_CCR4                       ((uint16_t)0xFFFF)            /* Capture/Compare 4 Value */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_BDTR_DTG                        ((uint16_t)0x00FF)            /* DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_BDTR_DTG_0                      ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_BDTR_DTG_1                      ((uint16_t)0x0002)            /* Bit 1 */
#define  TIM_BDTR_DTG_2                      ((uint16_t)0x0004)            /* Bit 2 */
#define  TIM_BDTR_DTG_3                      ((uint16_t)0x0008)            /* Bit 3 */
#define  TIM_BDTR_DTG_4                      ((uint16_t)0x0010)            /* Bit 4 */
#define  TIM_BDTR_DTG_5                      ((uint16_t)0x0020)            /* Bit 5 */
#define  TIM_BDTR_DTG_6                      ((uint16_t)0x0040)            /* Bit 6 */
#define  TIM_BDTR_DTG_7                      ((uint16_t)0x0080)            /* Bit 7 */

#define  TIM_BDTR_LOCK                       ((uint16_t)0x0300)            /* LOCK[1:0] bits (Lock Configuration) */
#define  TIM_BDTR_LOCK_0                     ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_BDTR_LOCK_1                     ((uint16_t)0x0200)            /* Bit 1 */

#define  TIM_BDTR_OSSI                       ((uint16_t)0x0400)            /* Off-State Selection for Idle mode */
#define  TIM_BDTR_OSSR                       ((uint16_t)0x0800)            /* Off-State Selection for Run mode */
#define  TIM_BDTR_BKE                        ((uint16_t)0x1000)            /* Break enable */
#define  TIM_BDTR_BKP                        ((uint16_t)0x2000)            /* Break Polarity */
#define  TIM_BDTR_AOE                        ((uint16_t)0x4000)            /* Automatic Output enable */
#define  TIM_BDTR_MOE                        ((uint16_t)0x8000)            /* Main Output enable */

/*******************  Bit definition for TIM_DCR register  ********************/
#define  TIM_DCR_DBA                         ((uint16_t)0x001F)            /* DBA[4:0] bits (DMA Base Address) */
#define  TIM_DCR_DBA_0                       ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_DCR_DBA_1                       ((uint16_t)0x0002)            /* Bit 1 */
#define  TIM_DCR_DBA_2                       ((uint16_t)0x0004)            /* Bit 2 */
#define  TIM_DCR_DBA_3                       ((uint16_t)0x0008)            /* Bit 3 */
#define  TIM_DCR_DBA_4                       ((uint16_t)0x0010)            /* Bit 4 */

#define  TIM_DCR_DBL                         ((uint16_t)0x1F00)            /* DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DCR_DBL_0                       ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_DCR_DBL_1                       ((uint16_t)0x0200)            /* Bit 1 */
#define  TIM_DCR_DBL_2                       ((uint16_t)0x0400)            /* Bit 2 */
#define  TIM_DCR_DBL_3                       ((uint16_t)0x0800)            /* Bit 3 */
#define  TIM_DCR_DBL_4                       ((uint16_t)0x1000)            /* Bit 4 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define  TIM_DMAR_DMAB                       ((uint16_t)0xFFFF)            /* DMA register for burst accesses */

/******************************************************************************/
/*                                                                            */
/*                       Flexible Static Memory Controller                    */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for FSMC_BCR1 register  *******************/
#define  FSMC_BCR1_MBKEN                     ((uint32_t)0x00000001)        /* Memory bank enable bit */
#define  FSMC_BCR1_MUXEN                     ((uint32_t)0x00000002)        /* Address/data multiplexing enable bit */

#define  FSMC_BCR1_MTYP                      ((uint32_t)0x0000000C)        /* MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR1_MTYP_0                    ((uint32_t)0x00000004)        /* Bit 0 */
#define  FSMC_BCR1_MTYP_1                    ((uint32_t)0x00000008)        /* Bit 1 */

#define  FSMC_BCR1_MWID                      ((uint32_t)0x00000030)        /* MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR1_MWID_0                    ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BCR1_MWID_1                    ((uint32_t)0x00000020)        /* Bit 1 */

#define  FSMC_BCR1_FACCEN                    ((uint32_t)0x00000040)        /* Flash access enable */
#define  FSMC_BCR1_BURSTEN                   ((uint32_t)0x00000100)        /* Burst enable bit */
#define  FSMC_BCR1_WAITPOL                   ((uint32_t)0x00000200)        /* Wait signal polarity bit */
#define  FSMC_BCR1_WRAPMOD                   ((uint32_t)0x00000400)        /* Wrapped burst mode support */
#define  FSMC_BCR1_WAITCFG                   ((uint32_t)0x00000800)        /* Wait timing configuration */
#define  FSMC_BCR1_WREN                      ((uint32_t)0x00001000)        /* Write enable bit */
#define  FSMC_BCR1_WAITEN                    ((uint32_t)0x00002000)        /* Wait enable bit */
#define  FSMC_BCR1_EXTMOD                    ((uint32_t)0x00004000)        /* Extended mode enable */
#define  FSMC_BCR1_ASYNCWAIT                 ((uint32_t)0x00008000)       /* Asynchronous wait */
#define  FSMC_BCR1_CBURSTRW                  ((uint32_t)0x00080000)        /* Write burst enable */

/******************  Bit definition for FSMC_BCR2 register  *******************/
#define  FSMC_BCR2_MBKEN                     ((uint32_t)0x00000001)        /* Memory bank enable bit */
#define  FSMC_BCR2_MUXEN                     ((uint32_t)0x00000002)        /* Address/data multiplexing enable bit */

#define  FSMC_BCR2_MTYP                      ((uint32_t)0x0000000C)        /* MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR2_MTYP_0                    ((uint32_t)0x00000004)        /* Bit 0 */
#define  FSMC_BCR2_MTYP_1                    ((uint32_t)0x00000008)        /* Bit 1 */

#define  FSMC_BCR2_MWID                      ((uint32_t)0x00000030)        /* MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR2_MWID_0                    ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BCR2_MWID_1                    ((uint32_t)0x00000020)        /* Bit 1 */

#define  FSMC_BCR2_FACCEN                    ((uint32_t)0x00000040)        /* Flash access enable */
#define  FSMC_BCR2_BURSTEN                   ((uint32_t)0x00000100)        /* Burst enable bit */
#define  FSMC_BCR2_WAITPOL                   ((uint32_t)0x00000200)        /* Wait signal polarity bit */
#define  FSMC_BCR2_WRAPMOD                   ((uint32_t)0x00000400)        /* Wrapped burst mode support */
#define  FSMC_BCR2_WAITCFG                   ((uint32_t)0x00000800)        /* Wait timing configuration */
#define  FSMC_BCR2_WREN                      ((uint32_t)0x00001000)        /* Write enable bit */
#define  FSMC_BCR2_WAITEN                    ((uint32_t)0x00002000)        /* Wait enable bit */
#define  FSMC_BCR2_EXTMOD                    ((uint32_t)0x00004000)        /* Extended mode enable */
#define  FSMC_BCR2_ASYNCWAIT                 ((uint32_t)0x00008000)       /* Asynchronous wait */
#define  FSMC_BCR2_CBURSTRW                  ((uint32_t)0x00080000)        /* Write burst enable */

/******************  Bit definition for FSMC_BCR3 register  *******************/
#define  FSMC_BCR3_MBKEN                     ((uint32_t)0x00000001)        /* Memory bank enable bit */
#define  FSMC_BCR3_MUXEN                     ((uint32_t)0x00000002)        /* Address/data multiplexing enable bit */

#define  FSMC_BCR3_MTYP                      ((uint32_t)0x0000000C)        /* MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR3_MTYP_0                    ((uint32_t)0x00000004)        /* Bit 0 */
#define  FSMC_BCR3_MTYP_1                    ((uint32_t)0x00000008)        /* Bit 1 */

#define  FSMC_BCR3_MWID                      ((uint32_t)0x00000030)        /* MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR3_MWID_0                    ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BCR3_MWID_1                    ((uint32_t)0x00000020)        /* Bit 1 */

#define  FSMC_BCR3_FACCEN                    ((uint32_t)0x00000040)        /* Flash access enable */
#define  FSMC_BCR3_BURSTEN                   ((uint32_t)0x00000100)        /* Burst enable bit */
#define  FSMC_BCR3_WAITPOL                   ((uint32_t)0x00000200)        /* Wait signal polarity bit. */
#define  FSMC_BCR3_WRAPMOD                   ((uint32_t)0x00000400)        /* Wrapped burst mode support */
#define  FSMC_BCR3_WAITCFG                   ((uint32_t)0x00000800)        /* Wait timing configuration */
#define  FSMC_BCR3_WREN                      ((uint32_t)0x00001000)        /* Write enable bit */
#define  FSMC_BCR3_WAITEN                    ((uint32_t)0x00002000)        /* Wait enable bit */
#define  FSMC_BCR3_EXTMOD                    ((uint32_t)0x00004000)        /* Extended mode enable */
#define  FSMC_BCR3_ASYNCWAIT                 ((uint32_t)0x00008000)       /* Asynchronous wait */
#define  FSMC_BCR3_CBURSTRW                  ((uint32_t)0x00080000)        /* Write burst enable */

/******************  Bit definition for FSMC_BCR4 register  *******************/
#define  FSMC_BCR4_MBKEN                     ((uint32_t)0x00000001)        /* Memory bank enable bit */
#define  FSMC_BCR4_MUXEN                     ((uint32_t)0x00000002)        /* Address/data multiplexing enable bit */

#define  FSMC_BCR4_MTYP                      ((uint32_t)0x0000000C)        /* MTYP[1:0] bits (Memory type) */
#define  FSMC_BCR4_MTYP_0                    ((uint32_t)0x00000004)        /* Bit 0 */
#define  FSMC_BCR4_MTYP_1                    ((uint32_t)0x00000008)        /* Bit 1 */

#define  FSMC_BCR4_MWID                      ((uint32_t)0x00000030)        /* MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR4_MWID_0                    ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BCR4_MWID_1                    ((uint32_t)0x00000020)        /* Bit 1 */

#define  FSMC_BCR4_FACCEN                    ((uint32_t)0x00000040)        /* Flash access enable */
#define  FSMC_BCR4_BURSTEN                   ((uint32_t)0x00000100)        /* Burst enable bit */
#define  FSMC_BCR4_WAITPOL                   ((uint32_t)0x00000200)        /* Wait signal polarity bit */
#define  FSMC_BCR4_WRAPMOD                   ((uint32_t)0x00000400)        /* Wrapped burst mode support */
#define  FSMC_BCR4_WAITCFG                   ((uint32_t)0x00000800)        /* Wait timing configuration */
#define  FSMC_BCR4_WREN                      ((uint32_t)0x00001000)        /* Write enable bit */
#define  FSMC_BCR4_WAITEN                    ((uint32_t)0x00002000)        /* Wait enable bit */
#define  FSMC_BCR4_EXTMOD                    ((uint32_t)0x00004000)        /* Extended mode enable */
#define  FSMC_BCR4_ASYNCWAIT                 ((uint32_t)0x00008000)       /* Asynchronous wait */
#define  FSMC_BCR4_CBURSTRW                  ((uint32_t)0x00080000)        /* Write burst enable */

/******************  Bit definition for FSMC_BTR1 register  ******************/
#define  FSMC_BTR1_ADDSET                    ((uint32_t)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR1_ADDSET_0                  ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_BTR1_ADDSET_1                  ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_BTR1_ADDSET_2                  ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_BTR1_ADDSET_3                  ((uint32_t)0x00000008)        /* Bit 3 */

#define  FSMC_BTR1_ADDHLD                    ((uint32_t)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR1_ADDHLD_0                  ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BTR1_ADDHLD_1                  ((uint32_t)0x00000020)        /* Bit 1 */
#define  FSMC_BTR1_ADDHLD_2                  ((uint32_t)0x00000040)        /* Bit 2 */
#define  FSMC_BTR1_ADDHLD_3                  ((uint32_t)0x00000080)        /* Bit 3 */

#define  FSMC_BTR1_DATAST                    ((uint32_t)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR1_DATAST_0                  ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_BTR1_DATAST_1                  ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_BTR1_DATAST_2                  ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_BTR1_DATAST_3                  ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_BTR1_DATAST_4                  ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_BTR1_DATAST_5                  ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_BTR1_DATAST_6                  ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_BTR1_DATAST_7                  ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_BTR1_BUSTURN                   ((uint32_t)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR1_BUSTURN_0                 ((uint32_t)0x00010000)        /* Bit 0 */
#define  FSMC_BTR1_BUSTURN_1                 ((uint32_t)0x00020000)        /* Bit 1 */
#define  FSMC_BTR1_BUSTURN_2                 ((uint32_t)0x00040000)        /* Bit 2 */
#define  FSMC_BTR1_BUSTURN_3                 ((uint32_t)0x00080000)        /* Bit 3 */

#define  FSMC_BTR1_CLKDIV                    ((uint32_t)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR1_CLKDIV_0                  ((uint32_t)0x00100000)        /* Bit 0 */
#define  FSMC_BTR1_CLKDIV_1                  ((uint32_t)0x00200000)        /* Bit 1 */
#define  FSMC_BTR1_CLKDIV_2                  ((uint32_t)0x00400000)        /* Bit 2 */
#define  FSMC_BTR1_CLKDIV_3                  ((uint32_t)0x00800000)        /* Bit 3 */

#define  FSMC_BTR1_DATLAT                    ((uint32_t)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR1_DATLAT_0                  ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_BTR1_DATLAT_1                  ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_BTR1_DATLAT_2                  ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_BTR1_DATLAT_3                  ((uint32_t)0x08000000)        /* Bit 3 */

#define  FSMC_BTR1_ACCMOD                    ((uint32_t)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR1_ACCMOD_0                  ((uint32_t)0x10000000)        /* Bit 0 */
#define  FSMC_BTR1_ACCMOD_1                  ((uint32_t)0x20000000)        /* Bit 1 */

/******************  Bit definition for FSMC_BTR2 register  *******************/
#define  FSMC_BTR2_ADDSET                    ((uint32_t)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR2_ADDSET_0                  ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_BTR2_ADDSET_1                  ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_BTR2_ADDSET_2                  ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_BTR2_ADDSET_3                  ((uint32_t)0x00000008)        /* Bit 3 */

#define  FSMC_BTR2_ADDHLD                    ((uint32_t)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR2_ADDHLD_0                  ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BTR2_ADDHLD_1                  ((uint32_t)0x00000020)        /* Bit 1 */
#define  FSMC_BTR2_ADDHLD_2                  ((uint32_t)0x00000040)        /* Bit 2 */
#define  FSMC_BTR2_ADDHLD_3                  ((uint32_t)0x00000080)        /* Bit 3 */

#define  FSMC_BTR2_DATAST                    ((uint32_t)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR2_DATAST_0                  ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_BTR2_DATAST_1                  ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_BTR2_DATAST_2                  ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_BTR2_DATAST_3                  ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_BTR2_DATAST_4                  ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_BTR2_DATAST_5                  ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_BTR2_DATAST_6                  ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_BTR2_DATAST_7                  ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_BTR2_BUSTURN                   ((uint32_t)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR2_BUSTURN_0                 ((uint32_t)0x00010000)        /* Bit 0 */
#define  FSMC_BTR2_BUSTURN_1                 ((uint32_t)0x00020000)        /* Bit 1 */
#define  FSMC_BTR2_BUSTURN_2                 ((uint32_t)0x00040000)        /* Bit 2 */
#define  FSMC_BTR2_BUSTURN_3                 ((uint32_t)0x00080000)        /* Bit 3 */

#define  FSMC_BTR2_CLKDIV                    ((uint32_t)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR2_CLKDIV_0                  ((uint32_t)0x00100000)        /* Bit 0 */
#define  FSMC_BTR2_CLKDIV_1                  ((uint32_t)0x00200000)        /* Bit 1 */
#define  FSMC_BTR2_CLKDIV_2                  ((uint32_t)0x00400000)        /* Bit 2 */
#define  FSMC_BTR2_CLKDIV_3                  ((uint32_t)0x00800000)        /* Bit 3 */

#define  FSMC_BTR2_DATLAT                    ((uint32_t)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR2_DATLAT_0                  ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_BTR2_DATLAT_1                  ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_BTR2_DATLAT_2                  ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_BTR2_DATLAT_3                  ((uint32_t)0x08000000)        /* Bit 3 */

#define  FSMC_BTR2_ACCMOD                    ((uint32_t)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR2_ACCMOD_0                  ((uint32_t)0x10000000)        /* Bit 0 */
#define  FSMC_BTR2_ACCMOD_1                  ((uint32_t)0x20000000)        /* Bit 1 */

/*******************  Bit definition for FSMC_BTR3 register  *******************/
#define  FSMC_BTR3_ADDSET                    ((uint32_t)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR3_ADDSET_0                  ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_BTR3_ADDSET_1                  ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_BTR3_ADDSET_2                  ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_BTR3_ADDSET_3                  ((uint32_t)0x00000008)        /* Bit 3 */

#define  FSMC_BTR3_ADDHLD                    ((uint32_t)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR3_ADDHLD_0                  ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BTR3_ADDHLD_1                  ((uint32_t)0x00000020)        /* Bit 1 */
#define  FSMC_BTR3_ADDHLD_2                  ((uint32_t)0x00000040)        /* Bit 2 */
#define  FSMC_BTR3_ADDHLD_3                  ((uint32_t)0x00000080)        /* Bit 3 */

#define  FSMC_BTR3_DATAST                    ((uint32_t)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR3_DATAST_0                  ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_BTR3_DATAST_1                  ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_BTR3_DATAST_2                  ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_BTR3_DATAST_3                  ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_BTR3_DATAST_4                  ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_BTR3_DATAST_5                  ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_BTR3_DATAST_6                  ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_BTR3_DATAST_7                  ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_BTR3_BUSTURN                   ((uint32_t)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR3_BUSTURN_0                 ((uint32_t)0x00010000)        /* Bit 0 */
#define  FSMC_BTR3_BUSTURN_1                 ((uint32_t)0x00020000)        /* Bit 1 */
#define  FSMC_BTR3_BUSTURN_2                 ((uint32_t)0x00040000)        /* Bit 2 */
#define  FSMC_BTR3_BUSTURN_3                 ((uint32_t)0x00080000)        /* Bit 3 */

#define  FSMC_BTR3_CLKDIV                    ((uint32_t)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR3_CLKDIV_0                  ((uint32_t)0x00100000)        /* Bit 0 */
#define  FSMC_BTR3_CLKDIV_1                  ((uint32_t)0x00200000)        /* Bit 1 */
#define  FSMC_BTR3_CLKDIV_2                  ((uint32_t)0x00400000)        /* Bit 2 */
#define  FSMC_BTR3_CLKDIV_3                  ((uint32_t)0x00800000)        /* Bit 3 */

#define  FSMC_BTR3_DATLAT                    ((uint32_t)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR3_DATLAT_0                  ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_BTR3_DATLAT_1                  ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_BTR3_DATLAT_2                  ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_BTR3_DATLAT_3                  ((uint32_t)0x08000000)        /* Bit 3 */

#define  FSMC_BTR3_ACCMOD                    ((uint32_t)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR3_ACCMOD_0                  ((uint32_t)0x10000000)        /* Bit 0 */
#define  FSMC_BTR3_ACCMOD_1                  ((uint32_t)0x20000000)        /* Bit 1 */

/******************  Bit definition for FSMC_BTR4 register  *******************/
#define  FSMC_BTR4_ADDSET                    ((uint32_t)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR4_ADDSET_0                  ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_BTR4_ADDSET_1                  ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_BTR4_ADDSET_2                  ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_BTR4_ADDSET_3                  ((uint32_t)0x00000008)        /* Bit 3 */

#define  FSMC_BTR4_ADDHLD                    ((uint32_t)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR4_ADDHLD_0                  ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BTR4_ADDHLD_1                  ((uint32_t)0x00000020)        /* Bit 1 */
#define  FSMC_BTR4_ADDHLD_2                  ((uint32_t)0x00000040)        /* Bit 2 */
#define  FSMC_BTR4_ADDHLD_3                  ((uint32_t)0x00000080)        /* Bit 3 */

#define  FSMC_BTR4_DATAST                    ((uint32_t)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR4_DATAST_0                  ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_BTR4_DATAST_1                  ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_BTR4_DATAST_2                  ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_BTR4_DATAST_3                  ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_BTR4_DATAST_4                  ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_BTR4_DATAST_5                  ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_BTR4_DATAST_6                  ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_BTR4_DATAST_7                  ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_BTR4_BUSTURN                   ((uint32_t)0x000F0000)        /* BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR4_BUSTURN_0                 ((uint32_t)0x00010000)        /* Bit 0 */
#define  FSMC_BTR4_BUSTURN_1                 ((uint32_t)0x00020000)        /* Bit 1 */
#define  FSMC_BTR4_BUSTURN_2                 ((uint32_t)0x00040000)        /* Bit 2 */
#define  FSMC_BTR4_BUSTURN_3                 ((uint32_t)0x00080000)        /* Bit 3 */

#define  FSMC_BTR4_CLKDIV                    ((uint32_t)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR4_CLKDIV_0                  ((uint32_t)0x00100000)        /* Bit 0 */
#define  FSMC_BTR4_CLKDIV_1                  ((uint32_t)0x00200000)        /* Bit 1 */
#define  FSMC_BTR4_CLKDIV_2                  ((uint32_t)0x00400000)        /* Bit 2 */
#define  FSMC_BTR4_CLKDIV_3                  ((uint32_t)0x00800000)        /* Bit 3 */

#define  FSMC_BTR4_DATLAT                    ((uint32_t)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR4_DATLAT_0                  ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_BTR4_DATLAT_1                  ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_BTR4_DATLAT_2                  ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_BTR4_DATLAT_3                  ((uint32_t)0x08000000)        /* Bit 3 */

#define  FSMC_BTR4_ACCMOD                    ((uint32_t)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR4_ACCMOD_0                  ((uint32_t)0x10000000)        /* Bit 0 */
#define  FSMC_BTR4_ACCMOD_1                  ((uint32_t)0x20000000)        /* Bit 1 */

/******************  Bit definition for FSMC_BWTR1 register  ******************/
#define  FSMC_BWTR1_ADDSET                   ((uint32_t)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR1_ADDSET_0                 ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_BWTR1_ADDSET_1                 ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_BWTR1_ADDSET_2                 ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_BWTR1_ADDSET_3                 ((uint32_t)0x00000008)        /* Bit 3 */

#define  FSMC_BWTR1_ADDHLD                   ((uint32_t)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR1_ADDHLD_0                 ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BWTR1_ADDHLD_1                 ((uint32_t)0x00000020)        /* Bit 1 */
#define  FSMC_BWTR1_ADDHLD_2                 ((uint32_t)0x00000040)        /* Bit 2 */
#define  FSMC_BWTR1_ADDHLD_3                 ((uint32_t)0x00000080)        /* Bit 3 */

#define  FSMC_BWTR1_DATAST                   ((uint32_t)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR1_DATAST_0                 ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_BWTR1_DATAST_1                 ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_BWTR1_DATAST_2                 ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_BWTR1_DATAST_3                 ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_BWTR1_DATAST_4                 ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_BWTR1_DATAST_5                 ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_BWTR1_DATAST_6                 ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_BWTR1_DATAST_7                 ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_BWTR1_CLKDIV                   ((uint32_t)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR1_CLKDIV_0                 ((uint32_t)0x00100000)        /* Bit 0 */
#define  FSMC_BWTR1_CLKDIV_1                 ((uint32_t)0x00200000)        /* Bit 1 */
#define  FSMC_BWTR1_CLKDIV_2                 ((uint32_t)0x00400000)        /* Bit 2 */
#define  FSMC_BWTR1_CLKDIV_3                 ((uint32_t)0x00800000)        /* Bit 3 */

#define  FSMC_BWTR1_DATLAT                   ((uint32_t)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR1_DATLAT_0                 ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_BWTR1_DATLAT_1                 ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_BWTR1_DATLAT_2                 ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_BWTR1_DATLAT_3                 ((uint32_t)0x08000000)        /* Bit 3 */

#define  FSMC_BWTR1_ACCMOD                   ((uint32_t)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR1_ACCMOD_0                 ((uint32_t)0x10000000)        /* Bit 0 */
#define  FSMC_BWTR1_ACCMOD_1                 ((uint32_t)0x20000000)        /* Bit 1 */

/******************  Bit definition for FSMC_BWTR2 register  ******************/
#define  FSMC_BWTR2_ADDSET                   ((uint32_t)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR2_ADDSET_0                 ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_BWTR2_ADDSET_1                 ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_BWTR2_ADDSET_2                 ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_BWTR2_ADDSET_3                 ((uint32_t)0x00000008)        /* Bit 3 */

#define  FSMC_BWTR2_ADDHLD                   ((uint32_t)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR2_ADDHLD_0                 ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BWTR2_ADDHLD_1                 ((uint32_t)0x00000020)        /* Bit 1 */
#define  FSMC_BWTR2_ADDHLD_2                 ((uint32_t)0x00000040)        /* Bit 2 */
#define  FSMC_BWTR2_ADDHLD_3                 ((uint32_t)0x00000080)        /* Bit 3 */

#define  FSMC_BWTR2_DATAST                   ((uint32_t)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR2_DATAST_0                 ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_BWTR2_DATAST_1                 ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_BWTR2_DATAST_2                 ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_BWTR2_DATAST_3                 ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_BWTR2_DATAST_4                 ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_BWTR2_DATAST_5                 ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_BWTR2_DATAST_6                 ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_BWTR2_DATAST_7                 ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_BWTR2_CLKDIV                   ((uint32_t)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR2_CLKDIV_0                 ((uint32_t)0x00100000)        /* Bit 0 */
#define  FSMC_BWTR2_CLKDIV_1                 ((uint32_t)0x00200000)        /* Bit 1*/
#define  FSMC_BWTR2_CLKDIV_2                 ((uint32_t)0x00400000)        /* Bit 2 */
#define  FSMC_BWTR2_CLKDIV_3                 ((uint32_t)0x00800000)        /* Bit 3 */

#define  FSMC_BWTR2_DATLAT                   ((uint32_t)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR2_DATLAT_0                 ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_BWTR2_DATLAT_1                 ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_BWTR2_DATLAT_2                 ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_BWTR2_DATLAT_3                 ((uint32_t)0x08000000)        /* Bit 3 */

#define  FSMC_BWTR2_ACCMOD                   ((uint32_t)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR2_ACCMOD_0                 ((uint32_t)0x10000000)        /* Bit 0 */
#define  FSMC_BWTR2_ACCMOD_1                 ((uint32_t)0x20000000)        /* Bit 1 */

/******************  Bit definition for FSMC_BWTR3 register  ******************/
#define  FSMC_BWTR3_ADDSET                   ((uint32_t)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR3_ADDSET_0                 ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_BWTR3_ADDSET_1                 ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_BWTR3_ADDSET_2                 ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_BWTR3_ADDSET_3                 ((uint32_t)0x00000008)        /* Bit 3 */

#define  FSMC_BWTR3_ADDHLD                   ((uint32_t)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR3_ADDHLD_0                 ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BWTR3_ADDHLD_1                 ((uint32_t)0x00000020)        /* Bit 1 */
#define  FSMC_BWTR3_ADDHLD_2                 ((uint32_t)0x00000040)        /* Bit 2 */
#define  FSMC_BWTR3_ADDHLD_3                 ((uint32_t)0x00000080)        /* Bit 3 */

#define  FSMC_BWTR3_DATAST                   ((uint32_t)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR3_DATAST_0                 ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_BWTR3_DATAST_1                 ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_BWTR3_DATAST_2                 ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_BWTR3_DATAST_3                 ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_BWTR3_DATAST_4                 ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_BWTR3_DATAST_5                 ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_BWTR3_DATAST_6                 ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_BWTR3_DATAST_7                 ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_BWTR3_CLKDIV                   ((uint32_t)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR3_CLKDIV_0                 ((uint32_t)0x00100000)        /* Bit 0 */
#define  FSMC_BWTR3_CLKDIV_1                 ((uint32_t)0x00200000)        /* Bit 1 */
#define  FSMC_BWTR3_CLKDIV_2                 ((uint32_t)0x00400000)        /* Bit 2 */
#define  FSMC_BWTR3_CLKDIV_3                 ((uint32_t)0x00800000)        /* Bit 3 */

#define  FSMC_BWTR3_DATLAT                   ((uint32_t)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR3_DATLAT_0                 ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_BWTR3_DATLAT_1                 ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_BWTR3_DATLAT_2                 ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_BWTR3_DATLAT_3                 ((uint32_t)0x08000000)        /* Bit 3 */

#define  FSMC_BWTR3_ACCMOD                   ((uint32_t)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR3_ACCMOD_0                 ((uint32_t)0x10000000)        /* Bit 0 */
#define  FSMC_BWTR3_ACCMOD_1                 ((uint32_t)0x20000000)        /* Bit 1 */

/******************  Bit definition for FSMC_BWTR4 register  ******************/
#define  FSMC_BWTR4_ADDSET                   ((uint32_t)0x0000000F)        /* ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR4_ADDSET_0                 ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_BWTR4_ADDSET_1                 ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_BWTR4_ADDSET_2                 ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_BWTR4_ADDSET_3                 ((uint32_t)0x00000008)        /* Bit 3 */

#define  FSMC_BWTR4_ADDHLD                   ((uint32_t)0x000000F0)        /* ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR4_ADDHLD_0                 ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_BWTR4_ADDHLD_1                 ((uint32_t)0x00000020)        /* Bit 1 */
#define  FSMC_BWTR4_ADDHLD_2                 ((uint32_t)0x00000040)        /* Bit 2 */
#define  FSMC_BWTR4_ADDHLD_3                 ((uint32_t)0x00000080)        /* Bit 3 */

#define  FSMC_BWTR4_DATAST                   ((uint32_t)0x0000FF00)        /* DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR4_DATAST_0                 ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_BWTR4_DATAST_1                 ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_BWTR4_DATAST_2                 ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_BWTR4_DATAST_3                 ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_BWTR4_DATAST_4                 ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_BWTR4_DATAST_5                 ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_BWTR4_DATAST_6                 ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_BWTR4_DATAST_7                 ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_BWTR4_CLKDIV                   ((uint32_t)0x00F00000)        /* CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR4_CLKDIV_0                 ((uint32_t)0x00100000)        /* Bit 0 */
#define  FSMC_BWTR4_CLKDIV_1                 ((uint32_t)0x00200000)        /* Bit 1 */
#define  FSMC_BWTR4_CLKDIV_2                 ((uint32_t)0x00400000)        /* Bit 2 */
#define  FSMC_BWTR4_CLKDIV_3                 ((uint32_t)0x00800000)        /* Bit 3 */

#define  FSMC_BWTR4_DATLAT                   ((uint32_t)0x0F000000)        /* DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR4_DATLAT_0                 ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_BWTR4_DATLAT_1                 ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_BWTR4_DATLAT_2                 ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_BWTR4_DATLAT_3                 ((uint32_t)0x08000000)        /* Bit 3 */

#define  FSMC_BWTR4_ACCMOD                   ((uint32_t)0x30000000)        /* ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR4_ACCMOD_0                 ((uint32_t)0x10000000)        /* Bit 0 */
#define  FSMC_BWTR4_ACCMOD_1                 ((uint32_t)0x20000000)        /* Bit 1 */

/******************  Bit definition for FSMC_PCR2 register  *******************/
#define  FSMC_PCR2_PWAITEN                   ((uint32_t)0x00000002)        /* Wait feature enable bit */
#define  FSMC_PCR2_PBKEN                     ((uint32_t)0x00000004)        /* PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR2_PTYP                      ((uint32_t)0x00000008)        /* Memory type */

#define  FSMC_PCR2_PWID                      ((uint32_t)0x00000030)        /* PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR2_PWID_0                    ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_PCR2_PWID_1                    ((uint32_t)0x00000020)        /* Bit 1 */

#define  FSMC_PCR2_ECCEN                     ((uint32_t)0x00000040)        /* ECC computation logic enable bit */

#define  FSMC_PCR2_TCLR                      ((uint32_t)0x00001E00)        /* TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR2_TCLR_0                    ((uint32_t)0x00000200)        /* Bit 0 */
#define  FSMC_PCR2_TCLR_1                    ((uint32_t)0x00000400)        /* Bit 1 */
#define  FSMC_PCR2_TCLR_2                    ((uint32_t)0x00000800)        /* Bit 2 */
#define  FSMC_PCR2_TCLR_3                    ((uint32_t)0x00001000)        /* Bit 3 */

#define  FSMC_PCR2_TAR                       ((uint32_t)0x0001E000)        /* TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR2_TAR_0                     ((uint32_t)0x00002000)        /* Bit 0 */
#define  FSMC_PCR2_TAR_1                     ((uint32_t)0x00004000)        /* Bit 1 */
#define  FSMC_PCR2_TAR_2                     ((uint32_t)0x00008000)        /* Bit 2 */
#define  FSMC_PCR2_TAR_3                     ((uint32_t)0x00010000)        /* Bit 3 */

#define  FSMC_PCR2_ECCPS                     ((uint32_t)0x000E0000)        /* ECCPS[1:0] bits (ECC page size) */
#define  FSMC_PCR2_ECCPS_0                   ((uint32_t)0x00020000)        /* Bit 0 */
#define  FSMC_PCR2_ECCPS_1                   ((uint32_t)0x00040000)        /* Bit 1 */
#define  FSMC_PCR2_ECCPS_2                   ((uint32_t)0x00080000)        /* Bit 2 */

/******************  Bit definition for FSMC_PCR3 register  *******************/
#define  FSMC_PCR3_PWAITEN                   ((uint32_t)0x00000002)        /* Wait feature enable bit */
#define  FSMC_PCR3_PBKEN                     ((uint32_t)0x00000004)        /* PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR3_PTYP                      ((uint32_t)0x00000008)        /* Memory type */

#define  FSMC_PCR3_PWID                      ((uint32_t)0x00000030)        /* PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR3_PWID_0                    ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_PCR3_PWID_1                    ((uint32_t)0x00000020)        /* Bit 1 */

#define  FSMC_PCR3_ECCEN                     ((uint32_t)0x00000040)        /* ECC computation logic enable bit */

#define  FSMC_PCR3_TCLR                      ((uint32_t)0x00001E00)        /* TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR3_TCLR_0                    ((uint32_t)0x00000200)        /* Bit 0 */
#define  FSMC_PCR3_TCLR_1                    ((uint32_t)0x00000400)        /* Bit 1 */
#define  FSMC_PCR3_TCLR_2                    ((uint32_t)0x00000800)        /* Bit 2 */
#define  FSMC_PCR3_TCLR_3                    ((uint32_t)0x00001000)        /* Bit 3 */

#define  FSMC_PCR3_TAR                       ((uint32_t)0x0001E000)        /* TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR3_TAR_0                     ((uint32_t)0x00002000)        /* Bit 0 */
#define  FSMC_PCR3_TAR_1                     ((uint32_t)0x00004000)        /* Bit 1 */
#define  FSMC_PCR3_TAR_2                     ((uint32_t)0x00008000)        /* Bit 2 */
#define  FSMC_PCR3_TAR_3                     ((uint32_t)0x00010000)        /* Bit 3 */

#define  FSMC_PCR3_ECCPS                     ((uint32_t)0x000E0000)        /* ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR3_ECCPS_0                   ((uint32_t)0x00020000)        /* Bit 0 */
#define  FSMC_PCR3_ECCPS_1                   ((uint32_t)0x00040000)        /* Bit 1 */
#define  FSMC_PCR3_ECCPS_2                   ((uint32_t)0x00080000)        /* Bit 2 */

/******************  Bit definition for FSMC_PCR4 register  *******************/
#define  FSMC_PCR4_PWAITEN                   ((uint32_t)0x00000002)        /* Wait feature enable bit */
#define  FSMC_PCR4_PBKEN                     ((uint32_t)0x00000004)        /* PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR4_PTYP                      ((uint32_t)0x00000008)        /* Memory type */

#define  FSMC_PCR4_PWID                      ((uint32_t)0x00000030)        /* PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR4_PWID_0                    ((uint32_t)0x00000010)        /* Bit 0 */
#define  FSMC_PCR4_PWID_1                    ((uint32_t)0x00000020)        /* Bit 1 */

#define  FSMC_PCR4_ECCEN                     ((uint32_t)0x00000040)        /* ECC computation logic enable bit */

#define  FSMC_PCR4_TCLR                      ((uint32_t)0x00001E00)        /* TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR4_TCLR_0                    ((uint32_t)0x00000200)        /* Bit 0 */
#define  FSMC_PCR4_TCLR_1                    ((uint32_t)0x00000400)        /* Bit 1 */
#define  FSMC_PCR4_TCLR_2                    ((uint32_t)0x00000800)        /* Bit 2 */
#define  FSMC_PCR4_TCLR_3                    ((uint32_t)0x00001000)        /* Bit 3 */

#define  FSMC_PCR4_TAR                       ((uint32_t)0x0001E000)        /* TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR4_TAR_0                     ((uint32_t)0x00002000)        /* Bit 0 */
#define  FSMC_PCR4_TAR_1                     ((uint32_t)0x00004000)        /* Bit 1 */
#define  FSMC_PCR4_TAR_2                     ((uint32_t)0x00008000)        /* Bit 2 */
#define  FSMC_PCR4_TAR_3                     ((uint32_t)0x00010000)        /* Bit 3 */

#define  FSMC_PCR4_ECCPS                     ((uint32_t)0x000E0000)        /* ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR4_ECCPS_0                   ((uint32_t)0x00020000)        /* Bit 0 */
#define  FSMC_PCR4_ECCPS_1                   ((uint32_t)0x00040000)        /* Bit 1 */
#define  FSMC_PCR4_ECCPS_2                   ((uint32_t)0x00080000)        /* Bit 2 */

/*******************  Bit definition for FSMC_SR2 register  *******************/
#define  FSMC_SR2_IRS                        ((uint8_t)0x01)               /* Interrupt Rising Edge status */
#define  FSMC_SR2_ILS                        ((uint8_t)0x02)               /* Interrupt Level status */
#define  FSMC_SR2_IFS                        ((uint8_t)0x04)               /* Interrupt Falling Edge status */
#define  FSMC_SR2_IREN                       ((uint8_t)0x08)               /* Interrupt Rising Edge detection Enable bit */
#define  FSMC_SR2_ILEN                       ((uint8_t)0x10)               /* Interrupt Level detection Enable bit */
#define  FSMC_SR2_IFEN                       ((uint8_t)0x20)               /* Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR2_FEMPT                      ((uint8_t)0x40)               /* FIFO empty */

/*******************  Bit definition for FSMC_SR3 register  *******************/
#define  FSMC_SR3_IRS                        ((uint8_t)0x01)               /* Interrupt Rising Edge status */
#define  FSMC_SR3_ILS                        ((uint8_t)0x02)               /* Interrupt Level status */
#define  FSMC_SR3_IFS                        ((uint8_t)0x04)               /* Interrupt Falling Edge status */
#define  FSMC_SR3_IREN                       ((uint8_t)0x08)               /* Interrupt Rising Edge detection Enable bit */
#define  FSMC_SR3_ILEN                       ((uint8_t)0x10)               /* Interrupt Level detection Enable bit */
#define  FSMC_SR3_IFEN                       ((uint8_t)0x20)               /* Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR3_FEMPT                      ((uint8_t)0x40)               /* FIFO empty */

/*******************  Bit definition for FSMC_SR4 register  *******************/
#define  FSMC_SR4_IRS                        ((uint8_t)0x01)               /* Interrupt Rising Edge status */
#define  FSMC_SR4_ILS                        ((uint8_t)0x02)               /* Interrupt Level status */
#define  FSMC_SR4_IFS                        ((uint8_t)0x04)               /* Interrupt Falling Edge status */
#define  FSMC_SR4_IREN                       ((uint8_t)0x08)               /* Interrupt Rising Edge detection Enable bit */
#define  FSMC_SR4_ILEN                       ((uint8_t)0x10)               /* Interrupt Level detection Enable bit */
#define  FSMC_SR4_IFEN                       ((uint8_t)0x20)               /* Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR4_FEMPT                      ((uint8_t)0x40)               /* FIFO empty */

/******************  Bit definition for FSMC_PMEM2 register  ******************/
#define  FSMC_PMEM2_MEMSET2                  ((uint32_t)0x000000FF)        /* MEMSET2[7:0] bits (Common memory 2 setup time) */
#define  FSMC_PMEM2_MEMSET2_0                ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_PMEM2_MEMSET2_1                ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_PMEM2_MEMSET2_2                ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_PMEM2_MEMSET2_3                ((uint32_t)0x00000008)        /* Bit 3 */
#define  FSMC_PMEM2_MEMSET2_4                ((uint32_t)0x00000010)        /* Bit 4 */
#define  FSMC_PMEM2_MEMSET2_5                ((uint32_t)0x00000020)        /* Bit 5 */
#define  FSMC_PMEM2_MEMSET2_6                ((uint32_t)0x00000040)        /* Bit 6 */
#define  FSMC_PMEM2_MEMSET2_7                ((uint32_t)0x00000080)        /* Bit 7 */

#define  FSMC_PMEM2_MEMWAIT2                 ((uint32_t)0x0000FF00)        /* MEMWAIT2[7:0] bits (Common memory 2 wait time) */
#define  FSMC_PMEM2_MEMWAIT2_0               ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_PMEM2_MEMWAIT2_1               ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_PMEM2_MEMWAIT2_2               ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_PMEM2_MEMWAIT2_3               ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_PMEM2_MEMWAIT2_4               ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_PMEM2_MEMWAIT2_5               ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_PMEM2_MEMWAIT2_6               ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_PMEM2_MEMWAIT2_7               ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_PMEM2_MEMHOLD2                 ((uint32_t)0x00FF0000)        /* MEMHOLD2[7:0] bits (Common memory 2 hold time) */
#define  FSMC_PMEM2_MEMHOLD2_0               ((uint32_t)0x00010000)        /* Bit 0 */
#define  FSMC_PMEM2_MEMHOLD2_1               ((uint32_t)0x00020000)        /* Bit 1 */
#define  FSMC_PMEM2_MEMHOLD2_2               ((uint32_t)0x00040000)        /* Bit 2 */
#define  FSMC_PMEM2_MEMHOLD2_3               ((uint32_t)0x00080000)        /* Bit 3 */
#define  FSMC_PMEM2_MEMHOLD2_4               ((uint32_t)0x00100000)        /* Bit 4 */
#define  FSMC_PMEM2_MEMHOLD2_5               ((uint32_t)0x00200000)        /* Bit 5 */
#define  FSMC_PMEM2_MEMHOLD2_6               ((uint32_t)0x00400000)        /* Bit 6 */
#define  FSMC_PMEM2_MEMHOLD2_7               ((uint32_t)0x00800000)        /* Bit 7 */

#define  FSMC_PMEM2_MEMHIZ2                  ((uint32_t)0xFF000000)        /* MEMHIZ2[7:0] bits (Common memory 2 databus HiZ time) */
#define  FSMC_PMEM2_MEMHIZ2_0                ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_PMEM2_MEMHIZ2_1                ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_PMEM2_MEMHIZ2_2                ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_PMEM2_MEMHIZ2_3                ((uint32_t)0x08000000)        /* Bit 3 */
#define  FSMC_PMEM2_MEMHIZ2_4                ((uint32_t)0x10000000)        /* Bit 4 */
#define  FSMC_PMEM2_MEMHIZ2_5                ((uint32_t)0x20000000)        /* Bit 5 */
#define  FSMC_PMEM2_MEMHIZ2_6                ((uint32_t)0x40000000)        /* Bit 6 */
#define  FSMC_PMEM2_MEMHIZ2_7                ((uint32_t)0x80000000)        /* Bit 7 */

/******************  Bit definition for FSMC_PMEM3 register  ******************/
#define  FSMC_PMEM3_MEMSET3                  ((uint32_t)0x000000FF)        /* MEMSET3[7:0] bits (Common memory 3 setup time) */
#define  FSMC_PMEM3_MEMSET3_0                ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_PMEM3_MEMSET3_1                ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_PMEM3_MEMSET3_2                ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_PMEM3_MEMSET3_3                ((uint32_t)0x00000008)        /* Bit 3 */
#define  FSMC_PMEM3_MEMSET3_4                ((uint32_t)0x00000010)        /* Bit 4 */
#define  FSMC_PMEM3_MEMSET3_5                ((uint32_t)0x00000020)        /* Bit 5 */
#define  FSMC_PMEM3_MEMSET3_6                ((uint32_t)0x00000040)        /* Bit 6 */
#define  FSMC_PMEM3_MEMSET3_7                ((uint32_t)0x00000080)        /* Bit 7 */

#define  FSMC_PMEM3_MEMWAIT3                 ((uint32_t)0x0000FF00)        /* MEMWAIT3[7:0] bits (Common memory 3 wait time) */
#define  FSMC_PMEM3_MEMWAIT3_0               ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_PMEM3_MEMWAIT3_1               ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_PMEM3_MEMWAIT3_2               ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_PMEM3_MEMWAIT3_3               ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_PMEM3_MEMWAIT3_4               ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_PMEM3_MEMWAIT3_5               ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_PMEM3_MEMWAIT3_6               ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_PMEM3_MEMWAIT3_7               ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_PMEM3_MEMHOLD3                 ((uint32_t)0x00FF0000)        /* MEMHOLD3[7:0] bits (Common memory 3 hold time) */
#define  FSMC_PMEM3_MEMHOLD3_0               ((uint32_t)0x00010000)        /* Bit 0 */
#define  FSMC_PMEM3_MEMHOLD3_1               ((uint32_t)0x00020000)        /* Bit 1 */
#define  FSMC_PMEM3_MEMHOLD3_2               ((uint32_t)0x00040000)        /* Bit 2 */
#define  FSMC_PMEM3_MEMHOLD3_3               ((uint32_t)0x00080000)        /* Bit 3 */
#define  FSMC_PMEM3_MEMHOLD3_4               ((uint32_t)0x00100000)        /* Bit 4 */
#define  FSMC_PMEM3_MEMHOLD3_5               ((uint32_t)0x00200000)        /* Bit 5 */
#define  FSMC_PMEM3_MEMHOLD3_6               ((uint32_t)0x00400000)        /* Bit 6 */
#define  FSMC_PMEM3_MEMHOLD3_7               ((uint32_t)0x00800000)        /* Bit 7 */

#define  FSMC_PMEM3_MEMHIZ3                  ((uint32_t)0xFF000000)        /* MEMHIZ3[7:0] bits (Common memory 3 databus HiZ time) */
#define  FSMC_PMEM3_MEMHIZ3_0                ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_PMEM3_MEMHIZ3_1                ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_PMEM3_MEMHIZ3_2                ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_PMEM3_MEMHIZ3_3                ((uint32_t)0x08000000)        /* Bit 3 */
#define  FSMC_PMEM3_MEMHIZ3_4                ((uint32_t)0x10000000)        /* Bit 4 */
#define  FSMC_PMEM3_MEMHIZ3_5                ((uint32_t)0x20000000)        /* Bit 5 */
#define  FSMC_PMEM3_MEMHIZ3_6                ((uint32_t)0x40000000)        /* Bit 6 */
#define  FSMC_PMEM3_MEMHIZ3_7                ((uint32_t)0x80000000)        /* Bit 7 */

/******************  Bit definition for FSMC_PMEM4 register  ******************/
#define  FSMC_PMEM4_MEMSET4                  ((uint32_t)0x000000FF)        /* MEMSET4[7:0] bits (Common memory 4 setup time) */
#define  FSMC_PMEM4_MEMSET4_0                ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_PMEM4_MEMSET4_1                ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_PMEM4_MEMSET4_2                ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_PMEM4_MEMSET4_3                ((uint32_t)0x00000008)        /* Bit 3 */
#define  FSMC_PMEM4_MEMSET4_4                ((uint32_t)0x00000010)        /* Bit 4 */
#define  FSMC_PMEM4_MEMSET4_5                ((uint32_t)0x00000020)        /* Bit 5 */
#define  FSMC_PMEM4_MEMSET4_6                ((uint32_t)0x00000040)        /* Bit 6 */
#define  FSMC_PMEM4_MEMSET4_7                ((uint32_t)0x00000080)        /* Bit 7 */

#define  FSMC_PMEM4_MEMWAIT4                 ((uint32_t)0x0000FF00)        /* MEMWAIT4[7:0] bits (Common memory 4 wait time) */
#define  FSMC_PMEM4_MEMWAIT4_0               ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_PMEM4_MEMWAIT4_1               ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_PMEM4_MEMWAIT4_2               ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_PMEM4_MEMWAIT4_3               ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_PMEM4_MEMWAIT4_4               ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_PMEM4_MEMWAIT4_5               ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_PMEM4_MEMWAIT4_6               ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_PMEM4_MEMWAIT4_7               ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_PMEM4_MEMHOLD4                 ((uint32_t)0x00FF0000)        /* MEMHOLD4[7:0] bits (Common memory 4 hold time) */
#define  FSMC_PMEM4_MEMHOLD4_0               ((uint32_t)0x00010000)        /* Bit 0 */
#define  FSMC_PMEM4_MEMHOLD4_1               ((uint32_t)0x00020000)        /* Bit 1 */
#define  FSMC_PMEM4_MEMHOLD4_2               ((uint32_t)0x00040000)        /* Bit 2 */
#define  FSMC_PMEM4_MEMHOLD4_3               ((uint32_t)0x00080000)        /* Bit 3 */
#define  FSMC_PMEM4_MEMHOLD4_4               ((uint32_t)0x00100000)        /* Bit 4 */
#define  FSMC_PMEM4_MEMHOLD4_5               ((uint32_t)0x00200000)        /* Bit 5 */
#define  FSMC_PMEM4_MEMHOLD4_6               ((uint32_t)0x00400000)        /* Bit 6 */
#define  FSMC_PMEM4_MEMHOLD4_7               ((uint32_t)0x00800000)        /* Bit 7 */

#define  FSMC_PMEM4_MEMHIZ4                  ((uint32_t)0xFF000000)        /* MEMHIZ4[7:0] bits (Common memory 4 databus HiZ time) */
#define  FSMC_PMEM4_MEMHIZ4_0                ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_PMEM4_MEMHIZ4_1                ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_PMEM4_MEMHIZ4_2                ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_PMEM4_MEMHIZ4_3                ((uint32_t)0x08000000)        /* Bit 3 */
#define  FSMC_PMEM4_MEMHIZ4_4                ((uint32_t)0x10000000)        /* Bit 4 */
#define  FSMC_PMEM4_MEMHIZ4_5                ((uint32_t)0x20000000)        /* Bit 5 */
#define  FSMC_PMEM4_MEMHIZ4_6                ((uint32_t)0x40000000)        /* Bit 6 */
#define  FSMC_PMEM4_MEMHIZ4_7                ((uint32_t)0x80000000)        /* Bit 7 */

/******************  Bit definition for FSMC_PATT2 register  ******************/
#define  FSMC_PATT2_ATTSET2                  ((uint32_t)0x000000FF)        /* ATTSET2[7:0] bits (Attribute memory 2 setup time) */
#define  FSMC_PATT2_ATTSET2_0                ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_PATT2_ATTSET2_1                ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_PATT2_ATTSET2_2                ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_PATT2_ATTSET2_3                ((uint32_t)0x00000008)        /* Bit 3 */
#define  FSMC_PATT2_ATTSET2_4                ((uint32_t)0x00000010)        /* Bit 4 */
#define  FSMC_PATT2_ATTSET2_5                ((uint32_t)0x00000020)        /* Bit 5 */
#define  FSMC_PATT2_ATTSET2_6                ((uint32_t)0x00000040)        /* Bit 6 */
#define  FSMC_PATT2_ATTSET2_7                ((uint32_t)0x00000080)        /* Bit 7 */

#define  FSMC_PATT2_ATTWAIT2                 ((uint32_t)0x0000FF00)        /* ATTWAIT2[7:0] bits (Attribute memory 2 wait time) */
#define  FSMC_PATT2_ATTWAIT2_0               ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_PATT2_ATTWAIT2_1               ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_PATT2_ATTWAIT2_2               ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_PATT2_ATTWAIT2_3               ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_PATT2_ATTWAIT2_4               ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_PATT2_ATTWAIT2_5               ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_PATT2_ATTWAIT2_6               ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_PATT2_ATTWAIT2_7               ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_PATT2_ATTHOLD2                 ((uint32_t)0x00FF0000)        /* ATTHOLD2[7:0] bits (Attribute memory 2 hold time) */
#define  FSMC_PATT2_ATTHOLD2_0               ((uint32_t)0x00010000)        /* Bit 0 */
#define  FSMC_PATT2_ATTHOLD2_1               ((uint32_t)0x00020000)        /* Bit 1 */
#define  FSMC_PATT2_ATTHOLD2_2               ((uint32_t)0x00040000)        /* Bit 2 */
#define  FSMC_PATT2_ATTHOLD2_3               ((uint32_t)0x00080000)        /* Bit 3 */
#define  FSMC_PATT2_ATTHOLD2_4               ((uint32_t)0x00100000)        /* Bit 4 */
#define  FSMC_PATT2_ATTHOLD2_5               ((uint32_t)0x00200000)        /* Bit 5 */
#define  FSMC_PATT2_ATTHOLD2_6               ((uint32_t)0x00400000)        /* Bit 6 */
#define  FSMC_PATT2_ATTHOLD2_7               ((uint32_t)0x00800000)        /* Bit 7 */

#define  FSMC_PATT2_ATTHIZ2                  ((uint32_t)0xFF000000)        /* ATTHIZ2[7:0] bits (Attribute memory 2 databus HiZ time) */
#define  FSMC_PATT2_ATTHIZ2_0                ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_PATT2_ATTHIZ2_1                ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_PATT2_ATTHIZ2_2                ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_PATT2_ATTHIZ2_3                ((uint32_t)0x08000000)        /* Bit 3 */
#define  FSMC_PATT2_ATTHIZ2_4                ((uint32_t)0x10000000)        /* Bit 4 */
#define  FSMC_PATT2_ATTHIZ2_5                ((uint32_t)0x20000000)        /* Bit 5 */
#define  FSMC_PATT2_ATTHIZ2_6                ((uint32_t)0x40000000)        /* Bit 6 */
#define  FSMC_PATT2_ATTHIZ2_7                ((uint32_t)0x80000000)        /* Bit 7 */

/******************  Bit definition for FSMC_PATT3 register  ******************/
#define  FSMC_PATT3_ATTSET3                  ((uint32_t)0x000000FF)        /* ATTSET3[7:0] bits (Attribute memory 3 setup time) */
#define  FSMC_PATT3_ATTSET3_0                ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_PATT3_ATTSET3_1                ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_PATT3_ATTSET3_2                ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_PATT3_ATTSET3_3                ((uint32_t)0x00000008)        /* Bit 3 */
#define  FSMC_PATT3_ATTSET3_4                ((uint32_t)0x00000010)        /* Bit 4 */
#define  FSMC_PATT3_ATTSET3_5                ((uint32_t)0x00000020)        /* Bit 5 */
#define  FSMC_PATT3_ATTSET3_6                ((uint32_t)0x00000040)        /* Bit 6 */
#define  FSMC_PATT3_ATTSET3_7                ((uint32_t)0x00000080)        /* Bit 7 */

#define  FSMC_PATT3_ATTWAIT3                 ((uint32_t)0x0000FF00)        /* ATTWAIT3[7:0] bits (Attribute memory 3 wait time) */
#define  FSMC_PATT3_ATTWAIT3_0               ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_PATT3_ATTWAIT3_1               ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_PATT3_ATTWAIT3_2               ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_PATT3_ATTWAIT3_3               ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_PATT3_ATTWAIT3_4               ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_PATT3_ATTWAIT3_5               ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_PATT3_ATTWAIT3_6               ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_PATT3_ATTWAIT3_7               ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_PATT3_ATTHOLD3                 ((uint32_t)0x00FF0000)        /* ATTHOLD3[7:0] bits (Attribute memory 3 hold time) */
#define  FSMC_PATT3_ATTHOLD3_0               ((uint32_t)0x00010000)        /* Bit 0 */
#define  FSMC_PATT3_ATTHOLD3_1               ((uint32_t)0x00020000)        /* Bit 1 */
#define  FSMC_PATT3_ATTHOLD3_2               ((uint32_t)0x00040000)        /* Bit 2 */
#define  FSMC_PATT3_ATTHOLD3_3               ((uint32_t)0x00080000)        /* Bit 3 */
#define  FSMC_PATT3_ATTHOLD3_4               ((uint32_t)0x00100000)        /* Bit 4 */
#define  FSMC_PATT3_ATTHOLD3_5               ((uint32_t)0x00200000)        /* Bit 5 */
#define  FSMC_PATT3_ATTHOLD3_6               ((uint32_t)0x00400000)        /* Bit 6 */
#define  FSMC_PATT3_ATTHOLD3_7               ((uint32_t)0x00800000)        /* Bit 7 */

#define  FSMC_PATT3_ATTHIZ3                  ((uint32_t)0xFF000000)        /* ATTHIZ3[7:0] bits (Attribute memory 3 databus HiZ time) */
#define  FSMC_PATT3_ATTHIZ3_0                ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_PATT3_ATTHIZ3_1                ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_PATT3_ATTHIZ3_2                ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_PATT3_ATTHIZ3_3                ((uint32_t)0x08000000)        /* Bit 3 */
#define  FSMC_PATT3_ATTHIZ3_4                ((uint32_t)0x10000000)        /* Bit 4 */
#define  FSMC_PATT3_ATTHIZ3_5                ((uint32_t)0x20000000)        /* Bit 5 */
#define  FSMC_PATT3_ATTHIZ3_6                ((uint32_t)0x40000000)        /* Bit 6 */
#define  FSMC_PATT3_ATTHIZ3_7                ((uint32_t)0x80000000)        /* Bit 7 */

/******************  Bit definition for FSMC_PATT4 register  ******************/
#define  FSMC_PATT4_ATTSET4                  ((uint32_t)0x000000FF)        /* ATTSET4[7:0] bits (Attribute memory 4 setup time) */
#define  FSMC_PATT4_ATTSET4_0                ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_PATT4_ATTSET4_1                ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_PATT4_ATTSET4_2                ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_PATT4_ATTSET4_3                ((uint32_t)0x00000008)        /* Bit 3 */
#define  FSMC_PATT4_ATTSET4_4                ((uint32_t)0x00000010)        /* Bit 4 */
#define  FSMC_PATT4_ATTSET4_5                ((uint32_t)0x00000020)        /* Bit 5 */
#define  FSMC_PATT4_ATTSET4_6                ((uint32_t)0x00000040)        /* Bit 6 */
#define  FSMC_PATT4_ATTSET4_7                ((uint32_t)0x00000080)        /* Bit 7 */

#define  FSMC_PATT4_ATTWAIT4                 ((uint32_t)0x0000FF00)        /* ATTWAIT4[7:0] bits (Attribute memory 4 wait time) */
#define  FSMC_PATT4_ATTWAIT4_0               ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_PATT4_ATTWAIT4_1               ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_PATT4_ATTWAIT4_2               ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_PATT4_ATTWAIT4_3               ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_PATT4_ATTWAIT4_4               ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_PATT4_ATTWAIT4_5               ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_PATT4_ATTWAIT4_6               ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_PATT4_ATTWAIT4_7               ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_PATT4_ATTHOLD4                 ((uint32_t)0x00FF0000)        /* ATTHOLD4[7:0] bits (Attribute memory 4 hold time) */
#define  FSMC_PATT4_ATTHOLD4_0               ((uint32_t)0x00010000)        /* Bit 0 */
#define  FSMC_PATT4_ATTHOLD4_1               ((uint32_t)0x00020000)        /* Bit 1 */
#define  FSMC_PATT4_ATTHOLD4_2               ((uint32_t)0x00040000)        /* Bit 2 */
#define  FSMC_PATT4_ATTHOLD4_3               ((uint32_t)0x00080000)        /* Bit 3 */
#define  FSMC_PATT4_ATTHOLD4_4               ((uint32_t)0x00100000)        /* Bit 4 */
#define  FSMC_PATT4_ATTHOLD4_5               ((uint32_t)0x00200000)        /* Bit 5 */
#define  FSMC_PATT4_ATTHOLD4_6               ((uint32_t)0x00400000)        /* Bit 6 */
#define  FSMC_PATT4_ATTHOLD4_7               ((uint32_t)0x00800000)        /* Bit 7 */

#define  FSMC_PATT4_ATTHIZ4                  ((uint32_t)0xFF000000)        /* ATTHIZ4[7:0] bits (Attribute memory 4 databus HiZ time) */
#define  FSMC_PATT4_ATTHIZ4_0                ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_PATT4_ATTHIZ4_1                ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_PATT4_ATTHIZ4_2                ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_PATT4_ATTHIZ4_3                ((uint32_t)0x08000000)        /* Bit 3 */
#define  FSMC_PATT4_ATTHIZ4_4                ((uint32_t)0x10000000)        /* Bit 4 */
#define  FSMC_PATT4_ATTHIZ4_5                ((uint32_t)0x20000000)        /* Bit 5 */
#define  FSMC_PATT4_ATTHIZ4_6                ((uint32_t)0x40000000)        /* Bit 6 */
#define  FSMC_PATT4_ATTHIZ4_7                ((uint32_t)0x80000000)        /* Bit 7 */

/******************  Bit definition for FSMC_PIO4 register  *******************/
#define  FSMC_PIO4_IOSET4                    ((uint32_t)0x000000FF)        /* IOSET4[7:0] bits (I/O 4 setup time) */
#define  FSMC_PIO4_IOSET4_0                  ((uint32_t)0x00000001)        /* Bit 0 */
#define  FSMC_PIO4_IOSET4_1                  ((uint32_t)0x00000002)        /* Bit 1 */
#define  FSMC_PIO4_IOSET4_2                  ((uint32_t)0x00000004)        /* Bit 2 */
#define  FSMC_PIO4_IOSET4_3                  ((uint32_t)0x00000008)        /* Bit 3 */
#define  FSMC_PIO4_IOSET4_4                  ((uint32_t)0x00000010)        /* Bit 4 */
#define  FSMC_PIO4_IOSET4_5                  ((uint32_t)0x00000020)        /* Bit 5 */
#define  FSMC_PIO4_IOSET4_6                  ((uint32_t)0x00000040)        /* Bit 6 */
#define  FSMC_PIO4_IOSET4_7                  ((uint32_t)0x00000080)        /* Bit 7 */

#define  FSMC_PIO4_IOWAIT4                   ((uint32_t)0x0000FF00)        /* IOWAIT4[7:0] bits (I/O 4 wait time) */
#define  FSMC_PIO4_IOWAIT4_0                 ((uint32_t)0x00000100)        /* Bit 0 */
#define  FSMC_PIO4_IOWAIT4_1                 ((uint32_t)0x00000200)        /* Bit 1 */
#define  FSMC_PIO4_IOWAIT4_2                 ((uint32_t)0x00000400)        /* Bit 2 */
#define  FSMC_PIO4_IOWAIT4_3                 ((uint32_t)0x00000800)        /* Bit 3 */
#define  FSMC_PIO4_IOWAIT4_4                 ((uint32_t)0x00001000)        /* Bit 4 */
#define  FSMC_PIO4_IOWAIT4_5                 ((uint32_t)0x00002000)        /* Bit 5 */
#define  FSMC_PIO4_IOWAIT4_6                 ((uint32_t)0x00004000)        /* Bit 6 */
#define  FSMC_PIO4_IOWAIT4_7                 ((uint32_t)0x00008000)        /* Bit 7 */

#define  FSMC_PIO4_IOHOLD4                   ((uint32_t)0x00FF0000)        /* IOHOLD4[7:0] bits (I/O 4 hold time) */
#define  FSMC_PIO4_IOHOLD4_0                 ((uint32_t)0x00010000)        /* Bit 0 */
#define  FSMC_PIO4_IOHOLD4_1                 ((uint32_t)0x00020000)        /* Bit 1 */
#define  FSMC_PIO4_IOHOLD4_2                 ((uint32_t)0x00040000)        /* Bit 2 */
#define  FSMC_PIO4_IOHOLD4_3                 ((uint32_t)0x00080000)        /* Bit 3 */
#define  FSMC_PIO4_IOHOLD4_4                 ((uint32_t)0x00100000)        /* Bit 4 */
#define  FSMC_PIO4_IOHOLD4_5                 ((uint32_t)0x00200000)        /* Bit 5 */
#define  FSMC_PIO4_IOHOLD4_6                 ((uint32_t)0x00400000)        /* Bit 6 */
#define  FSMC_PIO4_IOHOLD4_7                 ((uint32_t)0x00800000)        /* Bit 7 */

#define  FSMC_PIO4_IOHIZ4                    ((uint32_t)0xFF000000)        /* IOHIZ4[7:0] bits (I/O 4 databus HiZ time) */
#define  FSMC_PIO4_IOHIZ4_0                  ((uint32_t)0x01000000)        /* Bit 0 */
#define  FSMC_PIO4_IOHIZ4_1                  ((uint32_t)0x02000000)        /* Bit 1 */
#define  FSMC_PIO4_IOHIZ4_2                  ((uint32_t)0x04000000)        /* Bit 2 */
#define  FSMC_PIO4_IOHIZ4_3                  ((uint32_t)0x08000000)        /* Bit 3 */
#define  FSMC_PIO4_IOHIZ4_4                  ((uint32_t)0x10000000)        /* Bit 4 */
#define  FSMC_PIO4_IOHIZ4_5                  ((uint32_t)0x20000000)        /* Bit 5 */
#define  FSMC_PIO4_IOHIZ4_6                  ((uint32_t)0x40000000)        /* Bit 6 */
#define  FSMC_PIO4_IOHIZ4_7                  ((uint32_t)0x80000000)        /* Bit 7 */

/******************  Bit definition for FSMC_ECCR2 register  ******************/
#define  FSMC_ECCR2_ECC2                     ((uint32_t)0xFFFFFFFF)        /* ECC result */

/******************  Bit definition for FSMC_ECCR3 register  ******************/
#define  FSMC_ECCR3_ECC3                     ((uint32_t)0xFFFFFFFF)        /* ECC result */

/******************************************************************************/
/*                                                                            */
/*                          SD host Interface                                 */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for SDIO_POWER register  ******************/
#define  SDIO_POWER_PWRCTRL                  ((uint8_t)0x03)               /* PWRCTRL[1:0] bits (Power supply control bits) */
#define  SDIO_POWER_PWRCTRL_0                ((uint8_t)0x01)               /* Bit 0 */
#define  SDIO_POWER_PWRCTRL_1                ((uint8_t)0x02)               /* Bit 1 */

/******************  Bit definition for SDIO_CLKCR register  ******************/
#define  SDIO_CLKCR_CLKDIV                   ((uint16_t)0x00FF)            /* Clock divide factor */
#define  SDIO_CLKCR_CLKEN                    ((uint16_t)0x0100)            /* Clock enable bit */
#define  SDIO_CLKCR_PWRSAV                   ((uint16_t)0x0200)            /* Power saving configuration bit */
#define  SDIO_CLKCR_BYPASS                   ((uint16_t)0x0400)            /* Clock divider bypass enable bit */

#define  SDIO_CLKCR_WIDBUS                   ((uint16_t)0x1800)            /* WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define  SDIO_CLKCR_WIDBUS_0                 ((uint16_t)0x0800)            /* Bit 0 */
#define  SDIO_CLKCR_WIDBUS_1                 ((uint16_t)0x1000)            /* Bit 1 */

#define  SDIO_CLKCR_NEGEDGE                  ((uint16_t)0x2000)            /* SDIO_CK dephasing selection bit */
#define  SDIO_CLKCR_HWFC_EN                  ((uint16_t)0x4000)            /* HW Flow Control enable */

/*******************  Bit definition for SDIO_ARG register  *******************/
#define  SDIO_ARG_CMDARG                     ((uint32_t)0xFFFFFFFF)            /* Command argument */

/*******************  Bit definition for SDIO_CMD register  *******************/
#define  SDIO_CMD_CMDINDEX                   ((uint16_t)0x003F)            /* Command Index */

#define  SDIO_CMD_WAITRESP                   ((uint16_t)0x00C0)            /* WAITRESP[1:0] bits (Wait for response bits) */
#define  SDIO_CMD_WAITRESP_0                 ((uint16_t)0x0040)            /*  Bit 0 */
#define  SDIO_CMD_WAITRESP_1                 ((uint16_t)0x0080)            /*  Bit 1 */

#define  SDIO_CMD_WAITINT                    ((uint16_t)0x0100)            /* CPSM Waits for Interrupt Request */
#define  SDIO_CMD_WAITPEND                   ((uint16_t)0x0200)            /* CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define  SDIO_CMD_CPSMEN                     ((uint16_t)0x0400)            /* Command path state machine (CPSM) Enable bit */
#define  SDIO_CMD_SDIOSUSPEND                ((uint16_t)0x0800)            /* SD I/O suspend command */
#define  SDIO_CMD_ENCMDCOMPL                 ((uint16_t)0x1000)            /* Enable CMD completion */
#define  SDIO_CMD_NIEN                       ((uint16_t)0x2000)            /* Not Interrupt Enable */
#define  SDIO_CMD_CEATACMD                   ((uint16_t)0x4000)            /* CE-ATA command */

/*****************  Bit definition for SDIO_RESPCMD register  *****************/
#define  SDIO_RESPCMD_RESPCMD                ((uint8_t)0x3F)               /* Response command index */

/******************  Bit definition for SDIO_RESP0 register  ******************/
#define  SDIO_RESP0_CARDSTATUS0              ((uint32_t)0xFFFFFFFF)        /* Card Status */

/******************  Bit definition for SDIO_RESP1 register  ******************/
#define  SDIO_RESP1_CARDSTATUS1              ((uint32_t)0xFFFFFFFF)        /* Card Status */

/******************  Bit definition for SDIO_RESP2 register  ******************/
#define  SDIO_RESP2_CARDSTATUS2              ((uint32_t)0xFFFFFFFF)        /* Card Status */

/******************  Bit definition for SDIO_RESP3 register  ******************/
#define  SDIO_RESP3_CARDSTATUS3              ((uint32_t)0xFFFFFFFF)        /* Card Status */

/******************  Bit definition for SDIO_RESP4 register  ******************/
#define  SDIO_RESP4_CARDSTATUS4              ((uint32_t)0xFFFFFFFF)        /* Card Status */

/******************  Bit definition for SDIO_DTIMER register  *****************/
#define  SDIO_DTIMER_DATATIME                ((uint32_t)0xFFFFFFFF)        /* Data timeout period. */

/******************  Bit definition for SDIO_DLEN register  *******************/
#define  SDIO_DLEN_DATALENGTH                ((uint32_t)0x01FFFFFF)        /* Data length value */

/******************  Bit definition for SDIO_DCTRL register  ******************/
#define  SDIO_DCTRL_DTEN                     ((uint16_t)0x0001)            /* Data transfer enabled bit */
#define  SDIO_DCTRL_DTDIR                    ((uint16_t)0x0002)            /* Data transfer direction selection */
#define  SDIO_DCTRL_DTMODE                   ((uint16_t)0x0004)            /* Data transfer mode selection */
#define  SDIO_DCTRL_DMAEN                    ((uint16_t)0x0008)            /* DMA enabled bit */

#define  SDIO_DCTRL_DBLOCKSIZE               ((uint16_t)0x00F0)            /* DBLOCKSIZE[3:0] bits (Data block size) */
#define  SDIO_DCTRL_DBLOCKSIZE_0             ((uint16_t)0x0010)            /* Bit 0 */
#define  SDIO_DCTRL_DBLOCKSIZE_1             ((uint16_t)0x0020)            /* Bit 1 */
#define  SDIO_DCTRL_DBLOCKSIZE_2             ((uint16_t)0x0040)            /* Bit 2 */
#define  SDIO_DCTRL_DBLOCKSIZE_3             ((uint16_t)0x0080)            /* Bit 3 */

#define  SDIO_DCTRL_RWSTART                  ((uint16_t)0x0100)            /* Read wait start */
#define  SDIO_DCTRL_RWSTOP                   ((uint16_t)0x0200)            /* Read wait stop */
#define  SDIO_DCTRL_RWMOD                    ((uint16_t)0x0400)            /* Read wait mode */
#define  SDIO_DCTRL_SDIOEN                   ((uint16_t)0x0800)            /* SD I/O enable functions */

/******************  Bit definition for SDIO_DCOUNT register  *****************/
#define  SDIO_DCOUNT_DATACOUNT               ((uint32_t)0x01FFFFFF)        /* Data count value */

/******************  Bit definition for SDIO_STA register  ********************/
#define  SDIO_STA_CCRCFAIL                   ((uint32_t)0x00000001)        /* Command response received (CRC check failed) */
#define  SDIO_STA_DCRCFAIL                   ((uint32_t)0x00000002)        /* Data block sent/received (CRC check failed) */
#define  SDIO_STA_CTIMEOUT                   ((uint32_t)0x00000004)        /* Command response timeout */
#define  SDIO_STA_DTIMEOUT                   ((uint32_t)0x00000008)        /* Data timeout */
#define  SDIO_STA_TXUNDERR                   ((uint32_t)0x00000010)        /* Transmit FIFO underrun error */
#define  SDIO_STA_RXOVERR                    ((uint32_t)0x00000020)        /* Received FIFO overrun error */
#define  SDIO_STA_CMDREND                    ((uint32_t)0x00000040)        /* Command response received (CRC check passed) */
#define  SDIO_STA_CMDSENT                    ((uint32_t)0x00000080)        /* Command sent (no response required) */
#define  SDIO_STA_DATAEND                    ((uint32_t)0x00000100)        /* Data end (data counter, SDIDCOUNT, is zero) */
#define  SDIO_STA_STBITERR                   ((uint32_t)0x00000200)        /* Start bit not detected on all data signals in wide bus mode */
#define  SDIO_STA_DBCKEND                    ((uint32_t)0x00000400)        /* Data block sent/received (CRC check passed) */
#define  SDIO_STA_CMDACT                     ((uint32_t)0x00000800)        /* Command transfer in progress */
#define  SDIO_STA_TXACT                      ((uint32_t)0x00001000)        /* Data transmit in progress */
#define  SDIO_STA_RXACT                      ((uint32_t)0x00002000)        /* Data receive in progress */
#define  SDIO_STA_TXFIFOHE                   ((uint32_t)0x00004000)        /* Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define  SDIO_STA_RXFIFOHF                   ((uint32_t)0x00008000)        /* Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define  SDIO_STA_TXFIFOF                    ((uint32_t)0x00010000)        /* Transmit FIFO full */
#define  SDIO_STA_RXFIFOF                    ((uint32_t)0x00020000)        /* Receive FIFO full */
#define  SDIO_STA_TXFIFOE                    ((uint32_t)0x00040000)        /* Transmit FIFO empty */
#define  SDIO_STA_RXFIFOE                    ((uint32_t)0x00080000)        /* Receive FIFO empty */
#define  SDIO_STA_TXDAVL                     ((uint32_t)0x00100000)        /* Data available in transmit FIFO */
#define  SDIO_STA_RXDAVL                     ((uint32_t)0x00200000)        /* Data available in receive FIFO */
#define  SDIO_STA_SDIOIT                     ((uint32_t)0x00400000)        /* SDIO interrupt received */
#define  SDIO_STA_CEATAEND                   ((uint32_t)0x00800000)        /* CE-ATA command completion signal received for CMD61 */

/*******************  Bit definition for SDIO_ICR register  *******************/
#define  SDIO_ICR_CCRCFAILC                  ((uint32_t)0x00000001)        /* CCRCFAIL flag clear bit */
#define  SDIO_ICR_DCRCFAILC                  ((uint32_t)0x00000002)        /* DCRCFAIL flag clear bit */
#define  SDIO_ICR_CTIMEOUTC                  ((uint32_t)0x00000004)        /* CTIMEOUT flag clear bit */
#define  SDIO_ICR_DTIMEOUTC                  ((uint32_t)0x00000008)        /* DTIMEOUT flag clear bit */
#define  SDIO_ICR_TXUNDERRC                  ((uint32_t)0x00000010)        /* TXUNDERR flag clear bit */
#define  SDIO_ICR_RXOVERRC                   ((uint32_t)0x00000020)        /* RXOVERR flag clear bit */
#define  SDIO_ICR_CMDRENDC                   ((uint32_t)0x00000040)        /* CMDREND flag clear bit */
#define  SDIO_ICR_CMDSENTC                   ((uint32_t)0x00000080)        /* CMDSENT flag clear bit */
#define  SDIO_ICR_DATAENDC                   ((uint32_t)0x00000100)        /* DATAEND flag clear bit */
#define  SDIO_ICR_STBITERRC                  ((uint32_t)0x00000200)        /* STBITERR flag clear bit */
#define  SDIO_ICR_DBCKENDC                   ((uint32_t)0x00000400)        /* DBCKEND flag clear bit */
#define  SDIO_ICR_SDIOITC                    ((uint32_t)0x00400000)        /* SDIOIT flag clear bit */
#define  SDIO_ICR_CEATAENDC                  ((uint32_t)0x00800000)        /* CEATAEND flag clear bit */

/******************  Bit definition for SDIO_MASK register  *******************/
#define  SDIO_MASK_CCRCFAILIE                ((uint32_t)0x00000001)        /* Command CRC Fail Interrupt Enable */
#define  SDIO_MASK_DCRCFAILIE                ((uint32_t)0x00000002)        /* Data CRC Fail Interrupt Enable */
#define  SDIO_MASK_CTIMEOUTIE                ((uint32_t)0x00000004)        /* Command TimeOut Interrupt Enable */
#define  SDIO_MASK_DTIMEOUTIE                ((uint32_t)0x00000008)        /* Data TimeOut Interrupt Enable */
#define  SDIO_MASK_TXUNDERRIE                ((uint32_t)0x00000010)        /* Tx FIFO UnderRun Error Interrupt Enable */
#define  SDIO_MASK_RXOVERRIE                 ((uint32_t)0x00000020)        /* Rx FIFO OverRun Error Interrupt Enable */
#define  SDIO_MASK_CMDRENDIE                 ((uint32_t)0x00000040)        /* Command Response Received Interrupt Enable */
#define  SDIO_MASK_CMDSENTIE                 ((uint32_t)0x00000080)        /* Command Sent Interrupt Enable */
#define  SDIO_MASK_DATAENDIE                 ((uint32_t)0x00000100)        /* Data End Interrupt Enable */
#define  SDIO_MASK_STBITERRIE                ((uint32_t)0x00000200)        /* Start Bit Error Interrupt Enable */
#define  SDIO_MASK_DBCKENDIE                 ((uint32_t)0x00000400)        /* Data Block End Interrupt Enable */
#define  SDIO_MASK_CMDACTIE                  ((uint32_t)0x00000800)        /* Command Acting Interrupt Enable */
#define  SDIO_MASK_TXACTIE                   ((uint32_t)0x00001000)        /* Data Transmit Acting Interrupt Enable */
#define  SDIO_MASK_RXACTIE                   ((uint32_t)0x00002000)        /* Data receive acting interrupt enabled */
#define  SDIO_MASK_TXFIFOHEIE                ((uint32_t)0x00004000)        /* Tx FIFO Half Empty interrupt Enable */
#define  SDIO_MASK_RXFIFOHFIE                ((uint32_t)0x00008000)        /* Rx FIFO Half Full interrupt Enable */
#define  SDIO_MASK_TXFIFOFIE                 ((uint32_t)0x00010000)        /* Tx FIFO Full interrupt Enable */
#define  SDIO_MASK_RXFIFOFIE                 ((uint32_t)0x00020000)        /* Rx FIFO Full interrupt Enable */
#define  SDIO_MASK_TXFIFOEIE                 ((uint32_t)0x00040000)        /* Tx FIFO Empty interrupt Enable */
#define  SDIO_MASK_RXFIFOEIE                 ((uint32_t)0x00080000)        /* Rx FIFO Empty interrupt Enable */
#define  SDIO_MASK_TXDAVLIE                  ((uint32_t)0x00100000)        /* Data available in Tx FIFO interrupt Enable */
#define  SDIO_MASK_RXDAVLIE                  ((uint32_t)0x00200000)        /* Data available in Rx FIFO interrupt Enable */
#define  SDIO_MASK_SDIOITIE                  ((uint32_t)0x00400000)        /* SDIO Mode Interrupt Received interrupt Enable */
#define  SDIO_MASK_CEATAENDIE                ((uint32_t)0x00800000)        /* CE-ATA command completion signal received Interrupt Enable */

/*****************  Bit definition for SDIO_FIFOCNT register  *****************/
#define  SDIO_FIFOCNT_FIFOCOUNT              ((uint32_t)0x00FFFFFF)        /* Remaining number of words to be written to or read from the FIFO */

/******************  Bit definition for SDIO_FIFO register  *******************/
#define  SDIO_FIFO_FIFODATA                  ((uint32_t)0xFFFFFFFF)        /* Receive and transmit FIFO data */

/************************************************************************************************
 *                                                                                              *
 *                                   CRC calculation unit                                       *
 *                                                                                              *
 ************************************************************************************************/

/* Bit definition for CRC_DR register
 ************************************************************************************************/
#define  CRC_DR_DR               ((uint32_t)0xFFFFFFFF)  ///< Data register bits

/* Bit definition for CRC_IDR register
 ************************************************************************************************/
#define  CRC_IDR_IDR             ((uint32_t)0x000000FF)  ///< General-purpose 8-bit data register bits

/* Bit definition for CRC_CR register
 ************************************************************************************************/
#define  CRC_CR_RESET            ((uint32_t)0x00000001)  ///< RESET bit

/************************************************************************************************
 *                                                                                              *
 *                             FLASH and Option Bytes Registers                                 *
 *                                                                                              *
 ************************************************************************************************/
 
/* Bit definition for FLASH_ACR register
 ************************************************************************************************/
#define  FLASH_ACR_LATENCY       ((uint32_t)0x00000003)  ///< LATENCY[2:0] bits (Latency)

#define  FLASH_ACR_LATENCY_0     ((uint32_t)0x00000000)  ///< Zero wait state, if 0 < SYSCLK≤ 24 MHz
#define  FLASH_ACR_LATENCY_1     ((uint32_t)0x00000001)  ///< One wait state, if 24 MHz < SYSCLK ≤ 48 MHz
#define  FLASH_ACR_LATENCY_2     ((uint32_t)0x00000002)  ///< Two wait states, if 48 MHz < SYSCLK ≤ 72 MHz

#define  FLASH_ACR_HLFCYA        ((uint32_t)0x00000008)  ///< Flash Half Cycle Access Enable
#define  FLASH_ACR_PRFTBE        ((uint32_t)0x00000010)  ///< Prefetch Buffer Enable */
#define  FLASH_ACR_PRFTBS        ((uint32_t)0x00000020)  ///< Prefetch Buffer Status */

/* Bit definition for FLASH_KEYR register
 ************************************************************************************************/
#define  FLASH_KEYR_FKEYR        ((uint32_t)0xFFFFFFFF)  ///< FPEC Key

/* FLASH Keys
 ************************************************************************************************/
#define RDP_Key                  ((uint16_t)0x00A5)
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)

/* Bit definition for FLASH_OPTKEYR register
 ************************************************************************************************/
#define  FLASH_OPTKEYR_OPTKEYR   ((uint32_t)0xFFFFFFFF)  ///< Option Byte Key

/* Bit definition for FLASH_SR register
 ************************************************************************************************/
#define  FLASH_SR_BSY            ((uint8_t)0x01)         ///< Busy
#define  FLASH_SR_PGERR          ((uint8_t)0x04)         ///< Programming Error
#define  FLASH_SR_WRPRTERR       ((uint8_t)0x10)         ///< Write Protection Error
#define  FLASH_SR_EOP            ((uint8_t)0x20)         ///< End of operation

/* Bit definition for FLASH_CR register
 ************************************************************************************************/
#define  FLASH_CR_PG             ((uint16_t)0x0001)      ///< Programming
#define  FLASH_CR_PER            ((uint16_t)0x0002)      ///< Page Erase
#define  FLASH_CR_MER            ((uint16_t)0x0004)      ///< Mass Erase
#define  FLASH_CR_OPTPG          ((uint16_t)0x0010)      ///< Option Byte Programming
#define  FLASH_CR_OPTER          ((uint16_t)0x0020)      ///< Option Byte Erase
#define  FLASH_CR_STRT           ((uint16_t)0x0040)      ///< Start
#define  FLASH_CR_LOCK           ((uint16_t)0x0080)      ///< Lock
#define  FLASH_CR_OPTWRE         ((uint16_t)0x0200)      ///< Option Bytes Write Enable
#define  FLASH_CR_ERRIE          ((uint16_t)0x0400)      ///< Error Interrupt Enable
#define  FLASH_CR_EOPIE          ((uint16_t)0x1000)      ///< End of operation interrupt enable

/* Bit definition for FLASH_AR register
 ************************************************************************************************/
#define  FLASH_AR_FAR            ((uint32_t)0xFFFFFFFF)  ///< Flash Address

/* Bit definition for FLASH_OBR register
 ************************************************************************************************/
#define  FLASH_OBR_OPTERR        ((uint16_t)0x0001)      ///< Option Byte Error
#define  FLASH_OBR_RDPRT         ((uint16_t)0x0002)      ///< Read protection

#define  FLASH_OBR_USER          ((uint16_t)0x03FC)      ///< User Option Bytes
#define  FLASH_OBR_WDG_SW        ((uint16_t)0x0004)      ///< WDG_SW
#define  FLASH_OBR_nRST_STOP     ((uint16_t)0x0008)      ///< nRST_STOP
#define  FLASH_OBR_nRST_STDBY    ((uint16_t)0x0010)      ///< nRST_STDBY
#define  FLASH_OBR_BFB2          ((uint16_t)0x0020)      ///< BFB2

/* Bit definition for FLASH_WRPR register
 ************************************************************************************************/
#define  FLASH_WRPR_WRP          ((uint32_t)0xFFFFFFFF)  ///< Write Protect

/* Bit definition for FLASH_RDP register
 ************************************************************************************************/
#define  FLASH_RDP_RDP           ((uint32_t)0x000000FF)  ///< Read protection option byte
#define  FLASH_RDP_nRDP          ((uint32_t)0x0000FF00)  ///< Read protection complemented option byte

/* Bit definition for FLASH_USER register
 ************************************************************************************************/
#define  FLASH_USER_USER         ((uint32_t)0x00FF0000)  ///< User option byte
#define  FLASH_USER_nUSER        ((uint32_t)0xFF000000)  ///< User complemented option byte

/* Bit definition for FLASH_Data0 register
 ************************************************************************************************/
#define  FLASH_Data0_Data0       ((uint32_t)0x000000FF)  ///< User data storage option byte
#define  FLASH_Data0_nData0      ((uint32_t)0x0000FF00)  ///< User data storage complemented option byte

/* Bit definition for FLASH_Data1 register
 ************************************************************************************************/
#define  FLASH_Data1_Data1       ((uint32_t)0x00FF0000)  ///< User data storage option byte
#define  FLASH_Data1_nData1      ((uint32_t)0xFF000000)  ///< User data storage complemented option byte

/* Bit definition for FLASH_WRP0 register
 ************************************************************************************************/
#define  FLASH_WRP0_WRP0         ((uint32_t)0x000000FF)  ///< Flash memory write protection option bytes
#define  FLASH_WRP0_nWRP0        ((uint32_t)0x0000FF00)  ///< Flash memory write protection complemented option bytes

/* Bit definition for FLASH_WRP1 register
 ************************************************************************************************/
#define  FLASH_WRP1_WRP1         ((uint32_t)0x00FF0000)  ///< Flash memory write protection option bytes
#define  FLASH_WRP1_nWRP1        ((uint32_t)0xFF000000)  ///< Flash memory write protection complemented option bytes

/* Bit definition for FLASH_WRP2 register
 ************************************************************************************************/
#define  FLASH_WRP2_WRP2         ((uint32_t)0x000000FF)  ///< Flash memory write protection option bytes
#define  FLASH_WRP2_nWRP2        ((uint32_t)0x0000FF00)  ///< Flash memory write protection complemented option bytes

/* Bit definition for FLASH_WRP3 register
 ************************************************************************************************/
#define  FLASH_WRP3_WRP3         ((uint32_t)0x00FF0000)  ///< Flash memory write protection option bytes
#define  FLASH_WRP3_nWRP3        ((uint32_t)0xFF000000)  ///< Flash memory write protection complemented option bytes

/************************************************************************************************
 *                                                                                              *
 *                                    Independent WATCHDOG                                      *
 *                                                                                              *
 ************************************************************************************************/
 
/* Bit definition for IWDG_KR register
 ************************************************************************************************/
#define  IWDG_KR_KEY             ((uint16_t)0xFFFF)      ///< Key value (write only, read 0000h)

/* Bit definition for IWDG_PR register
 ************************************************************************************************/
#define  IWDG_PR_PR              ((uint8_t)0x07)         ///< PR[2:0] (Prescaler divider)

/* Bit definition for IWDG_RLR register
 ************************************************************************************************/
#define  IWDG_RLR_RL             ((uint16_t)0x0FFF)      ///< Watchdog counter reload value

/* Bit definition for IWDG_SR register
 ************************************************************************************************/
#define  IWDG_SR_PVU             ((uint8_t)0x01)         ///< Watchdog prescaler value update
#define  IWDG_SR_RVU             ((uint8_t)0x02)         ///< Watchdog counter reload value update

/************************************************************************************************
 *                                                                                              *
 *                                      Window WATCHDOG                                         *
 *                                                                                              *
 ************************************************************************************************/
 
/* Bit definition for WWDG_CR register
 ************************************************************************************************/
#define  WWDG_CR_T               ((uint8_t)0x7F)         ///< T[6:0] bits (7-Bit counter (MSB to LSB))
#define  WWDG_CR_WDGA            ((uint8_t)0x80)         ///< Activation bit

/* Bit definition for WWDG_CFR register
 ************************************************************************************************/
#define  WWDG_CFR_W              ((uint16_t)0x007F)      ///< W[6:0] bits (7-bit window value)
#define  WWDG_CFR_WDGTB          ((uint16_t)0x0180)      ///< WDGTB[1:0] bits (Timer Base)
#define  WWDG_CFR_EWI            ((uint16_t)0x0200)      ///< Early Wakeup Interrupt

/* Bit definition for WWDG_SR register
 ************************************************************************************************/
#define  WWDG_SR_EWIF            ((uint8_t)0x01)         ///< Early Wakeup Interrupt Flag

/************************************************************************************************
 *                                                                                              *
 *                                         Debug MCU                                            *
 *                                                                                              *
 ************************************************************************************************/
 
/* Bit definition for DBGMCU_IDCODE register
 ************************************************************************************************/
#define  DBGMCU_IDCODE_DEV_ID    ((uint32_t)0x00000FFF)  ///< Device Identifier
#define  DBGMCU_IDCODE_REV_ID    ((uint32_t)0xFFFF0000)  ///< REV_ID[15:0] bits (Revision Identifier)

/* Bit definition for DBGMCU_CR register
 ************************************************************************************************/
#define  DBGMCU_CR_DBG_SLEEP     ((uint32_t)0x00000001)  ///< Debug Sleep Mode
#define  DBGMCU_CR_DBG_STOP      ((uint32_t)0x00000002)  ///< Debug Stop Mode
#define  DBGMCU_CR_DBG_STANDBY   ((uint32_t)0x00000004)  ///< Debug Standby mode
#define  DBGMCU_CR_TRACE_IOEN    ((uint32_t)0x00000020)  ///< Trace Pin Assignment Control
#define  DBGMCU_CR_TRACE_MODE    ((uint32_t)0x000000C0)  ///< TRACE_MODE[1:0] bits (Trace Pin Assignment Control)

#define  DBGMCU_CR_DBG_IWDG_STOP          ((uint32_t)0x00000100) ///< Debug Independent Watchdog stopped when Core is halted */
#define  DBGMCU_CR_DBG_WWDG_STOP          ((uint32_t)0x00000200) ///< Debug Window Watchdog stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM1_STOP          ((uint32_t)0x00000400) ///< TIM1 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM2_STOP          ((uint32_t)0x00000800) ///< TIM2 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM3_STOP          ((uint32_t)0x00001000) ///< TIM3 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM4_STOP          ((uint32_t)0x00002000) ///< TIM4 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_CAN1_STOP          ((uint32_t)0x00004000) ///< Debug CAN1 stopped when Core is halted */
#define  DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT ((uint32_t)0x00008000) ///< SMBUS timeout mode stopped when Core is halted */
#define  DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT ((uint32_t)0x00010000) ///< SMBUS timeout mode stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM8_STOP          ((uint32_t)0x00020000) ///< TIM8 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM5_STOP          ((uint32_t)0x00040000) ///< TIM5 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM6_STOP          ((uint32_t)0x00080000) ///< TIM6 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_TIM7_STOP          ((uint32_t)0x00100000) ///< TIM7 counter stopped when core is halted */
#define  DBGMCU_CR_DBG_CAN2_STOP          ((uint32_t)0x00200000) ///< Debug CAN2 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM15_STOP         ((uint32_t)0x00400000) ///< Debug TIM15 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM16_STOP         ((uint32_t)0x00800000) ///< Debug TIM16 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM17_STOP         ((uint32_t)0x01000000) ///< Debug TIM17 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM12_STOP         ((uint32_t)0x02000000) ///< Debug TIM12 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM13_STOP         ((uint32_t)0x04000000) ///< Debug TIM13 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM14_STOP         ((uint32_t)0x08000000) ///< Debug TIM14 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM9_STOP          ((uint32_t)0x10000000) ///< Debug TIM9 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM10_STOP         ((uint32_t)0x20000000) ///< Debug TIM10 stopped when Core is halted */
#define  DBGMCU_CR_DBG_TIM11_STOP         ((uint32_t)0x40000000) ///< Debug TIM11 stopped when Core is halted */

/* МАКРОСЫ
 ************************************************************************************************/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32F103X_H */
/*
** END OF FILE
***********************************************************************************************/
