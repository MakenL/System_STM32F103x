/** @file     system_stm32f103x.h
    @author   Lyga Maksim
    @version  V1.0
    @date     20.07.2015
 */
 
#ifndef __SYSTEM_STM32F103X_H
#define __SYSTEM_STM32F103X_H
 
#ifdef __cplusplus
 extern "C" {
#endif 
 
/* ПОДКЛЮЧЕНИЕ ЗАГОЛОВОЧНЫХ ФАЙЛОВ
 *************************************************************************************************/	
 
#include "stm32f103x.h"
	 
/* МАСТЕР КОНФИГУРАЦИИ
 *************************************************************************************************/
	 
//********************** <<< Use Configuration Wizard in Context Menu >>> **********************
// <h> Clock Configuration
//   <e0> HSEON: External High Speed clock enable 
//   <i> Default: HSE oscillator OFF (register RCC_CR)
//     <o7> External High Speed Clock [Hz] <4000000-16000000>
//     <i> clock value for the used External High Speed Clock (4MHz <= HSE <= 16MHz).
//     <i> Default: 8000000  (8MHz)
//     <o9> Time out for HSE start up <1000-32000>
//     <i> Default: 1280
//     <q1> CSSON: Clock Security System enable
//     <i> Default: Clock detector OFF (register RCC_CR)
//     <q2> HSEBYP: External High Speed clock Bypass
//     <i> Default: HSE oscillator not bypassed (register RCC_CR)
//   </e>
//   <e3> HSION: Internal High Speed clock enable
//   <i> Default: internal 8MHz RC oscillator ON (register RCC_CR)
//     <o4.3..7> HSITRIM: Internal High Speed clock trimming  <0-31>
//     <i> Default: 0x10 (register RCC_CR)
//   </e>
//   <e5> PLLON: PLL enable         
//   <i> Default: PLL Disabled (register RCC_CR)
//     <o6.17> PLLXTPRE: HSE divider for PLL entry
//     <i> Default: HSE (register RCC_CFGR)
//                   <0=> HSE
//                   <1=> HSE / 2
//     <o6.16> PLLSRC: PLL entry clock source         
//     <i> Default: HSI/2 (register RCC_CFGR)
//                   <0=> HSI / 2
//                   <1=> HSE (PLLXTPRE output)
//     <o6.18..21> PLLMUL: PLL Multiplication Factor
//     <i> Default: PLLSRC * 2 (register RCC_CFGR)
//                   <0=> PLLSRC * 2
//                   <1=> PLLSRC * 3
//                   <2=> PLLSRC * 4
//                   <3=> PLLSRC * 5
//                   <4=> PLLSRC * 6
//                   <5=> PLLSRC * 7
//                   <6=> PLLSRC * 8
//                   <7=> PLLSRC * 9
//                   <8=> PLLSRC * 10
//                   <9=> PLLSRC * 11
//                   <10=> PLLSRC * 12
//                   <11=> PLLSRC * 13
//                   <12=> PLLSRC * 14
//                   <13=> PLLSRC * 15
//                   <14=> PLLSRC * 16
//   </e>
//   <o6.0..1> SW: System Clock Switch
//   <i> Default: SYSCLK = HSI (register RCC_CFGR)
//                   <0=> SYSCLK = HSI
//                   <1=> SYSCLK = HSE
//                   <2=> SYSCLK = PLLCLK
//   <h> Configuration prescalers
//   <o6.4..7> HPRE: AHB prescaler 
//   <i> Default: HCLK = SYSCLK (register RCC_CFGR)
//                   <0=> HCLK = SYSCLK
//                   <8=> HCLK = SYSCLK / 2
//                   <9=> HCLK = SYSCLK / 4
//                   <10=> HCLK = SYSCLK / 8
//                   <11=> HCLK = SYSCLK / 16
//                   <12=> HCLK = SYSCLK / 64
//                   <13=> HCLK = SYSCLK / 128
//                   <14=> HCLK = SYSCLK / 256
//                   <15=> HCLK = SYSCLK / 512
//   <o6.11..13> PPRE2: APB High speed prescaler (APB2)
//   <i> Default: PCLK2 = HCLK (register RCC_CFGR)
//                   <0=> PCLK2 = HCLK
//                   <4=> PCLK2 = HCLK / 2 
//                   <5=> PCLK2 = HCLK / 4 
//                   <6=> PCLK2 = HCLK / 8 
//                   <7=> PCLK2 = HCLK / 16 
//   <o6.8..10> PPRE1: APB Low speed prescaler (APB1) 
//   <i> Default: PCLK1 = HCLK (register RCC_CFGR)
//                   <0=> PCLK1 = HCLK
//                   <4=> PCLK1 = HCLK / 2 
//                   <5=> PCLK1 = HCLK / 4 
//                   <6=> PCLK1 = HCLK / 8 
//                   <7=> PCLK1 = HCLK / 16 
//   <o6.22> USBPRE: USB prescaler
//   <i> Default: USBCLK = PLLCLK / 1.5 (register RCC_CFGR)
//                   <0=> USBCLK = PLLCLK / 1.5
//                   <1=> USBCLK = PLLCLK
//   <o6.14..15> ADCPRE: ADC prescaler
//   <i> Default: ADCCLK=PCLK2 / 2 (register RCC_CFGR)
//                   <0=> ADCCLK = PCLK2 / 2
//                   <1=> ADCCLK = PCLK2 / 4
//                   <2=> ADCCLK = PCLK2 / 6
//                   <3=> ADCCLK = PCLK2 / 8
//   </h>
//   <o6.24..26> MCO: Microcontroller Clock Output   
//   <i> Default: MCO = noClock (register RCC_CFGR)
//                   <0=> MCO = noClock
//                   <4=> MCO = SYSCLK
//                   <5=> MCO = HSI
//                   <6=> MCO = HSE
//                   <7=> MCO = PLLCLK / 2
// </h> End of Clock Configuration

#define __HSEON        1
#define __CSSON        1
#define __HSEBYP       0
#define __HSION        1
#define __HSITRIM      0x80
#define __PLLON        1
#define __RCC_CFGR     0x001D4402
#define __HSE_CLOCK    8000000
#define __HSI_CLOCK    8000000
#define __HSE_TIMEOUT  ((uint16_t)1300)

//<e0> Real Time Clock Configuration
//  <o1.8..9> RTC clock source selection
//  <i> Default: No Clock
//               <0=> No Clock
//               <1=> RTCCLK = LSE (32,768kHz)
//               <2=> RTCCLK = LSI (32 kHz)
//               <3=> RTCCLK = HSE/128
//  <o2> RTC period [ms] <10-1000:10>
//  <i> Set the timer period for Real Time Clock.
//  <i> Default: 1000  (1s)
//  <h> RTC interrupts
//      <o3.0> RTC_CRH.SECIE: Second interrupt enabled
//      <o3.1> RTC_CRH.ALRIE: Alarm interrupt enabled
//      <o3.2> RTC_CRH.OWIE: Overflow interrupt enabled
//  </h>
//</e>

#define __RTC_USED        1
#define __RCC_BDCR        0x00000100
#define __RTC_PERIOD      1000
#define __RTC_CRH         0x0001
#define __LSE_CLOCK       32768
#define __LSI_CLOCK       32000

// <h> Embedded Flash Configuration
//   <o0.0..2> LATENCY: Latency
//   <i> Default: 2 wait states
//                   <0=> 0 wait states
//                   <1=> 1 wait states
//                   <2=> 2 wait states
//   <o0.3> HLFCYA: Flash Half Cycle Access Enable
//   <o0.4> PRFTBE: Prefetch Buffer Enable
// </h>
#define __FLASH_ACR      ((uint32_t)0x00000012)

// <h> Nested Vectored Interrupt Controller (NVIC)
//   <o0> Vector Table Base
//   <i> Default: FLASH
//                   <0=> FLASH
//                   <1=> RAM
//   <o1> Vector Table Offset <0x0-0x1FFFFFC0:0x200>
//   <i> Vector Table base offset field. This value must be a multiple of 0x200
//   <i> Default: 0x00000000 
// </h>
#define __VECT_TAB_BASE   0
#define __VECT_TAB_OFFSET ((uint32_t)0x00000000)

// <e0> System Timer Configuration
//   <o1.2> System Timer clock source selection
//   <i> Default: SYSTICKCLK = HCLK/8
//                     <0=> SYSTICKCLK = HCLK/8
//                     <1=> SYSTICKCLK = HCLK
//   <o1.1> System Timer interrupt enabled
//   <o2> SYSTICK max period [ms] <100-1000>
//   <i> Set the timer max period for System Timer.
//   <i> Default: 1000 (1s)
// </e>
#define __SYSTICK_SETUP           1
#define __SYSTICK_CTRL            0x00000006
#define __SYSTICK_MAXPERIOD       100
#define __SYSTEM_TIMER_COUNT      5

//<h> GPIO Configuration
//  <e0.31> GPIOA port
//  <i> Port GPIOA clock enable. Port GPIOA is relevant to STM32F103Rx, STM32F103Vx, STM32F103Zx.
//      <e0.0> GPIOA0 pin
//          <o1.0..3> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.0> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.0> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.1> GPIOA1 pin
//          <o1.4..7> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.1> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.1> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.2> GPIOA2 pin
//          <o1.8..11> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.2> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.2> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.3> GPIOA3 pin
//          <o1.12..15> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.3> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.3> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.4> GPIOA4 pin
//          <o1.16..19> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.4> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.4> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.5> GPIOA5 pin
//          <o1.20..23> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.5> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.5> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.6> GPIOA6 pin
//          <o1.24..27> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.6> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.6> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.7> GPIOA7 pin
//          <o1.28..31> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.7> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.7> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.8> GPIOA8 pin
//          <o2.0..3> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.8> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.8> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.9> GPIOA9 pin
//          <o2.4..7> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.9> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.9> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.10> GPIOA10 pin
//          <o2.8..11> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.10> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.10> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.11> GPIOA11 pin
//          <o2.12..15> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.11> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.11> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.12> GPIOA12 pin
//          <o2.16..19> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.12> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.12> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.13> GPIOA13 pin
//          <o2.20..23> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.13> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.13> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.14> GPIOA14 pin
//          <o2.24..27> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.14> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.14> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.15> GPIOA15 pin
//          <o2.28..31> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.15> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.15> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e> </e>

#define __GPIOA_USED   0x8000060F
#define __GPIOA_CRL    0x44444988
#define __GPIOA_CRH    0x44444494
#define __GPIOA_ODR    0x00000003
#define __GPIOA_LCKR   0x00000000

//  <e0.31> GPIOB port
//  <i> Port GPIOB clock enable. Port GPIOB is relevant to STM32F103Rx, STM32F103Vx, STM32F103Zx.
//      <e0.0> GPIOB0 pin
//          <o1.0..3> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.0> Start output level 
//                <0=>0 - Low <1=>1 - High
//          <o4.0> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.1> GPIOB1 pin
//          <o1.4..7> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.1> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.1> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.2> GPIOB2 pin
//          <o1.8..11> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.2> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.2> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.3> GPIOB3 pin
//          <o1.12..15> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.3> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.3> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.4> GPIOB4 pin
//          <o1.16..19> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.4> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.4> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.5> GPIOB5 pin
//          <o1.20..23> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.5> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.5> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.6> GPIOB6 pin
//          <o1.24..27> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.6> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.6> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.7> GPIOB7 pin
//          <o1.28..31> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.7> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.7> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.8> GPIOB8 pin
//          <o2.0..3> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.8> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.8> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.9> GPIOB9 pin
//          <o2.4..7> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.9> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.9> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.10> GPIOB10 pin
//          <o2.8..11> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.10> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.10> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.11> GPIOB11 pin
//          <o2.12..15> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.11> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.11> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.12> GPIOB12 pin
//          <o2.16..19> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.12> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.12> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.13> GPIOB13 pin
//          <o2.20..23> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.13> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.13> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.14> GPIOB14 pin
//          <o2.24..27> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.14> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.14> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.15> GPIOB15 pin
//          <o2.28..31> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.15> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.15> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e> </e>

#define __GPIOB_USED   0x80000003
#define __GPIOB_CRL    0x44444488
#define __GPIOB_CRH    0x4444FF44
#define __GPIOB_ODR    0x00000003
#define __GPIOB_LCKR   0x00000000

//  <e0.31> GPIOC port
//  <i> Port GPIOC clock enable. Port GPIOC is relevant to STM32F103Rx, STM32F103Vx, STM32F103Zx.
//      <e0.0> GPIOC0 pin
//          <o1.0..3> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.0> Start output level 
//                <0=>0 - Low <1=>1 - High
//          <o4.0> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.1> GPIOC1 pin
//          <o1.4..7> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.1> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.1> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.2> GPIOC2 pin
//          <o1.8..11> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.2> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.2> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.3> GPIOC3 pin
//          <o1.12..15> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.3> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.3> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.4> GPIOC4 pin
//          <o1.16..19> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.4> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.4> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.5> GPIOC5 pin
//          <o1.20..23> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.5> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.5> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.6> GPIOC6 pin
//          <o1.24..27> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.6> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.6> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.7> GPIOC7 pin
//          <o1.28..31> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.7> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.7> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.8> GPIOC8 pin
//          <o2.0..3> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.8> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.8> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.9> GPIOC9 pin
//          <o2.4..7> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.9> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.9> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.10> GPIOC10 pin
//          <o2.8..11> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.10> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.10> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.11> GPIOC11 pin
//          <o2.12..15> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.11> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.11> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.12> GPIOC12 pin
//          <o2.16..19> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.12> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.12> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.13> GPIOC13 pin
//          <o2.20..23> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.13> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.13> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.14> GPIOC14 pin
//          <o2.24..27> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.14> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.14> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.15> GPIOC15 pin
//          <o2.28..31> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.15> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.15> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e> </e>

#define __GPIOC_USED   0x80001FC0
#define __GPIOC_CRL    0x88444444
#define __GPIOC_CRH    0x44488888
#define __GPIOC_ODR    0x00001FC0
#define __GPIOC_LCKR   0x00000000

//  <e0.31> GPIOD port
//  <i> Port GPIOD clock enable. Port GPIOD is relevant to STM32F103Vx, STM32F103Zx.
//      <e0.0> GPIOD0 pin
//          <o1.0..3> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.0> Start output level 
//                <0=>0 - Low <1=>1 - High
//          <o4.0> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.1> GPIOD1 pin
//          <o1.4..7> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.1> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.1> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.2> GPIOD2 pin
//          <o1.8..11> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.2> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.2> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.3> GPIOD3 pin
//          <o1.12..15> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.3> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.3> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.4> GPIOD4 pin
//          <o1.16..19> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.4> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.4> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.5> GPIOD5 pin
//          <o1.20..23> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.5> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.5> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.6> GPIOD6 pin
//          <o1.24..27> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.6> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.6> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.7> GPIOD7 pin
//          <o1.28..31> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.7> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.7> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.8> GPIOD8 pin
//          <o2.0..3> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.8> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.8> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.9> GPIOD9 pin
//          <o2.4..7> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.9> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.9> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.10> GPIOD10 pin
//          <o2.8..11> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.10> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.10> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.11> GPIOD11 pin
//          <o2.12..15> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.11> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.11> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.12> GPIOD12 pin
//          <o2.16..19> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.12> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.12> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.13> GPIOD13 pin
//          <o2.20..23> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.13> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.13> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.14> GPIOD14 pin
//          <o2.24..27> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.14> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.14> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.15> GPIOD15 pin
//          <o2.28..31> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.15> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.15> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e> </e>

#define __GPIOD_USED   0x8000EFB3
#define __GPIOD_CRL    0xB4BB44BB
#define __GPIOD_CRH    0xBB24BBBB
#define __GPIOD_ODR    0x00002000
#define __GPIOD_LCKR   0x00000000

//  <e0.31> GPIOE port
//  <i> Port GPIOE clock enable. Port GPIOE is relevant to STM32F103Vx, STM32F103Zx.
//      <e0.0> GPIOE0 pin
//          <o1.0..3> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.0> Start output level 
//                <0=>0 - Low <1=>1 - High
//          <o4.0> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.1> GPIOE1 pin
//          <o1.4..7> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.1> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.1> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.2> GPIOE2 pin
//          <o1.8..11> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.2> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.2> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.3> GPIOE3 pin
//          <o1.12..15> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.3> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.3> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.4> GPIOE4 pin
//          <o1.16..19> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.4> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.4> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.5> GPIOE5 pin
//          <o1.20..23> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.5> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.5> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.6> GPIOE6 pin
//          <o1.24..27> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.6> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.6> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.7> GPIOE7 pin
//          <o1.28..31> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.7> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.7> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.8> GPIOE8 pin
//          <o2.0..3> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.8> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.8> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.9> GPIOE9 pin
//          <o2.4..7> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.9> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.9> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.10> GPIOE10 pin
//          <o2.8..11> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.10> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.10> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.11> GPIOE11 pin
//          <o2.12..15> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.11> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.11> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.12> GPIOE12 pin
//          <o2.16..19> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.12> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.12> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.13> GPIOE13 pin
//          <o2.20..23> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.13> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.13> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.14> GPIOE14 pin
//          <o2.24..27> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.14> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.14> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e>
//      <e0.15> GPIOE15 pin
//          <o2.28..31> Pin used as 
//                <0=>Analog Input
//                <4=>Floating (HiZ) Input
//                <8=>Input with pull-up / pull-down
//                <2=>General Purpose Output push-pull (max speed  2MHz)
//                <1=>General Purpose Output push-pull (max speed 10MHz)
//                <3=>General Purpose Output push-pull (max speed 50MHz)
//                <6=>General Purpose Output open-drain (max speed  2MHz)
//                <5=>General Purpose Output open-drain (max speed 10MHz)
//                <7=>General Purpose Output open-drain (max speed 50MHz)
//                <10=>Alternate Function push-pull (max speed  2MHz)
//                <9=>Alternate Function push-pull (max speed 10MHz)
//                <11=>Alternate Function push-pull (max speed 50MHz)
//                <14=>Alternate Function open-drain (max speed  2MHz)
//                <13=>Alternate Function open-drain (max speed 10MHz)
//                <15=>Alternate Function open-drain (max speed 50MHz)
//          <o3.15> Start output level
//                <0=>0 - Low <1=>1 - High
//          <o4.15> Locked setup 
//               <0=>0 - Unlocked <1=>1 - Locked
//      </e> </e>
//</h>

#define __GPIOE_USED   0x8000FF82
#define __GPIOE_CRL    0xB2244424
#define __GPIOE_CRH    0xBBBBBBBB
#define __GPIOE_ODR    0x00000062
#define __GPIOE_LCKR   0x00000000

// <e0> Alternate Function enable
// <i> Set/reset bit AFIOEN of register APB2ENR. Alternative funftion clock enable. 
//   <h> Alternate Function remap Configuration
//     <o1.0> SPI1 remapping
//       <i> Default: No Remap (NSS/PA4, SCK/PA5, MISO/PA6, MOSI/PA7)
//                <0=> No Remap (NSS/PA4, SCK/PA5, MISO/PA6, MOSI/PA7)
//                <1=> Remap (NSS/PA15, SCK/PB3, MISO/PB4, MOSI/PB5)
//     <o1.1> I2C1 remapping
//       <i> Default: No Remap (SCL/PB6, SDA/PB7)
//                <0=> No Remap (SCL/PB6, SDA/PB7)
//                <1=> Remap (SCL/PB8, SDA/PB9)
//     <o1.2> USART1 remapping
//       <i> Default: No Remap (TX/PA9, RX/PA10)
//                <0=> No Remap (TX/PA9, RX/PA10)
//                <1=> Remap (TX/PB6, RX/PB7)
//     <o1.3> USART2 remapping
//       <i> Default: No Remap (CTS/PA0, RTS/PA1, TX/PA2, RX/PA3, CK/PA4)
//                <0=> No Remap (CTS/PA0, RTS/PA1, TX/PA2, RX/PA3, CK/PA4)
//                <1=> Remap (CTS/PD3, RTS/PD4, TX/PD5, RX/PD6, CK/PD7)
//     <o1.4..5> USART3 remapping
//       <i> Default: No Remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14)
//                <0=> No Remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14)
//                <1=> Partial Remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14)
//                <3=> Full Remap (TX/PD8, RX/PD9, CK/PD10, CTS/PD11, RTS/PD12)
//     <o1.6..7> TIM1 remapping
//       <i> Default: No Remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15)
//                <0=> No Remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15)
//                <1=> Partial Remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1)
//                <3=> Full Remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12)
//     <o1.8..9> TIM2 remapping
//       <i> Default: No Remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3)
//                <0=> No Remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3)
//                <1=> Partial Remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3)
//                <2=> Partial Remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11)
//                <3=> Full Remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11)
//     <o1.10..11> TIM3 remapping
//       <i> Default: No Remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1)
//                <0=> No Remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1)
//                <2=> Partial Remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1)
//                <3=> Full Remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9)
//     <o1.12> TIM4 remapping
//       <i> Default: No Remap (CH1/PB6, CH2/PB7, CH3/PB8, CH4/PB9)
//                <0=> No Remap (CH1/PB6, CH2/PB7, CH3/PB8, CH4/PB9)
//                <1=> Remap (CH1/PD12, CH2/PD13, CH3/PD14, CH4/PD15)
//     <o1.13..14> CAN remapping
//       <i> Default: No Remap (CANRX/PA11, CANTX/PA12)
//                <0=> No Remap (CANRX/PA11, CANTX/PA12)
//                <2=> Remap (CANRX/PB8, CANTX/PB9)
//                <3=> Remap (CANRX/PD0, CANTX/PPD1)
//     <o1.15> PD01 remapping
//       <i> Default: No Remap
//                <0=> No Remap
//                <1=> Remap (PD0/OSCIN, PD1/OSC_OUT)
//     <o1.24..26> Serial wire JTAG configuration
//       <i> Default: Full SWJ (JTAG-DP + SW-DP)
//                <0=> Reset State: Full SWJ (JTAG-DP + SW-DP)
//                <1=> Full SWJ (JTAG-DP + SW-DP) but without NJTRST
//                <2=> JTAG-DP Disabled and SW-DP Enabled
//                <4=> JTAG-DP Disabled and SW-DP Disabled
//   </h>
// </e>

#define __GPIO_AFIOEN  1
#define __AFIO_MAPR    0x00000C00

//<h> USART Configuration
//  <e0> USART1 enable
//      <o1> Baudrate 
//            <9600=>    9600 Baud
//            <14400=>   14400 Baud
//            <19200=>   19200 Baud
//            <28800=>   28800 Baud
//            <38400=>   38400 Baud
//            <56000=>   56000 Baud
//            <57600=>   57600 Baud
//            <115200=>  115200 Baud
//      <o2.12> Data Bits 
//            <0=> 8 Data Bits
//            <1=> 9 Data Bits
//      <o3.12..13> Stop Bits
//            <1=> 0.5 Stop Bit
//            <0=>   1 Stop Bit
//            <3=> 1.5 Stop Bits
//            <2=>   2 Stop Bits
//      <o2.9..10> Parity 
//            <0=> No Parity
//            <2=> Even Parity
//            <3=> Odd Parity
//      <o4> Softwire receiver (RX) buffer size
//            <4=>   4 byte
//            <8=>   8 byte
//            <16=>  16 byte
//            <32=>  32 byte
//            <64=>  64 byte
//            <128=> 128 byte
//            <256=> 256 byte
//            <512=> 512 byte
//            <1024=> 1024 byte
//      <o5> Softwire transmitter (TX) buffer size
//            <4=>   4 byte
//            <8=>   8 byte
//            <16=>  16 byte
//            <32=>  32 byte
//            <64=>  64 byte
//            <128=> 128 byte
//            <256=> 256 byte
//            <512=> 512 byte
//            <1024=> 1024 byte
//  </e>

#define __USART1_USED      1
#define __USART1_BAUDRATE  115200
#define __USART1_CR1       0x00000000
#define __USART1_CR2       0x00000000
#define __USART1_RX_BUF    128
#define __USART1_TX_BUF    1024

//  <e0> USART2 enable
//      <o1> Baudrate 
//            <9600=>    9600 Baud
//            <14400=>   14400 Baud
//            <19200=>   19200 Baud
//            <28800=>   28800 Baud
//            <38400=>   38400 Baud
//            <56000=>   56000 Baud
//            <57600=>   57600 Baud
//            <115200=>  115200 Baud
//      <o2.12> Data Bits 
//            <0=> 8 Data Bits
//            <1=> 9 Data Bits
//      <o3.12..13> Stop Bits
//            <1=> 0.5 Stop Bit
//            <0=>   1 Stop Bit
//            <3=> 1.5 Stop Bits
//            <2=>   2 Stop Bits
//      <o2.9..10> Parity 
//            <0=> No Parity
//            <2=> Even Parity
//            <3=> Odd Parity
//      <o4> Softwire receiver (RX) buffer size
//            <4=>   4 byte
//            <8=>   8 byte
//            <16=>  16 byte
//            <32=>  32 byte
//            <64=>  64 byte
//            <128=> 128 byte
//            <256=> 256 byte
//            <512=> 512 byte
//            <1024=> 1024 byte
//      <o5> Softwire transmitter (TX) buffer size
//            <4=>   4 byte
//            <8=>   8 byte
//            <16=>  16 byte
//            <32=>  32 byte
//            <64=>  64 byte
//            <128=> 128 byte
//            <256=> 256 byte
//            <512=> 512 byte
//            <1024=> 1024 byte
//  </e>

#define __USART2_USED      1
#define __USART2_BAUDRATE  115200
#define __USART2_CR1       0x00000000
#define __USART2_CR2       0x00000000
#define __USART2_RX_BUF    1024
#define __USART2_TX_BUF    1024

//  <e0> USART3 enable
//      <o1> Baudrate 
//            <9600=>    9600 Baud
//            <14400=>   14400 Baud
//            <19200=>   19200 Baud
//            <28800=>   28800 Baud
//            <38400=>   38400 Baud
//            <56000=>   56000 Baud
//            <57600=>   57600 Baud
//            <115200=>  115200 Baud
//      <o2.12> Data Bits 
//            <0=> 8 Data Bits
//            <1=> 9 Data Bits
//      <o3.12..13> Stop Bits
//            <1=> 0.5 Stop Bit
//            <0=>   1 Stop Bit
//            <3=> 1.5 Stop Bits
//            <2=>   2 Stop Bits
//      <o2.9..10> Parity 
//            <0=> No Parity
//            <2=> Even Parity
//            <3=> Odd Parity
//      <o4> Softwire receiver (RX) buffer size
//            <4=>   4 byte
//            <8=>   8 byte
//            <16=>  16 byte
//            <32=>  32 byte
//            <64=>  64 byte
//            <128=> 128 byte
//            <256=> 256 byte
//            <512=> 512 byte
//            <1024=> 1024 byte
//      <o5> Softwire transmitter (TX) buffer size
//            <4=>   4 byte
//            <8=>   8 byte
//            <16=>  16 byte
//            <32=>  32 byte
//            <64=>  64 byte
//            <128=> 128 byte
//            <256=> 256 byte
//            <512=> 512 byte
//            <1024=> 1024 byte
//  </e>

#define __USART3_USED      0
#define __USART3_BAUDRATE  115200
#define __USART3_CR1       0x00000000
#define __USART3_CR2       0x00000000
#define __USART3_RX_BUF    8
#define __USART3_TX_BUF    8

//</h>
//<h> I2C Configuration
//  <e0> I2C1 enable
//  <i> Master mode and 7-bit addressing only.
//  <i> I2C is used with polling.
//      <o1> Speed 
//            <100000=> Sm 100 kHz
//            <400000=> Fm 400 kHz
//  </e>
//  <e2> I2C2 enable
//  <i> Master mode and 7-bit addressing only.
//  <i> I2C is used with polling.
//      <o3> Speed 
//            <100000=> Sm 100 kHz
//            <400000=> Fm 400 kHz
//  </e>
//</h>

#define __I2C1_USED    0
#define __I2C1_SPEED   100000
#define __I2C2_USED    0
#define __I2C2_SPEED   100000



//**************************** <<< end of configuration section >>> ****************************

#if   ((__RCC_CFGR & 0x00000003) == 0x00)
	#if __HSION
	  // HSI is used as system clock
    #define __SYSCLK_CLOCK   __HSI_CLOCK
	#else
	  #error "HSI is disable (see configuration wizard)"
	#endif
#elif ((__RCC_CFGR & 0x00000003) == 0x01)
	#if __HSEON
	  // HSE is used as system clock
    #define __SYSCLK_CLOCK   __HSE_CLOCK
	#else
	  #error "HSE is disable (see configuration wizard)"
	#endif
#elif ((__RCC_CFGR & 0x00000003) == 0x02)
	#if __PLLON
    // PLL is used as system clock
    #define __PLLMULL  (((__RCC_CFGR & 0x003C0000) >> 18) + 2)
	#else
	  #error "PLL is disable (see configuration wizard)"
	#endif

  // HSE is PLL clock source
  #if (__RCC_CFGR & 0x00010000)
	
		#if __HSEON
	    // HSE/2 is used
      #if (__RCC_CFGR & 0x00020000)
        // SYSCLK = HSE/2 * pllmull		
        #define __SYSCLK  ((__HSE_CLOCK >> 1) * __PLLMULL)
      // HSE is used
			#else
		    // SYSCLK = HSE   * pllmul
        #define __SYSCLK  ((__HSE_CLOCK >> 0) * __PLLMULL)
      #endif 
	  #else
	    #error "HSE is disable (see configuration wizard)"
	  #endif
	// HSI/2 is PLL clock source
  #else
		#if __HSION
	    // SYSCLK = HSI/2 * pllmul
      #define __SYSCLK  ((__HSI_CLOCK >> 1) * __PLLMULL)
	  #else
	    #error "HSI is disable (see configuration wizard)"
	  #endif
  #endif
#else
   #error "Value of System Clock Switch is not correct (see configuration wizard)"
#endif

#if __SYSCLK > 72000000
	#error "Value of SYSCLK is more than 72 MHz (see configuration wizard)"
#else
		
  #if (((__RCC_CFGR & 0x000000F0) >> 4) & 0x08)
    #define __HCLK  (__SYSCLK >> ((((__RCC_CFGR & 0x000000F0) >> 4) & 0x07)+1))
  #else
    #define __HCLK  (__SYSCLK)
  #endif
	
  #if (((__RCC_CFGR & 0x00000700) >> 8) & 0x04)
    #define __PCLK1 (__HCLK >> ((((__RCC_CFGR & 0x00000700) >> 8) & 0x03) + 1))
  #else
    #define __PCLK1 (__HCLK)
  #endif
			
  #if (((__RCC_CFGR & 0x00003800) >> 11) & 0x04)
    #define __PCLK2 (__HCLK >> ((((__RCC_CFGR & 0x00003800) >> 11) & 0x03)+1))
  #else
    #define __PCLK2 (__HCLK)
  #endif
		
	#if __HCLK > 72000000
	  #error "Value of HCLK is more than 72 MHz (see configuration wizard)"
  #endif
		
	#if __PCLK1 > 36000000
	  #error "Value of PCLK1 is more than 36 MHz (see configuration wizard)"
  #endif		
		
#endif

/* КОНСТАНТЫ
 *************************************************************************************************/

/* МАКРОСЫ
 ************************************************************************************************/

// Вспомогательные макросы для макроса BIT_BAND_REG
#define _MASK_TO_BIT31(A)        (A==0x80000000)? 31 : 0
#define _MASK_TO_BIT30(A)        (A==0x40000000)? 30 : _MASK_TO_BIT31(A)
#define _MASK_TO_BIT29(A)        (A==0x20000000)? 29 : _MASK_TO_BIT30(A)
#define _MASK_TO_BIT28(A)        (A==0x10000000)? 28 : _MASK_TO_BIT29(A)
#define _MASK_TO_BIT27(A)        (A==0x08000000)? 27 : _MASK_TO_BIT28(A)
#define _MASK_TO_BIT26(A)        (A==0x04000000)? 26 : _MASK_TO_BIT27(A)
#define _MASK_TO_BIT25(A)        (A==0x02000000)? 25 : _MASK_TO_BIT26(A)
#define _MASK_TO_BIT24(A)        (A==0x01000000)? 24 : _MASK_TO_BIT25(A)
#define _MASK_TO_BIT23(A)        (A==0x00800000)? 23 : _MASK_TO_BIT24(A)
#define _MASK_TO_BIT22(A)        (A==0x00400000)? 22 : _MASK_TO_BIT23(A)
#define _MASK_TO_BIT21(A)        (A==0x00200000)? 21 : _MASK_TO_BIT22(A)
#define _MASK_TO_BIT20(A)        (A==0x00100000)? 20 : _MASK_TO_BIT21(A)
#define _MASK_TO_BIT19(A)        (A==0x00080000)? 19 : _MASK_TO_BIT20(A)
#define _MASK_TO_BIT18(A)        (A==0x00040000)? 18 : _MASK_TO_BIT19(A)
#define _MASK_TO_BIT17(A)        (A==0x00020000)? 17 : _MASK_TO_BIT18(A)
#define _MASK_TO_BIT16(A)        (A==0x00010000)? 16 : _MASK_TO_BIT17(A)
#define _MASK_TO_BIT15(A)        (A==0x00008000)? 15 : _MASK_TO_BIT16(A)
#define _MASK_TO_BIT14(A)        (A==0x00004000)? 14 : _MASK_TO_BIT15(A)
#define _MASK_TO_BIT13(A)        (A==0x00002000)? 13 : _MASK_TO_BIT14(A)
#define _MASK_TO_BIT12(A)        (A==0x00001000)? 12 : _MASK_TO_BIT13(A)
#define _MASK_TO_BIT11(A)        (A==0x00000800)? 11 : _MASK_TO_BIT12(A)
#define _MASK_TO_BIT10(A)        (A==0x00000400)? 10 : _MASK_TO_BIT11(A)
#define _MASK_TO_BIT09(A)        (A==0x00000200)? 9  : _MASK_TO_BIT10(A)
#define _MASK_TO_BIT08(A)        (A==0x00000100)? 8  : _MASK_TO_BIT09(A)
#define _MASK_TO_BIT07(A)        (A==0x00000080)? 7  : _MASK_TO_BIT08(A)
#define _MASK_TO_BIT06(A)        (A==0x00000040)? 6  : _MASK_TO_BIT07(A)
#define _MASK_TO_BIT05(A)        (A==0x00000020)? 5  : _MASK_TO_BIT06(A)
#define _MASK_TO_BIT04(A)        (A==0x00000010)? 4  : _MASK_TO_BIT05(A)
#define _MASK_TO_BIT03(A)        (A==0x00000008)? 3  : _MASK_TO_BIT04(A)
#define _MASK_TO_BIT02(A)        (A==0x00000004)? 2  : _MASK_TO_BIT03(A)
#define _MASK_TO_BIT01(A)        (A==0x00000002)? 1  : _MASK_TO_BIT02(A)
#define _MASK_TO_BIT(A)          (A==0x00000001)? 0  : _MASK_TO_BIT01(A)


/** @brief Макрос позволяет записать/читать биты регистров STM32F103x с помощью области bitbanding.
    @param REG - регистр STM32F103x (не указатель на него!)
    @param BIT_MASK - маска бита в соответствии с описанием в заголовочном файле stm32f103x.h.  
    @code
    // Пример применения:
    BIT_BAND_REG(TIM1->SR, TIM_SR_UIF) = VALUE; // запись бита TIM_SR_UIF в регистр TIM1->SR
    VALUE = BIT_BAND_REG(TIM1->SR, TIM_SR_UIF); // чтение бита TIM_SR_UIF из регистра TIM1->SR
    @endcode
    @warning Внимание! Можно использовать маски, которые описывают конкретный бит, а не группу битов.
 */
#define BIT_BAND_REG(REG,BIT_MASK) (*(volatile uint32_t*)(PERIPH_BB_BASE+32*((uint32_t)(&(REG))-PERIPH_BASE)+4*((uint32_t)(_MASK_TO_BIT(BIT_MASK)))))

// Маски для настройки входов GPIO
#define GPIO_MODE_INPUT_ANALOG              0x00  ///< Аналоговый вход
#define GPIO_MODE_INPUT_FLOATING            0x04  ///< Вход HiZ, без подтяжки
#define GPIO_MODE_INPUT_PULL_UP_DOWN        0x08  ///< Вход с подтяжкой
#define GPIO_MODE_OUTPUT2_PUSH_PULL         0x02  ///< Двухтактный выход, скорость 2МГц
#define GPIO_MODE_OUTPUT10_PUSH_PULL        0x01  ///< Двухтактный выход, скорость 10МГц
#define GPIO_MODE_OUTPUT50_PUSH_PULL        0x03  ///< Двухтактный выход, скорость 50МГц
#define GPIO_MODE_OUTPUT2_OPEN_DRAIN        0x06  ///< Выход открытый коллектор, скорость 2МГц
#define GPIO_MODE_OUTPUT10_OPEN_DRAIN       0x05  ///< Выход открытый коллектор, скорость 10МГц
#define GPIO_MODE_OUTPUT50_OPEN_DRAIN       0x07  ///< Выход открытый коллектор, скорость 50МГц
#define GPIO_MODE_OUTPUT2_ALT_PUSH_PULL     0x0A  ///< Двухтактный выход альтернативных функций, скорость 2МГц
#define GPIO_MODE_OUTPUT10_ALT_PUSH_PULL    0x09  ///< Двухтактный выход альтернативных функций, скорость 10МГц
#define GPIO_MODE_OUTPUT50_ALT_PUSH_PULL    0x0B  ///< Двухтактный выход альтернативных функций, скорость 50МГц
#define GPIO_MODE_OUTPUT2_ALT_OPEN_DRAIN    0x0E  ///< Выход альтернативных функций с открытым коллектором, скорость 2МГц
#define GPIO_MODE_OUTPUT10_ALT_OPEN_DRAIN   0x0D  ///< Выход альтернативных функций с открытым коллектором, скорость 10МГц
#define GPIO_MODE_OUTPUT50_ALT_OPEN_DRAIN   0x0F  ///< Выход альтернативных функций с открытым коллектором, скорость 50МГц
 
//  Вспомогательные макросы для макроса GPIO_INIT_PIN
#define GPIO_INIT_CRL(PORT,PIN_NUM,PIN_MODE) PORT->CRL = (PORT->CRL &(~((uint32_t)0x0F<<(((PIN_NUM) & 0x07)<<2)))) | (((uint32_t)(PIN_MODE)&0x0F)<<(((PIN_NUM) & 0x07)<<2))
#define GPIO_INIT_CRH(PORT,PIN_NUM,PIN_MODE) PORT->CRH = (PORT->CRH &(~((uint32_t)0x0F<<(((PIN_NUM) & 0x07)<<2)))) | (((uint32_t)(PIN_MODE)&0x0F)<<(((PIN_NUM) & 0x07)<<2))

/** @brief Макрос настраивает пин порта ввода/вывода STM32F103x
    @param PORT - название порта (GPIOA, GPIOB, GPIOC и т.д.)
    @param NUM  - номер пина (число от 0 до 15)
    @param MODE - режим пина согласно маски, описанной как GPIO_MODE_...
    @code
    // Пример применения:
    GPIO_INIT(GPIOA,1,GPIO_MODE_OUTPUT2_PUSH_PULL); // настройка 1 пина порта GPIOA на выход двухтактный 2МГц
    @endcode
 */
#define GPIO_INIT(PORT,NUM,MODE)     ((NUM)<8)? (GPIO_INIT_CRL(PORT,NUM,MODE)):(GPIO_INIT_CRH(PORT,NUM,MODE)) 


/** @brief Макрос устанавливает пин порта ввода/вывода STM32F103x
    @param PORT - название порта (GPIOA, GPIOB, GPIOC и т.д.)
    @param NUM  - номер пина (число от 0 до 15)
    @code
    // Пример применения:
    GPIO_SET(GPIOA,1); // установка 1 пина порта GPIOA
    @endcode
 */
#define GPIO_SET(PORT,NUM)   (PORT -> BSRR |= ((uint32_t)0x00000001 << NUM))

/** @brief Макрос сбрасывает пин порта ввода/вывода STM32F103x
    @param PORT - название порта (GPIOA, GPIOB, GPIOC и т.д.)
    @param NUM  - номер пина (число от 0 до 15)
    @code
    // Пример применения:
    GPIO_RESET(GPIOA,1); // сброс 1 пина порта GPIOA
    @endcode
 */
#define GPIO_RESET(PORT,NUM) (PORT -> BSRR |= ((uint32_t)0x00010000 << NUM))

/* ОПРЕДЕЛЕНИЕ ТИПОВ
 *************************************************************************************************/	 

/// Коды ошибок
typedef enum 
{
  SUCCESS                      = 0, 
  ERROR_POINTER_IS_NULL        = 1,    ///< Указатель равен NULL
  ERROR_INVALID_ARGUMENT_VALUE = 2,    ///< Некорректное значение аргумента
  ERROR_INVALID_SEQUENCE       = 8,    ///< Не верная последовательность операций
  ERROR_TIMEOUT                = 9,    ///< Истечение таймаута
  ERROR_USART_BUFFER_IS_EMPTY  = 10,   ///< Программный буфер Usart пуст
  ERROR_USART_BUFFER_IS_FULL   = 11,   ///< Программный буфер Usart полон
  ERROR_I2C_START              = 40,   ///< Не удалось сформировать START на шине
  ERROR_I2C_BUS_IS_BUSY        = 41,   ///< Шина заблокирована
  ERROR_I2C_DEVICE_NOT_FOUND   = 42,   ///< Не получен ACK в ответ на адрес
  ERROR_I2C_DEVICE_BUSY        = 43    ///< Не получен ACK в ответ переданный байт
} ErrorStatus_t;

/// Системные частоты микроконтролера
typedef enum
{
  FREQ_SYSCLK = 0, ///< Системная частота
  FREQ_HCLK   = 1, ///< Частота шины AHB
  FREQ_PCLK1  = 2, ///< Частота шины APB1
  FREQ_PCLK2  = 3  ///< Частота шины APB2
} SystemFreq_t;

/* ОПРЕДЕЛЕНИЕ ГЛОБАЛЬНЫХ ПЕРЕМЕННЫХ
 *************************************************************************************************/

/* ПРОТОТИПЫ ГЛОБАЛЬНЫХ ФУНКЦИЙ
 *************************************************************************************************/	

// Функция инициализации микроконтроллера
void SystemInit(void);
// Функция возвращает текущие системные частоты
uint32_t RCC_GetSysClock(SystemFreq_t FreqType);

// Функция реализовывает блокирующую задержку
void DelayMs(uint32_t Delay);

ErrorStatus_t BKP_GetRegister(const uint8_t nRegister, uint16_t *const Value);
ErrorStatus_t BKP_SetRegister(const uint8_t nRegister, const uint16_t Value);

#if __RTC_USED
// Инициализация модуля RTC
void RTC_Init(void (*SecExec_Handler)(void), void (*AlarmExec_Handler)(void));
// Получить значение из счетчика RTC
uint32_t RTC_GetCount(void);
// Записать значение в счетчик RTC
void RTC_SetCount(const uint32_t Value);
// Получить значение из регистра будильника RTC
uint32_t RTC_GetAlarm(void);
// Записать значение в регистр будильника RTC
void RTC_SetAlarm(const uint32_t Value);
#endif

#if (__USART1_USED || __USART2_USED || __USART3_USED)
// Инициализация модулей USART
void USART_Init(void);
#endif
#if __USART1_USED
// Передача символа через модуль USART1
ErrorStatus_t USART1_Transmit_IT(const char vChar);
// Чтение символа принятого из модуля USART1
ErrorStatus_t USART1_Receive_IT(char *const pChar);
// Передача строки через модуль USART1
void USART1_TransmitStr(const char *pString);
#endif
#if __USART2_USED
// Передача символа через модуль USART1
ErrorStatus_t USART2_Transmit_IT(const char vChar);
// Чтение символа принятого из модуля USART1
ErrorStatus_t USART2_Receive_IT(char *const pChar);
// Передача строки через модуль USART1
void USART2_TransmitStr(const char *pString);
#endif
#if __USART3_USED
// Передача символа через модуль USART1
ErrorStatus_t USART3_Transmit_IT(const char vChar);
// Чтение символа принятого из модуля USART1
ErrorStatus_t USART3_Receive_IT(char *const pChar);
// Передача строки через модуль USART1
void USART3_TransmitStr(const char *pString);
#endif

#if (__I2C1_USED || __I2C2_USED)
// Функция инициализации модулей I2C в режиме мастера
void I2C_MasterInit(void);
// Функция чтения 2 байт из ячейки памяти устройства I2C в режиме мастера
ErrorStatus_t I2C_MemRead(I2C_TypeDef *const pI2C, const uint8_t I2C_DevAdress, const uint8_t I2C_MemAdress, uint8_t *const pData, uint16_t Size);
// Функция записи 2 байт в ячейку памяти устройства I2C в режиме мастера
ErrorStatus_t I2C_MemWrite(I2C_TypeDef *const pI2C, const uint8_t I2C_DevAdress, const uint8_t I2C_MemAdress, uint8_t *const pData, uint16_t Size);
#endif


/* ОПИСАНИЕ INLINE ФУНКЦИЙ
 *************************************************************************************************/	

#if __SYSTICK_SETUP

__inline static ErrorStatus_t SysTick_Enable(uint16_t msPeriod) 
{
#if (__SYSTICK_CTRL & 0x04)
	uint32_t SysTickClock = RCC_GetSysClock(FREQ_HCLK);
#else
	uint32_t SysTickClock = RCC_GetSysClock(FREQ_HCLK) / 8;
#endif
	
	if ((msPeriod > __SYSTICK_MAXPERIOD) || ((msPeriod * (SysTickClock/1000) - 1) > SysTick_LOAD_RELOAD_Msk))
	{
		return ERROR_INVALID_ARGUMENT_VALUE;
	}
	else
	{
		SysTick->LOAD  = msPeriod * (SysTickClock/1000) - 1; // установка значения в регистр перезагрузки
    SysTick->CTRL  = __SYSTICK_CTRL;                     // установка регистра настройки таймера
    SysTick->VAL   =  0;                                 // очистка счетчика
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;            // включение таймера
    return SUCCESS; 
	}
}

__inline static void SysTick_Disable(void)
{ 
	// Выключить таймер и прерывание SysTick
  SysTick->CTRL  &= ~(SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}

#endif

#ifdef __cplusplus
}
#endif

#endif

/* КОНЕЦ ФАЙЛА
 *************************************************************************************************/
