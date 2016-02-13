/** @file     system_stm32f103x.c
    @author   Lyga Maksim
    @version  V1.0
    @date     20.07.2015
 */

/* ПОДКЛЮЧЕНИЕ ЗАГОЛОВОЧНЫХ ФАЙЛОВ
 *************************************************************************************************/
 
#include "system_stm32f103x.h"
#include "stddef.h"

/* КОНСТАНТЫ
 *************************************************************************************************/

/* Глобальные переменные
*************************************************************************************************/	

/* Локальные переменные
*************************************************************************************************/

volatile uint32_t *BKPRegister1  = (volatile uint32_t*)(BKP_BASE + 0x0004);
volatile uint32_t *BKPRegister2  = (volatile uint32_t*)(BKP_BASE + 0x0040);

#if (__RTC_USED && __RTC_CRH)
void (*RTC_SecExec)(void);    ///< Указатель на callback-функцию обработки секундного прерывания
void (*RTC_AlarmExec)(void);  ///< Указатель на callback-функцию обработки прерывания будильника
#endif

/* ПРОТОТИПЫ ЛОКАЛЬНЫХ ФУНКЦИЙ
*************************************************************************************************/	

__inline static void RCC_Reset(void);
__inline static void RCC_Init(void);
__inline static void NVIC_Init(void);
#if __GPIO_AFIOEN
__inline static void AFIO_Init(void);
#endif
__inline static void GPIO_Init(void);
__inline static void BKP_DomainInit(void);

/* ОПИСАНИЕ ФУНКЦИЙ
*************************************************************************************************/	

void SystemInit (void)
{
	RCC_Reset();
  RCC_Init();
  NVIC_Init();
#if __GPIO_AFIOEN
	AFIO_Init();
#endif
  GPIO_Init();
	BKP_DomainInit();
}

// Сброс модуля RCC
__inline static void RCC_Reset(void)
{
	// Включение внутреннего генератора HSI
  BIT_BAND_REG(RCC->CR, RCC_CR_HSION) = SET;
  // Ожидание включения внутреннего генератора HSI (HSIRDY = 1)
  while (BIT_BAND_REG(RCC->CR, RCC_CR_HSIRDY) == RESET);
	
	RCC->CR |= (uint32_t)(__HSITRIM & RCC_CR_HSITRIM);

	// Сброс битов регистра CFGR
  //  SW[1:0]     - тактирование от HSI SYSCLK = HSI
  //  HPRE[3:0]   - коэфициент деления частоты шины AHB  HCLK = SYSCLK / 1 
  //  PRE1[2:0]   - коэфициент деления частоты шины APB1 PCLK1 = HCLK 
  //  PRE2[2:0]   - коэфициент деления частоты шины APB2 PCLK2 = HCLK 
	//  ADCPRE[1:0] - коэфициент деления частоты ADC PCLK2 / 2
  //  MCO[2:0]    - 
  RCC->CFGR &= (uint32_t)0xF8FF0000;
	
	// Ожидание переключения на тактирование от внутреннего генератора HSI
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
	
  // Сброс битов регистра CR
  //  HSEON       - выключить генератор HSE
	//  CSSON       - выключить CSS генератора HSE
	//  PLLON       - выключить PLL
  RCC->CR &= (uint32_t)0xFEF6FFFF;
  // Сброс бита HSEBYP - к генератору HSE подключен кварцевый резонатор 4-16MHz
	BIT_BAND_REG(RCC->CR, RCC_CR_HSEBYP) = RESET;
  // Сброс битов регистра CR
  //  PLLSRC      - источником для PLL выбран HSI / 2
	//  PLLXTPRE    - выключен делитель генератора HSE для PLL
	//  PLLMUL[3:0] - коэффициент умножения PLL х2
	//  USBPRE      - на модуль USB подается частота PLL /1,5
  RCC->CFGR &= (uint32_t)0xFF80FFFF;
  // Отключить все прерывания и очистить флаги
  RCC->CIR = (uint32_t)0x009F0000;
}

// Функция настройки модуля RCC
__inline static void RCC_Init(void)
{
#if __HSEON
	__IO uint32_t StartUpCounter = 0;
#endif

  // Конфигурируем Flash 
	FLASH->ACR = (uint32_t)(__FLASH_ACR);

#if __HSEON

  #if __HSEBYP
	  BIT_BAND_REG(RCC->CR, RCC_CR_HSEBYP) = SET;
	#endif
		
	// Включение HSE
  BIT_BAND_REG(RCC->CR, RCC_CR_HSEON) = SET;
  // Ожидание включения HSE (HSERDY = 1) или истечения таймаута 
  while((BIT_BAND_REG(RCC->CR, RCC_CR_HSERDY) == RESET) && (StartUpCounter < __HSE_TIMEOUT))
  {
    StartUpCounter++;
  }
		
	if (BIT_BAND_REG(RCC->CR, RCC_CR_HSERDY) == SET)
  {
		#if __CSSON
		  BIT_BAND_REG(RCC->CR, RCC_CR_CSSON) = SET;
		#endif
		
		#if __PLLON	
		  // Установка конфигурации тактирования
		  RCC->CFGR = (uint32_t)(__RCC_CFGR & (~RCC_CFGR_SW));
		
		  // Включаем PLL
      BIT_BAND_REG(RCC->CR, RCC_CR_PLLON) = SET;
      // Ожидание включения PLL (PLLRDY = 1)
      while(BIT_BAND_REG(RCC->CR, RCC_CR_PLLRDY) == RESET);
		
		  // Переключение SYSCLK на PLL
		  RCC->CFGR |= (uint32_t)(RCC_CFGR_SW_PLL);		
		#else
			// Установка конфигурации тактирования
		  RCC->CFGR |= (uint32_t)(__RCC_CFGR);
    #endif

    // Ожидание переключения на тактирование SYSCLK от выбранного источника
    while ((RCC->CFGR & RCC_CFGR_SWS) != ((__RCC_CFGR << 2) & RCC_CFGR_SWS));
	}
  else // HSE не стартовал. Если мы здесь, то мы работаем от HSI
  {
    // Сброс битов регистра CR
    //  HSEON       - выключить генератор HSE
	  //  CSSON       - выключить CSS генератора HSE
	  //  PLLON       - выключить PLL
    RCC->CR &= (uint32_t)0xFEF6FFFF;
    // Сброс бита HSEBYP - к генератору HSE подключен кварцевый резонатор 4-16MHz
	  BIT_BAND_REG(RCC->CR, RCC_CR_HSEBYP) = RESET;
		// Конфигурируем Flash на 0 циклов ожидани
    FLASH->ACR &= (uint32_t)(~FLASH_ACR_LATENCY);
  }
	
#else

	#if __PLLON
		// Установка конфигурации тактирования
		RCC->CFGR = (uint32_t)(__RCC_CFGR & (~RCC_CFGR_SW));
	
	  // Включаем PLL
    BIT_BAND_REG(RCC->CR, RCC_CR_PLLON) = SET;
    // Ожидание включения PLL (PLLRDY = 1)
    while(BIT_BAND_REG(RCC->CR, RCC_CR_PLLRDY) == RESET);
	
		// Переключение SYSCLK на PLL
		RCC->CFGR |= (uint32_t)(RCC_CFGR_SW_PLL);		
	#else
		// Установка конфигурации тактирования
	  RCC->CFGR |= (uint32_t)__RCC_CFGR;
  #endif	
	
  // Ожидание переключения на тактирование SYSCLK от выбранного источника
  while ((RCC->CFGR & RCC_CFGR_SWS) != ((__RCC_CFGR << 2) & RCC_CFGR_SWS));
	
#endif
}

// Функция настройки модуля NVIC
__inline static void NVIC_Init(void)
{
#if (__VECT_TAB_BASE)
	// Vector Table Relocation in Internal SRAM
  SCB->VTOR = SRAM_BASE | __VECT_TAB_OFFSET;
#else
	// Vector Table Relocation in Internal FLASH
  SCB->VTOR = FLASH_BASE | __VECT_TAB_OFFSET;
#endif
}

// Функция сброса модуля AFIO
#if __GPIO_AFIOEN
__inline static void AFIO_Init(void) 
{
	// тактирование альтернативных функций GPIO	
	BIT_BAND_REG(RCC->APB2ENR, RCC_APB2ENR_AFIOEN) = SET;
	
#if __AFIO_MAPR
	AFIO->MAPR = __AFIO_MAPR;
#endif
}
#endif

// Функция начальной инициализации портов ввода/вывода STM32F103x.
__inline static void GPIO_Init(void)
{
#if __GPIOA_USED
	// включение тактирование порта GPIOA
  BIT_BAND_REG(RCC->APB2ENR, RCC_APB2ENR_IOPAEN) = SET;
	GPIOA->ODR = __GPIOA_ODR;
	GPIOA->CRL = __GPIOA_CRL;
	GPIOA->CRH = __GPIOA_CRH;
	
	#if __GPIOA_LCKR
    GPIOA->LCKR = __GPIOA_LCKR;
	  // выполнение алгоритма блокировки пинов порта GPIOA
		BIT_BAND_REG(GPIOA->LCKR, GPIOA_LCKR_LCKK) = SET;  
	  BIT_BAND_REG(GPIOA->LCKR, GPIOA_LCKR_LCKK) = RESET;
	  BIT_BAND_REG(GPIOA->LCKR, GPIOA_LCKR_LCKK) = SET;
	  if (BIT_BAND_REG(GPIOA->LCKR, GPIOA_LCKR_LCKK) == RESET);
	#endif
#endif
		
#if __GPIOB_USED
	// включение тактирование порта GPIOB
	BIT_BAND_REG(RCC->APB2ENR, RCC_APB2ENR_IOPBEN) = SET;
	GPIOB->ODR = __GPIOB_ODR;
	GPIOB->CRL = __GPIOB_CRL;
	GPIOB->CRH = __GPIOB_CRH;
	
	#if __GPIOB_LCKR
    GPIOB->LCKR = __GPIOB_LCKR;
	  // выполнение алгоритма блокировки пинов порта GPIOB
		BIT_BAND_REG(GPIOB->LCKR, GPIOB_LCKR_LCKK) = SET;  
	  BIT_BAND_REG(GPIOB->LCKR, GPIOB_LCKR_LCKK) = RESET;
	  BIT_BAND_REG(GPIOB->LCKR, GPIOB_LCKR_LCKK) = SET;
	  if (BIT_BAND_REG(GPIOB->LCKR, GPIOB_LCKR_LCKK) == RESET);
	#endif
#endif
	
#if __GPIOC_USED
	// включение тактирование порта GPIOC
	BIT_BAND_REG(RCC->APB2ENR, RCC_APB2ENR_IOPCEN) = SET;
	GPIOC->ODR = __GPIOC_ODR;
	GPIOC->CRL = __GPIOC_CRL;
	GPIOC->CRH = __GPIOC_CRH;
	
	#if __GPIOC_LCKR
    GPIOC->LCKR = __GPIOC_LCKR;
	  // выполнение алгоритма блокировки пинов порта GPIOC
		BIT_BAND_REG(GPIOC->LCKR, GPIOC_LCKR_LCKK) = SET;  
	  BIT_BAND_REG(GPIOC->LCKR, GPIOC_LCKR_LCKK) = RESET;
	  BIT_BAND_REG(GPIOC->LCKR, GPIOC_LCKR_LCKK) = SET;
	  if (BIT_BAND_REG(GPIOC->LCKR, GPIOC_LCKR_LCKK) == RESET);
	#endif
#endif
		
#if __GPIOD_USED
	// включение тактирование порта GPIOD
	BIT_BAND_REG(RCC->APB2ENR, RCC_APB2ENR_IOPDEN) = SET;
	GPIOD->ODR = __GPIOD_ODR;
	GPIOD->CRL = __GPIOD_CRL;
	GPIOD->CRH = __GPIOD_CRH;
	
	#if __GPIOD_LCKR
    GPIOD->LCKR = __GPIOD_LCKR;
	  // выполнение алгоритма блокировки пинов порта GPIOD
		BIT_BAND_REG(GPIOD->LCKR, GPIOD_LCKR_LCKK) = SET;  
	  BIT_BAND_REG(GPIOD->LCKR, GPIOD_LCKR_LCKK) = RESET;
	  BIT_BAND_REG(GPIOD->LCKR, GPIOD_LCKR_LCKK) = SET;
	  if (BIT_BAND_REG(GPIOD->LCKR, GPIOD_LCKR_LCKK) == RESET);
	#endif
#endif
		
#if __GPIOE_USED
  // включение тактирование порта GPIOE
	BIT_BAND_REG(RCC->APB2ENR, RCC_APB2ENR_IOPEEN) = SET;
	GPIOE->ODR = __GPIOE_ODR;
	GPIOE->CRL = __GPIOE_CRL;
	GPIOE->CRH = __GPIOE_CRH;
	
	#if __GPIOE_LCKR
    GPIOE->LCKR = __GPIOE_LCKR;
	  // выполнение алгоритма блокировки пинов порта GPIOE
		BIT_BAND_REG(GPIOE->LCKR, GPIOE_LCKR_LCKK) = SET;  
	  BIT_BAND_REG(GPIOE->LCKR, GPIOE_LCKR_LCKK) = RESET;
	  BIT_BAND_REG(GPIOE->LCKR, GPIOE_LCKR_LCKK) = SET;
	  if (BIT_BAND_REG(GPIOE->LCKR, GPIOE_LCKR_LCKK) == RESET);
	#endif
#endif
}

// Функция возвращает текущую системную частоту
uint32_t RCC_GetSysClock(SystemFreq_t FreqType) 
{
	uint32_t SystemClock = 0;
	uint32_t AHBClock    = 0;
	uint32_t ClockResult = 0;
	
  if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI)
  {
		// HSI источник для SYSCLK
    SystemClock = __HSI_CLOCK;
  }
  else if ((__RCC_CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE)
  {
		// HSE источник для SYSCLK
    SystemClock = __HSE_CLOCK;
  }
  else
  {	
    // PLL источник для SYSCLK
    uint32_t Pllmull = ((RCC->CFGR & RCC_CFGR_PLLMULL) >> 18) + 2;

    if (RCC->CFGR & RCC_CFGR_PLLSRC)
	  {
	    if (RCC->CFGR & RCC_CFGR_PLLXTPRE)
			{
        // SYSCLK = HSE/2 * pllmull		
        SystemClock = (__HSE_CLOCK >> 1) * Pllmull;
			}
      else
			{
		    // SYSCLK = HSE   * pllmul
        SystemClock = __HSE_CLOCK * Pllmull;
      }
	  }
    else
	  {
	    // SYSCLK = HSI/2 * pllmul
      SystemClock = (__HSI_CLOCK >> 1) * Pllmull;
    }
	}
	
	if (((RCC->CFGR & RCC_CFGR_HPRE) >> 4) & 0x08)
	{
    AHBClock = SystemClock >> ((((RCC->CFGR & RCC_CFGR_HPRE) >> 4) & 0x07) + 1);
	}
	else
	{
		AHBClock = SystemClock;
	}
	
	switch (FreqType)
	{
	  case FREQ_SYSCLK:
			               ClockResult = SystemClock;
			               break;
		case FREQ_HCLK:
                     ClockResult = AHBClock;
			               break;
    case FREQ_PCLK1:
			               if (((RCC->CFGR & RCC_CFGR_PPRE1) >> 8) & 0x04)
										 {
										   ClockResult = AHBClock >> ((((RCC->CFGR & RCC_CFGR_PPRE1) >> 8) & 0x03) + 1);
										 }
										 else
										 {
											 ClockResult = AHBClock;
										 }
			               break;
    case FREQ_PCLK2:
			               if (((RCC->CFGR & RCC_CFGR_PPRE2) >> 11) & 0x04)
										 {
                       ClockResult = AHBClock >> ((((RCC->CFGR & RCC_CFGR_PPRE2) >> 11) & 0x03) + 1);
                     }
										 else
										 {
                       ClockResult = AHBClock;
										 } 
			               break;
	}
	
	return ClockResult;
}

// Инициализация Backup Domain
__inline static void BKP_DomainInit(void)
{
	// Включить тактирование PWR и Backup
	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
}

ErrorStatus_t BKP_SetRegister(const uint8_t nRegister, const uint16_t Value)
{
	if ((nRegister > 0) && (nRegister < 43))
	{
		BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = SET;   // Разрешить доступ к Backup области
		if (nRegister < 11)
		{
		  BKPRegister1[nRegister - 1] = Value;
		}
		else
		{
			BKPRegister2[nRegister - 11] = Value;
		}
		BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = RESET; // Запретить доступ к Backup области
		return SUCCESS;
	}
	else
	{
		return ERROR_INVALID_ARGUMENT_VALUE;
	}
}

ErrorStatus_t BKP_GetRegister(const uint8_t nRegister, uint16_t *const Value)
{
	if ((nRegister > 0) && (nRegister < 43))
	{
		BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = SET;   // Разрешить доступ к Backup области
		if (nRegister < 11)
		{
		  *Value = BKPRegister1[nRegister - 1];
		}
		else
		{
			*Value = BKPRegister2[nRegister - 11];
		}
		BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = RESET; // Запретить доступ к Backup области
		return SUCCESS;
	}
	else
	{
		return ERROR_INVALID_ARGUMENT_VALUE;
	}
}

#if __RTC_USED

// Инициализация модуля RTC
void RTC_Init(void (*SecExec_Handler)(void), void (*AlarmExec_Handler)(void))                                                                           
{
	BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = SET;                     // Разрешить доступ к Backup области
	
	// Проверка работы часов и источника тактового сигнала
  // Если не включены или изменился источник, то инициализировать
  if ((BIT_BAND_REG(RCC->BDCR, RCC_BDCR_RTCEN) != SET) || ((RCC->BDCR & RCC_BDCR_RTCSEL) != __RCC_BDCR))
  {
		
    BIT_BAND_REG(RCC->BDCR, RCC_BDCR_BDRST) = SET; 
		BIT_BAND_REG(RCC->BDCR, RCC_BDCR_BDRST) = RESET;           // Сбросить Backup область

#if ((__RCC_BDCR & 0x00000300) == 0x00000000)
		// NoClock is RTC clock source
		#define __RTC_CLOCK 0
#elif ((__RCC_BDCR & 0x00000300) == 0x00000100)
		// LSE is RTC clock source
		BIT_BAND_REG(RCC->BDCR, RCC_BDCR_LSEON) = SET;             // Включить LSE
    while (BIT_BAND_REG(RCC->BDCR, RCC_BDCR_LSERDY) == RESET); // Дождаться включения LSE
		#define __RTC_CLOCK __LSE_CLOCK
#elif	((__RCC_BDCR & 0x00000300) == 0x00000200)     
		// LSI is RTC clock source
		BIT_BAND_REG(RCC->CSR, RCC_CSR_LSION) = SET;               // Включить LSI
    while (BIT_BAND_REG(RCC->CSR, RCC_CSR_LSIRDY) == RESET);   // Дождаться включения LSI
		#define __RTC_CLOCK __LSI_CLOCK
#elif ((__RCC_BDCR & 0x00000300) == 0x00000300)  
    // HSE/128 is RTC clock source
		#define __RTC_CLOCK (__HSE_CLOCK/128)
#endif		 
		RCC->BDCR |= RCC_BDCR_RTCEN | __RCC_BDCR;                  // Выбрать источник и подать тактирование
    //BKP->RTCCR |= 3;                                         // Калибровка RTC
	}	
		
  while (BIT_BAND_REG(RTC->CRL, RTC_CRL_RTOFF) == RESET);      // Проверить закончены ли изменения регистров RTC
  BIT_BAND_REG(RTC->CRL, RTC_CRL_CNF) = SET;                   // Разрешить запись в регистры RTC
		
	// Настроить делитель RTC
	RTC->PRLH  = ((__RTC_PERIOD * __RTC_CLOCK/1000 - 1) >> 16) & 0x00FF;
  RTC->PRLL  = ((__RTC_PERIOD * __RTC_CLOCK/1000 - 1)) & 0xFFFF;
	// Настроить прерывания RTC
  RTC->CRH = __RTC_CRH;
                                  	
	BIT_BAND_REG(RTC->CRL, RTC_CRL_CNF) = RESET;                // Запретить запись в регистры RTC
  while (BIT_BAND_REG(RTC->CRL, RTC_CRL_RTOFF) == RESET);     // Дождаться окончания записи

	BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = RESET;                  // Запретить доступ к Backup области	
	
  // Установить обработчик секундного прерывания
  RTC_SecExec = SecExec_Handler;
	// Установить обработчик прерывания будильника
	RTC_AlarmExec = AlarmExec_Handler;

#if (__RTC_CRH)
	NVIC_EnableIRQ (RTC_IRQn);                                  // Разрешить прерывания от RTC
#endif
	
}

// Получить значение из счетчика RTC
uint32_t RTC_GetCount(void)                                                             
{
	return  (uint32_t)((RTC->CNTH << 16) | RTC->CNTL);
}

// Записать значение в счетчик RTC
void RTC_SetCount(const uint32_t Value)
{
  BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = SET;                   // Разрешить доступ к Backup области
  while (BIT_BAND_REG(RTC->CRL, RTC_CRL_RTOFF) == RESET);    // Проверить закончены ли изменения регистров RTC
  BIT_BAND_REG(RTC->CRL, RTC_CRL_CNF) = SET;                 // Разрешить запись в регистры RTC
	// Записать новое значение счетного регистра
  RTC->CNTH = Value >> 16;                                                              
  RTC->CNTL = Value;
	
  BIT_BAND_REG(RTC->CRL, RTC_CRL_CNF) = RESET;               // Запретить запись в регистры RTC
  while (BIT_BAND_REG(RTC->CRL, RTC_CRL_RTOFF) == RESET);    // Дождаться окончания записи
  BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = RESET;                 // Запретить доступ к Backup области
}

// Получить значение из регистра будильника RTC
uint32_t RTC_GetAlarm(void)                                                             
{
	return  (uint32_t)((RTC->ALRH << 16) | RTC->ALRL);
}

// Записать значение в регистр будильника RTC
void RTC_SetAlarm(const uint32_t Value)
{
  BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = SET;                   // Разрешить доступ к Backup области
  while (BIT_BAND_REG(RTC->CRL, RTC_CRL_RTOFF) == RESET);    // Проверить закончены ли изменения регистров RTC
  BIT_BAND_REG(RTC->CRL, RTC_CRL_CNF) = SET;                 // Разрешить запись в регистры RTC
	// Записать новое значение регистра будильника
  RTC->ALRH = Value >> 16;                                                              
  RTC->ALRL = Value;
	
  BIT_BAND_REG(RTC->CRL, RTC_CRL_CNF) = RESET;               // Запретить запись в регистры RTC
  while (BIT_BAND_REG(RTC->CRL, RTC_CRL_RTOFF) == RESET);    // Дождаться окончания записи
  BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = RESET;                 // Запретить доступ к Backup области
}

#endif

// Функция реализовывает блокирующую задержку
void DelayMs(uint32_t Delay)
{
    if (Delay > 1000) Delay = 1000;
    Delay *= RCC_GetSysClock(FREQ_SYSCLK)/2000;
    while (Delay--);
}

//  Обработчик прерывания CSS
//----------------------------------------------------------------------------------------------
#if __CSSON
void NMI_Handler(void) 
{
	if (BIT_BAND_REG(RCC->CIR, RCC_CIR_CSSF) == SET)
	{
		// Очищаем флаг прерывания CSS
	  BIT_BAND_REG(RCC->CIR, RCC_CIR_CSSC) = RESET;
	  // Сброс битов регистра CFGR
    //  SW[1:0]     - тактирование от HSI SYSCLK = HSI
    //  HPRE[3:0]   - коэфициент деления частоты шины AHB  HCLK = SYSCLK / 1 
    //  PRE1[2:0]   - коэфициент деления частоты шины APB1 PCLK1 = HCLK 
    //  PRE2[2:0]   - коэфициент деления частоты шины APB2 PCLK2 = HCLK 
	  //  ADCPRE[1:0] - коэфициент деления частоты ADC PCLK2 / 2
    //  MCO[2:0]    - 
    RCC->CFGR &= (uint32_t)0xF8FF0000;

    // Сброс битов регистра CR
    //  HSEON       - выключить генератор HSE
	  //  CSSON       - выключить CSS генератора HSE
	  //  PLLON       - выключить PLL
    RCC->CR &= (uint32_t)0xFEF6FFFF;
    // Сброс бита HSEBYP - к генератору HSE подключен кварцевый резонатор 4-16MHz
	  BIT_BAND_REG(RCC->CR, RCC_CR_HSEBYP) = RESET;
    // Сброс битов регистра CR
    //  PLLSRC      - источником для PLL выбран HSI / 2
	  //  PLLXTPRE    - выключен делитель генератора HSE для PLL
	  //  PLLMUL[3:0] - коэффициент умножения PLL х2
	  //  USBPRE      - на модуль USB подается частота PLL /1,5
    RCC->CFGR &= (uint32_t)0xFF80FFFF;
		
		// Конфигурируем Flash на 0 циклов ожидания
    FLASH->ACR &= (uint32_t)(~FLASH_ACR_LATENCY);
	}
} 
#endif

//  Обработчик прерывания модуля RTC
//----------------------------------------------------------------------------------------------
#if (__RTC_USED && __RTC_CRH)
void RTC_IRQHandler(void)
{	
	BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = SET;                   // Разрешить доступ к Backup области
  while (BIT_BAND_REG(RTC->CRL, RTC_CRL_RTOFF) == RESET);    // Проверить закончены ли изменения регистров RTC
  BIT_BAND_REG(RTC->CRL, RTC_CRL_CNF) = SET;                 // Разрешить запись в регистры RTC
	
	if (BIT_BAND_REG(RTC->CRL, RTC_CRL_SECF) == SET)
  {
		BIT_BAND_REG(RTC->CRL, RTC_CRL_SECF) = RESET;            // Очистить флаг прерывания SECF
		if (RTC_SecExec != NULL)
		{
			RTC_SecExec();
		}
  }
	
	if (BIT_BAND_REG(RTC->CRL, RTC_CRL_ALRF) == SET)
  {
		BIT_BAND_REG(RTC->CRL, RTC_CRL_ALRF) = RESET;            // Очистить флаг прерывания ALRF
		if (RTC_AlarmExec != NULL)
		{
			RTC_AlarmExec();
		}
  }
	
	if (BIT_BAND_REG(RTC->CRL, RTC_CRL_OWF) == SET)
  {
		BIT_BAND_REG(RTC->CRL, RTC_CRL_OWF) = RESET;             // Очистить флаг прерывания OWF
  }
	
	BIT_BAND_REG(RTC->CRL, RTC_CRL_CNF) = RESET;               // Запретить запись в регистры RTC
  while (BIT_BAND_REG(RTC->CRL, RTC_CRL_RTOFF) == RESET);    // Дождаться окончания записи
  BIT_BAND_REG(PWR->CR, PWR_CR_DBP) = RESET;                 // Запретить доступ к Backup области 
}

#endif

/* КОНЕЦ ФАЙЛА
 *************************************************************************************************/
