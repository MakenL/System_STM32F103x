/** @file     stm32f103x_i2c.c
    @author   Lyga Maksim
    @version  V1.0
    @date     20.07.2015
		@details  Библиотека реализует настройку и работу модулей I2C STM32F103x. Содержит Keil Configuration Wizard.
 
    @warning  Бибилиотека реализовывалась для 32-разрядных микроконтроллеров STM32F103
 */

/* ПОДКЛЮЧЕНИЕ ЗАГОЛОВОЧНЫХ ФАЙЛОВ
 *************************************************************************************************/
 
#include "system_stm32f103x.h"

/* КОНСТАНТЫ
 *************************************************************************************************/

#if __I2C1_USED
  #if (__AFIO_MAPR & 0x00000002)
    #define SDA1_PORT GPIOB
    #define SDA1_PIN  9
    #define SCL1_PORT GPIOB
    #define SCL1_PIN  8
  #else
    #define SDA1_PORT GPIOB
    #define SDA1_PIN  7
    #define SCL1_PORT GPIOB
    #define SCL1_PIN  6
  #endif
#endif

#if __I2C2_USED
  #define SDA2_PORT GPIOB
  #define SDA2_PIN  11
  #define SCL2_PORT GPIOB
  #define SCL2_PIN  10
#endif

#define I2C_WAIT_START 1                    ///< таймаут формирования START
#define I2C_WAIT_1BYTE 9                    ///< таймаут получения ACK на передачу 1 байта
#define I2C_WAIT_2BYTE (2 * I2C_WAIT_1BYTE) ///< таймаут получения ACK на передачу 2 байт

/* Глобальные переменные
 *************************************************************************************************/	

/* Локальные переменные
 *************************************************************************************************/

/* ПРОТОТИПЫ ЛОКАЛЬНЫХ ФУНКЦИЙ
 *************************************************************************************************/

         void I2C_Reset(I2C_TypeDef *const pI2C);
         void I2C_SetSpeed(I2C_TypeDef *pI2C);
         void I2C_ResetBus(I2C_TypeDef *const pI2C);
ErrorStatus_t I2C_MasterRequest(I2C_TypeDef *const pI2C, const uint8_t I2C_DevAdress);
ErrorStatus_t I2C_WaitFlag(I2C_TypeDef *const pI2C, const uint16_t I2C_Flag, uint16_t I2C_Timeout);

/* ОПИСАНИЕ ФУНКЦИЙ
 *************************************************************************************************/	

/** @brief  Инициализация модулей I2C в режиме мастера.

    Функция производит инициализацию модулей I2C в соответствии с настройками через Keil Configuration Wizard в т.ч.:
    - включение тактирования модулей I2C;
    - настройку таймингов в соответсвии с установленной скоростью обмена;
    - проверку и сброс шины I2C (см. EERATA стр.28).
 */
void I2C_MasterInit(void)
{
#if __I2C1_USED
  // Включить тактирование модуля I2C1
  BIT_BAND_REG(RCC->APB1ENR, RCC_APB1ENR_I2C1EN) = SET;
  // Выключение тактирование переферии І2С1
  BIT_BAND_REG(I2C1->CR1, I2C_CR1_PE) = RESET;
  // Настройка скорсти обмена на шине
  I2C_SetSpeed(I2C1);	
  // Включение тактирование переферии І2С
  BIT_BAND_REG(I2C1->CR1, I2C_CR1_PE) = SET;
  // Проверка блокировки шины EERATA стр.28
  I2C_ResetBus(I2C1);
#endif

#if __I2C2_USED
  // Включить тактирование модуля I2C2
  BIT_BAND_REG(RCC->APB1ENR, RCC_APB1ENR_I2C2EN) = SET;
  // Выключение тактирование переферии І2С2
  BIT_BAND_REG(I2C2->CR1, I2C_CR1_PE) = RESET;
  // Настройка скорсти обмена на шине
  I2C_SetSpeed(I2C2);
  // Включение тактирование переферии І2С
  BIT_BAND_REG(I2C2->CR1, I2C_CR1_PE) = SET;
  // Проверка блокировки шины EERATA стр.28
  I2C_ResetBus(I2C2);
#endif
}

/** @brief Запись 2 байт в ячейку памяти устройства I2C в режиме мастера.

    Функция производит запись 2 байт в ячейку памяти устройства на шине I2C.
    Функция является блокирующей и использует полинг флагов модуля I2C.
		
    @param[in] pI2C указатель на модуль I2C типа I2C_TypeDef (описание структуры в stm32f103x.h)
    @param[in] I2C_DevAdress адрес устройства на шине I2C
    @param[in] I2C_MemAdress адрес ячейки памяти устройства в которую производится запись
    @param[in] pData указатель на массив записываемых данных
    @param[in] Size размер массива записываемых данных
		
    @return SUCCESS - операция успешна
    @return ERROR_I2C_START - не удалось сформировать START на шине
    @return ERROR_I2C_BUS_IS_BUSY - шина заблокирована
    @return ERROR_I2C_DEVICE_NOT_FOUND - не получен ACK в ответ на адрес
    @return ERROR_I2C_DEVICE_BUSY - не получен ACK в ответ переданный байт
 */
ErrorStatus_t I2C_MemWrite(I2C_TypeDef *const pI2C, const uint8_t I2C_DevAdress, const uint8_t I2C_MemAdress, uint8_t *const pData, uint16_t Size)
{
  ErrorStatus_t I2C_Error;
	uint16_t      DataCount = 0;

  // Проверка блокировки шины EERATA стр.28
  I2C_ResetBus(pI2C);
	// Включение потверждения приема байта
  pI2C->CR1 |= I2C_CR1_ACK;
	// Генерация старта на шине (запись)
	I2C_Error = I2C_MasterRequest(pI2C, I2C_DevAdress);
	
	if (!I2C_Error)
  {
    // Если от ведомого пришел ответ ACK на адрес
    (void) pI2C->SR1;
    (void) pI2C->SR2;
    // Передача адреса ячейки памяти которую нужно записать
    pI2C->DR = I2C_MemAdress;
		
		while (DataCount < Size)
		{
      // Ожидание освобождения регистра данных (событие EV8)
      while (!(pI2C->SR1 & I2C_SR1_TXE));
      // Передача байта
      pI2C->DR = pData[DataCount];
			DataCount++;
		}
    // Ожидание окончания передачи последнего байта (событие EV8_2)
		if (I2C_WaitFlag(pI2C, I2C_SR1_BTF, I2C_WAIT_1BYTE))
		{
			// Очистка флага AF
      pI2C->SR1 &= ~I2C_SR1_AF;
			I2C_Error = ERROR_I2C_DEVICE_BUSY;
		}
    // Формирование сигнала стоп на шине
    pI2C->CR1 |= I2C_CR1_STOP;
  }
	
  // Проверка блокировки шины EERATA стр.28
  I2C_ResetBus(pI2C);
	
  return I2C_Error;
}

/** @brief Чтение 2 байт из ячейки памяти устройства I2C в режиме мастера.

    Функция производит запись 2 байт в ячейку памяти устройства на шине I2C.
    Функция является блокирующей и использует полинг флагов модуля I2C.

    @param[in] pI2C указатель на модуль I2C типа I2C_TypeDef (описание структуры в stm32f103x.h)
    @param[in] I2C_DevAdress адрес устройства на шине I2C
    @param[in] I2C_MemAdress адрес ячейки памяти устройства из которой производится чтение
    @param[in] pData указатель на массив прочитанных данных

    @return SUCCESS - операция успешна
    @return ERROR_I2C_START - не удалось сформировать START на шине
    @return ERROR_I2C_BUS_IS_BUSY - шина заблокирована
    @return ERROR_I2C_DEVICE_NOT_FOUND - не получен ACK в ответ на адрес
    @return ERROR_I2C_DEVICE_BUSY - не получен ACK в ответ переданный байт
 */
ErrorStatus_t I2C_MemRead(I2C_TypeDef *const pI2C, const uint8_t I2C_DevAdress, const uint8_t I2C_MemAdress, uint8_t *const pData, uint16_t Size)
{
  ErrorStatus_t I2C_Error;

  // Проверка блокировки шины EERATA стр.28
  I2C_ResetBus(pI2C);
	// Включение потверждения приема байта
  pI2C->CR1 |= I2C_CR1_ACK;
	// Генерация старта на шине (запись)
	I2C_Error = I2C_MasterRequest(pI2C, I2C_DevAdress);
	
	if (!I2C_Error)
  {
    // Если от ведомого пришел ответ ACK на адрес
    (void) pI2C->SR1;
    (void) pI2C->SR2;
		// Передача адреса ячейки памяти которую нужно считать
    pI2C->DR = I2C_MemAdress; 
    // Ожидание окончания передачи ячейки памяти (событие EV8_2)
		if (I2C_WaitFlag(pI2C, I2C_SR1_BTF, I2C_WAIT_1BYTE))
		{
			// Очистка флага AF
      pI2C->SR1 &= ~I2C_SR1_AF;
			// Формирование сигнала стоп на шине
      pI2C->CR1 |= I2C_CR1_STOP;
			
			I2C_Error = ERROR_I2C_DEVICE_BUSY;
		}
	}
		
	if (!I2C_Error)
  {		
    // Генерация повторного старта на шине (чтение)
		I2C_Error = I2C_MasterRequest(pI2C, I2C_DevAdress + 1);
		
		if (!I2C_Error)
		{
      // Если от ведомого пришел ответ ACK на адрес
      // Далее последовательность из EERATA стр. 23 (EV6_1)
      #if __I2C1_USED		
      if (pI2C == I2C1)
      {				
        // Перенастройка вывода SCL как вывод общего назначения с открытым коллектором
        GPIO_INIT_PIN(SCL1_PORT, SCL1_PIN, GPIO_MODE_OUTPUT50_OPEN_DRAIN);
        // Установить на выводе SCL = 0
        GPIO_RESET_PIN(SCL1_PORT, SCL1_PIN);
			}
			#endif				
      #if __I2C2_USED		
      if (pI2C == I2C2)
      {				
        // Перенастройка вывода SCL как вывод общего назначения с открытым коллектором
        GPIO_INIT(SCL2_PORT, SCL2_PIN, GPIO_MODE_OUTPUT50_OPEN_DRAIN);
        // Установить на выводе SCL = 0
        GPIO_RESET(SCL2_PORT, SCL2_PIN);
			}
			#endif

      (void) pI2C->SR1;
      (void) pI2C->SR2;
      // Сброс флага потверждение приема байта
      pI2C->CR1 &= ~I2C_CR1_ACK;
				
      #if __I2C1_USED		
      if (pI2C == I2C1)
      {				
        // Перенастройка вывода SCL как альтернативный вывод с открытым коллектором
        GPIO_INIT_PIN(SCL1_PORT, SCL1_PIN, GPIO_MODE_OUTPUT50_ALT_OPEN_DRAIN);
			}
			#endif				
      #if __I2C2_USED		
      if (pI2C == I2C2)
      {				
        // Перенастройка вывода SCL как альтернативный вывод с открытым коллектором
        GPIO_INIT(SCL2_PORT, SCL2_PIN, GPIO_MODE_OUTPUT50_ALT_OPEN_DRAIN);
			}
			#endif
			
			// Ожидание окончания приема 2 байт данных (EV7_3)
      while (!(pI2C->SR1 & I2C_SR1_BTF));
			
      // Далее последовательность из EERATA стр. 24
      #if __I2C1_USED		
      if (pI2C == I2C1)
      {				
        // Перенастройка вывода SCL как вывод общего назначения с открытым коллектором
        GPIO_INIT_PIN(SCL1_PORT, SCL1_PIN, GPIO_MODE_OUTPUT50_OPEN_DRAIN);
        // Установить на выводе SCL = 0
        GPIO_RESET_PIN(SCL1_PORT, SCL1_PIN);
			}
			#endif				
      #if __I2C2_USED		
      if (pI2C == I2C2)
      {				
        // Перенастройка вывода SCL как вывод общего назначения с открытым коллектором
        GPIO_INIT(SCL2_PORT, SCL2_PIN, GPIO_MODE_OUTPUT50_OPEN_DRAIN);
        // Установить на выводе SCL = 0
        GPIO_RESET(SCL2_PORT, SCL2_PIN);
			}
			#endif		
      
			// Формирование сигнала стоп на шине
      pI2C->CR1 |= I2C_CR1_STOP;
      // Считывание байта N-1
      *pData  = pI2C->DR;
        
			#if __I2C1_USED		
      if (pI2C == I2C1)
      {				
        // Перенастройка вывода SCL как альтернативный вывод с открытым коллектором
        GPIO_INIT_PIN(SCL1_PORT, SCL1_PIN, GPIO_MODE_OUTPUT50_ALT_OPEN_DRAIN);
			}
			#endif				
      #if __I2C2_USED		
      if (pI2C == I2C2)
      {				
        // Перенастройка вывода SCL как альтернативный вывод с открытым коллектором
        GPIO_INIT(SCL2_PORT, SCL2_PIN, GPIO_MODE_OUTPUT50_ALT_OPEN_DRAIN);
			}
			#endif
				
      // Считывание байта N
      *(pData + 1) = pI2C->DR;
			// Очистка флага AF
      pI2C->SR1 &= ~I2C_SR1_AF;
		}
	}	
	
  // Проверка блокировки шины EERATA стр.28
  I2C_ResetBus(pI2C);
	
  return I2C_Error;
}

/** @brief Сброс модуля I2C.

    Функция производит сброс модуля I2C.
    @param[in] pI2C указатель на модуль I2C типа I2C_TypeDef (описание структуры в stm32f103x.h)
 */
void I2C_Reset(I2C_TypeDef *const pI2C)
{
  pI2C->CR1 |= I2C_CR1_SWRST;
  pI2C->CR1 &= ~I2C_CR1_SWRST;
}

/** @brief Настройка скорости обмена модуля I2C.

    Функция производит настройку скорости обмена модуля I2C в соответствии с настройками Keil Configuration Wizard.
    @aram[in] pI2C указатель на модуль I2C типа I2C_TypeDef (описание структуры в stm32f103x.h)
 */
void I2C_SetSpeed(I2C_TypeDef *const pI2C)
{
  // Настройка частоты тактирования модуля I2C2 (PCLK1)
  pI2C->CR2  |= RCC_GetSysClock(FREQ_PCLK1)/1000000;
	
	#if __I2C1_USED		
  if (pI2C == I2C1)
  {
    // Настройка конечного коэфициента деления
    pI2C->CCR  |= ((uint16_t)(RCC_GetSysClock(FREQ_PCLK1)/__I2C1_SPEED / 2)) & I2C_CCR_CCR;
	}
	#endif				
  #if __I2C2_USED		
  if (pI2C == I2C2)
  {				
    // Настройка конечного коэфициента деления
    pI2C->CCR  |= ((uint16_t)(RCC_GetSysClock(FREQ_PCLK1)/__I2C2_SPEED / 2)) & I2C_CCR_CCR;
	}
	#endif
	
  // Время установления логического уровня в циклах тактового генератора I2C 
  pI2C->TRISE = 1000/(pI2C->CR2 & I2C_CR2_FREQ) + 1;
}

/** @brief Сброс шины I2C.

    Функция производит проверку и при необходимости сброс шины I2C (см. EERATA стр.28), сброс модуля I2C и его перенастройку.
    @param[in] pI2C указатель на модуль I2C типа I2C_TypeDef (описание структуры в stm32f103x.h)
 */
void I2C_ResetBus(I2C_TypeDef *const pI2C)
{
	if (pI2C->SR2 & I2C_SR2_BUSY) 
	{
    // Выключение тактирование переферии І2С
    pI2C->CR1 &= ~I2C_CR1_PE;	
		
#if __I2C1_USED			
		if (pI2C == I2C1)
    {
      // Перенастройка выводов SCL/SDA как вводы общего назначения с открытым коллектором
      GPIO_INIT_PIN(SCL1_PORT, SCL1_PIN, GPIO_MODE_OUTPUT50_OPEN_DRAIN);
      GPIO_INIT_PIN(SDA1_PORT, SDA1_PIN, GPIO_MODE_OUTPUT50_OPEN_DRAIN);
      // Установить на выводах SCL/SDA = 1 и прочитать значение
      GPIO_SET_PIN(SCL1_PORT, SCL1_PIN);
      GPIO_SET_PIN(SDA1_PORT, SDA1_PIN);
      (void)GPIOB->IDR;
      // Установить на выводах SDA = 0 и прочитать значение
      GPIO_RESET_PIN(SDA1_PORT, SDA1_PIN);
      (void)GPIOB->IDR;
      // Установить на выводах SCL = 0 и прочитать значение
      GPIO_RESET_PIN(SCL1_PORT, SCL1_PIN);
      (void)GPIOB->IDR;
      // Установить на выводах SCL = 1 и прочитать значение
      GPIO_SET_PIN(SCL1_PORT, SCL1_PIN);
      (void)GPIOB->IDR;
      // Установить на выводах SDA = 1 и прочитать значение
      GPIO_SET_PIN(SDA1_PORT, SDA1_PIN);
      (void)GPIOB->IDR;		
      // Перенастройка выводов SCL/SDA как альтернативные вводы с открытым коллектором
      GPIO_INIT_PIN(SCL1_PORT, SCL1_PIN, GPIO_MODE_OUTPUT50_ALT_OPEN_DRAIN);
      GPIO_INIT_PIN(SDA1_PORT, SDA1_PIN, GPIO_MODE_OUTPUT50_ALT_OPEN_DRAIN);
    }
#endif
#if __I2C2_USED		
    if (pI2C == I2C2)
    {
      // Перенастройка выводов SCL/SDA как вводы общего назначения с открытым коллектором
      GPIO_INIT(SCL2_PORT, SCL2_PIN, GPIO_MODE_OUTPUT50_OPEN_DRAIN);
      GPIO_INIT(SDA2_PORT, SDA2_PIN, GPIO_MODE_OUTPUT50_OPEN_DRAIN);
      // Установить на выводах SCL/SDA = 1 и прочитать значение
      GPIO_SET(SCL2_PORT, SCL2_PIN);
      GPIO_SET(SDA2_PORT, SDA2_PIN);
      (void)GPIOB->IDR;
      // Установить на выводах SDA = 0 и прочитать значение
      GPIO_RESET(SDA2_PORT, SDA2_PIN);
      (void)GPIOB->IDR;
      // Установить на выводах SCL = 0 и прочитать значение
      GPIO_RESET(SCL2_PORT, SCL2_PIN);
      (void)GPIOB->IDR;
      // Установить на выводах SCL = 1 и прочитать значение
      GPIO_SET(SCL2_PORT, SCL2_PIN);
      (void)GPIOB->IDR;
      // Установить на выводах SDA = 1 и прочитать значение
      GPIO_SET(SDA2_PORT, SDA2_PIN);
      (void)GPIOB->IDR;		
      // Перенастройка выводов SCL/SDA как альтернативные вводы с открытым коллектором
      GPIO_INIT(SCL2_PORT, SCL2_PIN, GPIO_MODE_OUTPUT50_ALT_OPEN_DRAIN);
      GPIO_INIT(SDA2_PORT, SDA2_PIN, GPIO_MODE_OUTPUT50_ALT_OPEN_DRAIN);
    }
#endif	
		
    // Сброс модуля I2C
    I2C_Reset(pI2C);
    // Настройка скорости обмена на шине I2C2
    I2C_SetSpeed(pI2C);
    // Включение тактирование переферии І2С2
    pI2C->CR1 |= I2C_CR1_PE;		
	}
}

/** @brief Генерация START на шине I2C и передача адреса в режиме мастера.

    Функция производит запись 2 байт в ячейку памяти устройства на шине I2C.
    Функция является блокирующей и использует полинг флагов модуля I2C.

    @param[in] pI2C указатель на модуль I2C типа I2C_TypeDef (описание структуры в stm32f103x.h)
    @param[in] I2C_DevAdress адрес устройства на шине I2C

    @return SUCCESS - операция успешна   
    @return ERROR_I2C_START - не удалось сформировать START на шине
    @return ERROR_I2C_DEVICE_NOT_FOUND - не получен ACK в ответ на адрес
 */
ErrorStatus_t I2C_MasterRequest(I2C_TypeDef *const pI2C, const uint8_t I2C_DevAdress)
{
  // Генерация старта на шине	
  pI2C->CR1 |= I2C_CR1_START;
  // Ожидание окончания генерации старт (EV5) или истечения таймаута
  if (I2C_WaitFlag(pI2C, I2C_SR1_SB, I2C_WAIT_START))
  {
		// Проверка на ошибки шины EERATA стр.26
    if (pI2C->SR1 & I2C_SR1_BERR)
    {
      // Выключение тактирование переферии І2С
      pI2C->CR1 &= ~I2C_CR1_PE;
      // Сброс модуля I2C
      I2C_Reset(pI2C);
      // Настройка скорости обмена на шине I2C2
      I2C_SetSpeed(pI2C);
      // Включение тактирование переферии І2С2
      pI2C->CR1 |= I2C_CR1_PE;
    }
		// Формирование сигнала стоп на шине
    pI2C->CR1 |= I2C_CR1_STOP;	
    return ERROR_I2C_START;
  }
  else
  {
		// Передача адреса ведомого
    pI2C->DR = I2C_DevAdress;
		// Ожидание окончания передачи адреса (событие EV6)
    if (I2C_WaitFlag(pI2C, I2C_SR1_ADDR, I2C_WAIT_1BYTE))
		{
			// Если от ведомого пришел ответ NACK на адрес
		  // Очистка флага AF
      pI2C->SR1 &= ~I2C_SR1_AF;
      // Формирование сигнала стоп на шине
      pI2C->CR1 |= I2C_CR1_STOP;			
			return ERROR_I2C_DEVICE_NOT_FOUND;
		}
		else
		{
			// Если от ведомого пришел ответ ACK на адрес
      return SUCCESS;
		}
  }
}

/** @brief Проверка установки флага события модуля I2C.

    Функция производит проверку установки флагов регистра SR1 модулей I2C.
    Функция является блокирующей и использует полинг.
    Если в течении заданного таймаута флаг не установлен, возвращает ошибку ERROR_TIMEOUT.

    @param[in] pI2C указатель на модуль I2C типа I2C_TypeDef (описание структуры в stm32f103x.h)
    @param[in] I2C_Flag проверяемый флаг регистра SR1 модуля I2C (описание битов в stm32f103x.h)
    @param[in] I2C_Timeout время ожидания установки флага (в битах)

    @return SUCCESS - флаг установился (событие произошло)
    @return ERROR_TIMEOUT - истек таймаут ожидания установки флага
 */
ErrorStatus_t I2C_WaitFlag(I2C_TypeDef *const pI2C, const uint16_t I2C_Flag, uint16_t I2C_Timeout)
{
	I2C_Timeout *= (pI2C->CCR & I2C_CCR_CCR);
	
  while((!(pI2C->SR1 & I2C_Flag)) && I2C_Timeout) 
  {
	  I2C_Timeout--;
	}
  if (pI2C->SR1 & I2C_Flag)
  {
    return SUCCESS;
  }
	else
	{
		return ERROR_TIMEOUT;
	}
}

/* КОНЕЦ ФАЙЛА
 *************************************************************************************************/
