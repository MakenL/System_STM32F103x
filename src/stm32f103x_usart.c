/** @file     stm32f103x_usart.c
    @author   Lyga Maksim
    @version  V1.0
    @date     20.07.2015
    @details  Библиотека реализует работу модулей USART STM32F103x и поддерживает:
    - программные настраевыемые буферы приемника и передатчика;
    - проверку программного буфера передатчика на переполнение и опустошение;
    - проверку программного буфера приемника на опустошение;
    - прием и передачу символов в прерывании;
    - Keil Configuration Wizard позволяющий настраивать, включать и выключать функции.

    Описание использования библиотеки:
    - Вызвать функцию инициализации USART_Init;
    - В основной программе:
    - для передачи использовать функции USARTx_Transmit_IT и USARTx_TransmitStr;
    - для приема использовать функцию USARTx_Receive_IT;
    - Функции приема/передачи используют программные буферы. 
      Отправка и прием символов модулем USART осуществляется в/из программных буферов в прерывании.
 
    @code
    // Пример применения в режиме эха
    int main(void)
    {
      char UsartData;
      USART1_Init();
      while(1)
      {
        if (USART1_ReadChar(&UsartData) == SUCCESS)
        {
    		   while (USART1_WriteChar(UsartData));
        }
      }
    }
    @endcode
 
    @warning  Бибилиотека реализовывалась для 32-разрядных микроконтроллеров STM32F103
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

#if __USART1_USED
/// Структура программного буфера модуля USART1.
__IO static struct 
{
  char     TxBuf[__USART1_TX_BUF]; ///< кольцевой программный буфер передатчика USART1
	uint16_t TxIn;                   ///< указатель записи в программный буфер передатчика USART
	uint16_t TxOut;                  ///< указатель чтения из программного буфера передатчика USART
	uint16_t TxMask;                 ///< маска буфера передатчика USART
  char     RxBuf[__USART1_RX_BUF]; ///< кольцевой программный буфер приемника USART1
  uint16_t RxIn;                   ///< указатель записи в программный буфер приемника USART
  uint16_t RxOut;                  ///< указатель чтения из программного буфера приемника USART
	uint16_t RxMask;                 ///< маска буфера приемника USART
} USART1_Buffer;
#endif

#if __USART2_USED
/// Структура программного буфера модуля USART2.
__IO static struct 
{
  char     TxBuf[__USART2_TX_BUF]; ///< кольцевой программный буфер передатчика USART1
	uint16_t TxIn;                   ///< указатель записи в программный буфер передатчика USART
	uint16_t TxOut;                  ///< указатель чтения из программного буфера передатчика USART
	uint16_t TxMask;                 ///< маска буфера передатчика USART
  char     RxBuf[__USART2_RX_BUF]; ///< кольцевой программный буфер приемника USART1
  uint16_t RxIn;                   ///< указатель записи в программный буфер приемника USART
  uint16_t RxOut;                  ///< указатель чтения из программного буфера приемника USART
	uint16_t RxMask;                 ///< маска буфера приемника USART
} USART2_Buffer;
#endif

#if __USART3_USED
/// Структура программного буфера модуля USART3.
__IO static struct 
{
  char     TxBuf[__USART3_TX_BUF]; ///< кольцевой программный буфер передатчика USART1
	uint16_t TxIn;                   ///< указатель записи в программный буфер передатчика USART
	uint16_t TxOut;                  ///< указатель чтения из программного буфера передатчика USART
	uint16_t TxMask;                 ///< маска буфера передатчика USART
  char     RxBuf[__USART3_RX_BUF]; ///< кольцевой программный буфер приемника USART1
  uint16_t RxIn;                   ///< указатель записи в программный буфер приемника USART
  uint16_t RxOut;                  ///< указатель чтения из программного буфера приемника USART
	uint16_t RxMask;                 ///< маска буфера приемника USART
} USART3_Buffer;
#endif

/* ПРОТОТИПЫ ЛОКАЛЬНЫХ ФУНКЦИЙ
 *************************************************************************************************/

/* ОПИСАНИЕ ФУНКЦИЙ
 *************************************************************************************************/	

#if __USART1_USED
__inline static void USART1_Init(void)
{ 
	// инициализация программного буфера USART1
	USART1_Buffer.TxIn   = 0;
	USART1_Buffer.TxOut  = 0;
	USART1_Buffer.TxMask = __USART1_TX_BUF - 1;
	USART1_Buffer.RxIn   = 0;
	USART1_Buffer.RxOut  = 0;
	USART1_Buffer.RxMask = __USART1_RX_BUF - 1;
	
	// включить тактирование USART1
	BIT_BAND_REG(RCC->APB2ENR, RCC_APB2ENR_USART1EN) = SET;
	// настройка скорости передачи USART1
	USART1->BRR = RCC_GetSysClock(FREQ_PCLK2) / __USART1_BAUDRATE;
	
	USART1->CR1 |= __USART1_CR1;
	USART1->CR2 |= __USART1_CR2;

  USART1->CR1  |= (USART_CR1_RXNEIE |   // включить прерывание по приему
	                 USART_CR1_UE |       // включить модуль USART1
	                 USART_CR1_TE |       // включить передатчик USART1
	                 USART_CR1_RE);       // включить приемник USART1
	
	NVIC_EnableIRQ (USART1_IRQn);         // разрешить прерывания от USART1
}
#endif

#if __USART2_USED
__inline static void USART2_Init(void)
{ 
	// инициализация программного буфера USART2
	USART2_Buffer.TxIn  = 0;
	USART2_Buffer.TxOut = 0;
	USART2_Buffer.TxMask = __USART2_TX_BUF - 1;
	USART2_Buffer.RxIn  = 0;
	USART2_Buffer.RxOut = 0;
	USART2_Buffer.RxMask = __USART2_RX_BUF - 1;
	
	// включить тактирование USART2
	BIT_BAND_REG(RCC->APB1ENR, RCC_APB1ENR_USART2EN) = SET;
	// настройка скорости передачи USART2
	USART2->BRR = RCC_GetSysClock(FREQ_PCLK1) / __USART2_BAUDRATE;
	
	USART2->CR1 |= __USART2_CR1;
	USART2->CR2 |= __USART2_CR2;

  USART2->CR1  |= (USART_CR1_RXNEIE |   // включить прерывание по приему
	                 USART_CR1_UE |       // включить модуль USART2
	                 USART_CR1_TE |       // включить передатчик USART2
	                 USART_CR1_RE);       // включить приемник USART2
	
	NVIC_EnableIRQ (USART2_IRQn);         // разрешить прерывания от USART2
}
#endif

#if __USART3_USED
__inline static void USART3_Init(void)
{ 
	// инициализация программного буфера USART3
	USART3_Buffer.TxIn  = 0;
	USART3_Buffer.TxOut = 0;
	USART3_Buffer.TxMask = __USART3_TX_BUF - 1;
	USART3_Buffer.RxIn  = 0;
	USART3_Buffer.RxOut = 0;
	USART3_Buffer.RxMask = __USART3_RX_BUF - 1;
	
	// включить тактирование USART3
	BIT_BAND_REG(RCC->APB1ENR, RCC_APB1ENR_USART3EN) = SET;
	// настройка скорости передачи USART1
	USART2->BRR = RCC_GetSysClock(FREQ_PCLK1) / __USART3_BAUDRATE;
	
	USART3->CR1 |= __USART3_CR1;
	USART3->CR2 |= __USART3_CR2;

  USART3->CR1  |= (USART_CR1_RXNEIE |   // включить прерывание по приему
	                 USART_CR1_UE |       // включить модуль USART3
	                 USART_CR1_TE |       // включить передатчик USART3
	                 USART_CR1_RE);       // включить приемник USART3
	
	NVIC_EnableIRQ (USART3_IRQn);         // разрешить прерывания от USART3
}
#endif

#if (__USART1_USED || __USART2_USED || __USART3_USED)
/** @brief  Инициализация модулей USART.

    Функция производит инициализацию модулей USART в соответствии с настройками через Keil Configuration Wizard в т.ч.:
		- очистку программных буферов
    - включение тактирования модуля USART;
    - настройку формата и скорости обмена;
    - включение прерываний модуля USART;
 */
void USART_Init(void)
{ 
#if __USART1_USED
  USART1_Init();
#endif
#if __USART2_USED
  USART2_Init();
#endif
#if __USART3_USED
  USART3_Init();
#endif
}
#endif

#if __USART1_USED
/** @brief Передача символа через модуль USART1.

    Функция производит запись 1 байта (символа) в программный буфер USART1.
    Если передатчик USART1 выключен, производится включение прерывание по опустошению буфера передачи.
    Передача из программного буфера производится в прерывании.
		
    @param[in] vChar передаваемый символ
		
    @return SUCCESS - операция успешна
    @return ERROR_BUFFER_IS_FULL - буфер полон
 */
ErrorStatus_t USART1_Transmit_IT(const char vChar)
{
	uint16_t Old_Value;
	uint16_t New_Value;
	
	do
	{
		// Запоминаем текущий индекс и возводим флаг эксклюзивного доступа
	  Old_Value = __LDREXH(&USART1_Buffer.TxIn);
		// Вычисление следующего индекса программного буфера
		New_Value = (Old_Value + 1) & USART1_Buffer.TxMask;
		// Проверка на заполненность буфера
	  if (New_Value == USART1_Buffer.TxOut)
	  {
		  New_Value = Old_Value;
		}
	} while(__STREXH(New_Value, &USART1_Buffer.TxIn));
	
	// Если индексы не равны, значит есть место для записи
	if (Old_Value != New_Value)
	{
		//запись значения в буфер
		USART1_Buffer.TxBuf[Old_Value] = vChar;
		
		if(BIT_BAND_REG(USART1->SR, USART_SR_TXE) == SET)
    {
		  // Включить прерывание по опустошению буфера передачи
      BIT_BAND_REG(USART1->CR1, USART_CR1_TXEIE) = SET;
    }
		
	  return SUCCESS;
	}
	else
	{
		return ERROR_USART_BUFFER_IS_FULL;
	}
}

/** @brief Чтение символа принятого из модуля USART1.

    Функция производит чтение 1 байта (символа) из программного буфера USART1.
    Прием символов в программный буфер производится в прерывании.
		
    @param[in] pChar указатель на переменную, в которую производится чтение
		
    @return SUCCESS - операция успешна
    @return ERROR_POINTER_IS_NULL - указатель равен NULL
    @return ERROR_BUFFER_IS_EMPTY - буфер пуст
 */
ErrorStatus_t USART1_Receive_IT(char *const pChar)
{
	if (pChar != NULL)
	{
		// проверка на наличие данных в буфере приемника USART1
    if (USART1_Buffer.RxIn == USART1_Buffer.RxOut)
    {
      return ERROR_USART_BUFFER_IS_EMPTY;
    }
    else
    {
			// чтение значения из буфера
      *pChar = USART1_Buffer.RxBuf[USART1_Buffer.RxOut];
			// установка указателя чтения на следующее место
      USART1_Buffer.RxOut = (USART1_Buffer.RxOut + 1) & USART1_Buffer.RxMask;

      return SUCCESS;
    }
	}
	else
	{
		return ERROR_POINTER_IS_NULL;
	}
}

/** @brief Передача строки через модуль USART1.

    Функция производит запись строки в программный буфер USART1.
    @param[in] pString указатель на передаваемую строку
 */

void USART1_TransmitStr(const char *pString)
{
 	if (pString != NULL)
	{
	  while (*pString)
	  {
		  while (USART1_Transmit_IT(*pString));
		  pString++;
	  }
	}
}
#endif

#if __USART2_USED
/** @brief Передача символа через модуль USART2.

    Функция производит запись 1 байта (символа) в программный буфер USART2.
    Если передатчик USART2 выключен, производится включение прерывание по опустошению буфера передачи.
    Передача из программного буфера производится в прерывании.
		
    @param[in] vChar передаваемый символ
		
    @return SUCCESS - операция успешна
    @return ERROR_BUFFER_IS_FULL - буфер полон
 */
ErrorStatus_t USART2_Transmit_IT(const char vChar)
{
	uint16_t Old_Value;
	uint16_t New_Value;
	
	do
	{
		// Запоминаем текущий индекс и возводим флаг эксклюзивного доступа
	  Old_Value = __LDREXH(&USART2_Buffer.TxIn);
		// Вычисление следующего индекса программного буфера
		New_Value = (Old_Value + 1) & USART2_Buffer.TxMask;
		// Проверка на заполненность буфера
	  if (New_Value == USART2_Buffer.TxOut)
	  {
		  New_Value = Old_Value;
		}
	} while(__STREXH(New_Value, &USART2_Buffer.TxIn));
	
	// Если индексы не равны, значит есть место для записи
	if (Old_Value != New_Value)
	{
		//запись значения в буфер
		USART2_Buffer.TxBuf[Old_Value] = vChar;
		
		if(BIT_BAND_REG(USART2->SR, USART_SR_TXE) == SET)
    {
		  // Включить прерывание по опустошению буфера передачи
      BIT_BAND_REG(USART2->CR1, USART_CR1_TXEIE) = SET;
    }
		
	  return SUCCESS;
	}
	else
	{
		return ERROR_USART_BUFFER_IS_FULL;
	}
}

/** @brief Чтение символа принятого из модуля USART2.

    Функция производит чтение 1 байта (символа) из программного буфера USART2.
    Прием символов в программный буфер производится в прерывании.
		
    @param[in] pChar указатель на переменную, в которую производится чтение
		
    @return SUCCESS - операция успешна
    @return ERROR_POINTER_IS_NULL - указатель равен NULL
    @return ERROR_BUFFER_IS_EMPTY - буфер пуст
 */
ErrorStatus_t USART2_Receive_IT(char *const pChar)
{
	if (pChar != NULL)
	{
		// проверка на наличие данных в буфере приемника USART2
    if (USART2_Buffer.RxIn == USART2_Buffer.RxOut)
    {
      return ERROR_USART_BUFFER_IS_EMPTY;
    }
    else
    {  
			// чтение значения из буфера
      *pChar = USART2_Buffer.RxBuf[USART2_Buffer.RxOut];
			// установка указателя чтения на следующее место
      USART2_Buffer.RxOut = (USART2_Buffer.RxOut + 1) & USART2_Buffer.RxMask;
			
      return SUCCESS;
    }
	}
	else
	{
		return ERROR_POINTER_IS_NULL;
	}
}

/** @brief Передача строки через модуль USART2.

    Функция производит запись строки в программный буфер USART2.
    @param[in] pString указатель на передаваемую строку
 */

void USART2_TransmitStr(const char *pString)
{
	if (pString != NULL)
	{
	  while (*pString)
	  {
		  while (USART2_Transmit_IT(*pString));
		  pString++;
	  }
	}
}
#endif

#if __USART3_USED
/** @brief Передача символа через модуль USART3.

    Функция производит запись 1 байта (символа) в программный буфер USART3.
    Если передатчик USART3 выключен, производится включение прерывание по опустошению буфера передачи.
    Передача из программного буфера производится в прерывании.
		
    @param[in] vChar передаваемый символ
		
    @return SUCCESS - операция успешна
    @return ERROR_BUFFER_IS_FULL - буфер полон
 */
ErrorStatus_t USART3_Transmit_IT(const char vChar)
{
	uint16_t Old_Value;
	uint16_t New_Value;
	
	do
	{
		// Запоминаем текущий индекс и возводим флаг эксклюзивного доступа
	  Old_Value = __LDREXH(&USART3_Buffer.TxIn);
		// Вычисление следующего индекса программного буфера
		New_Value = (Old_Value + 1) & USART3_Buffer.TxMask;
		// Проверка на заполненность буфера
	  if (New_Value == USART3_Buffer.TxOut)
	  {
		  New_Value = Old_Value;
		}
	} while(__STREXH(New_Value, &USART3_Buffer.TxIn));
	
	// Если индексы не равны, значит есть место для записи
	if (Old_Value != New_Value)
	{
		//запись значения в буфер
		USART3_Buffer.TxBuf[Old_Value] = vChar;
		
		if(BIT_BAND_REG(USART3->SR, USART_SR_TXE) == SET)
    {
		  // Включить прерывание по опустошению буфера передачи
      BIT_BAND_REG(USART3->CR1, USART_CR1_TXEIE) = SET;
    }
		
	  return SUCCESS;
	}
	else
	{
		return ERROR_BUFFER_IS_FULL;
	}
}

/** @brief Чтение символа принятого из модуля USART3.

    Функция производит чтение 1 байта (символа) из программного буфера USART3.
    Прием символов в программный буфер производится в прерывании.
		
    @param[in] pChar указатель на переменную, в которую производится чтение
		
    @return SUCCESS - операция успешна
    @return ERROR_POINTER_IS_NULL - указатель равен NULL
    @return ERROR_BUFFER_IS_EMPTY - буфер пуст
 */
ErrorStatus_t USART3_Receive_IT(char *const pChar)
{
	if (pChar != NULL)
	{
		// проверка на наличие данных в буфере приемника USART3
    if (USART3_Buffer.RxIn == USART3_Buffer.RxOut)
    {
      return ERROR_BUFFER_IS_EMPTY;
    }
    else
    {  
			// чтение значения из буфера
      *pChar = USART3_Buffer.RxBuf[USART3_Buffer.RxOut];
			// установка указателя чтения на следующее место
      USART3_Buffer.RxOut = (USART3_Buffer.RxOut + 1) & USART3_Buffer.RxMask;
			
      return SUCCESS;
    }
	}
	else
	{
		return ERROR_POINTER_IS_NULL;
	}
}

/** @brief Передача строки через модуль USART3.

    Функция производит запись строки в программный буфер USART3.
    @param[in] pString указатель на передаваемую строку
 */

void USART3_TransmitStr(const char *pString)
{
	if (pString != NULL)
	{
	  while (*pString)
	  {
		  while (USART3_Transmit_IT(*pString));
		  pString++;
	  }
	}
}
#endif

//  Обработчики прерывания
//----------------------------------------------------------------------------------------------

#if __USART1_USED
/** @brief Обработчик прерывания модуля USART.

   Обработчик производит прием/передачу символов через модуль USART1, а так же обработку ошибок модуля.
   Прием символов производится в программный буфер RxBuf. 
   Передаваемые символы считываются из программного буфера TxBuf.
 */
void USART1_IRQHandler(void)
{
	// Прерывание по приему байта 
	// Флаг USART_SR_RXNE сбрасывается, как только происходит чтение из регистра USART1->DR
	// Флаги USART_SR_NE,USART_SR_FE, USART_SR_PE, USART_SR_ORE сбрасываются, после чтения регистров USART1->SR и USART1->DR
	if(BIT_BAND_REG(USART1->SR, USART_SR_RXNE) == SET)
  {
		// Проверка приемника USART на наличие ошибок
		if ((USART1->SR & (USART_SR_NE|USART_SR_FE|USART_SR_PE|USART_SR_ORE)) == 0)
    {
		  // Вычисление следующего индекса программного буфера
	    uint16_t RxIn_Next = (USART1_Buffer.RxIn + 1) & USART1_Buffer.RxMask;
	
	    if (RxIn_Next != USART1_Buffer.RxOut)
	    {
				// чтение данных из USART1 в буфер приемника
			  USART1_Buffer.RxBuf[USART1_Buffer.RxIn] = USART1->DR;
			  // установка указателя записи на следующее место
        USART1_Buffer.RxIn = RxIn_Next;
			}
			else
			{
				// Программный буфер приемника полон, данные потеряны
			  (void)USART1->DR;
			}
    }
    else 
		{
			// Обработчик ошибок USART
			(void)USART1->DR;
		}
  }
	// Прерывание по опустошению буфера передачи 
	// Флаг USART_SR_TXE сбрасывается только записью в регистр USART1->DR 	
	// Флаг передачи данных USART_SR_TС сбрасывается после чтения регистра USART1->SR и записи в регистр USART1->DR
	if(BIT_BAND_REG(USART1->SR, USART_SR_TXE) == SET)
  {
		//проверка на наличие в буфере передатчика USART1 данных для отправки
		if (USART1_Buffer.TxIn == USART1_Buffer.TxOut)
    {
			// Если данных для передачи нет, выключить прерывание по опустошению буфера передачи
      BIT_BAND_REG(USART1->CR1, USART_CR1_TXEIE) = RESET;
    }
    else
    {
			//запись данных из буфера передатчика в USART1
      USART1->DR = USART1_Buffer.TxBuf[USART1_Buffer.TxOut];
			//установка указателя чтения на следующее место
      USART1_Buffer.TxOut = (USART1_Buffer.TxOut + 1) & USART1_Buffer.TxMask;
    }
  }
}
#endif

#if __USART2_USED
/** @brief Обработчик прерывания модуля USART2.

   Обработчик производит прием/передачу символов через модуль USART2, а так же обработку ошибок модуля.
   Прием символов производится в программный буфер RxBuf. 
   Передаваемые символы считываются из программного буфера TxBuf.
 */
void USART2_IRQHandler(void)
{
	// Прерывание по приему байта 
	// Флаг USART_SR_RXNE сбрасывается, как только происходит чтение из регистра USART2->DR
	// Флаги USART_SR_NE,USART_SR_FE, USART_SR_PE, USART_SR_ORE сбрасываются, после чтения регистров USART2->SR и USART2->DR
	if(BIT_BAND_REG(USART2->SR, USART_SR_RXNE) == SET)
  {
		// Проверка приемника USART на наличие ошибок
		if ((USART2->SR & (USART_SR_NE|USART_SR_FE|USART_SR_PE|USART_SR_ORE)) == 0)
    {
			// Вычисление следующего индекса программного буфера
	    uint16_t RxIn_Next = (USART2_Buffer.RxIn + 1) & USART2_Buffer.RxMask;
	
	    if (RxIn_Next != USART2_Buffer.RxOut)
	    {
				// чтение данных из USART2 в буфер приемника
			  USART2_Buffer.RxBuf[USART2_Buffer.RxIn] = USART2->DR;
        // установка указателя записи на следующее место
        USART2_Buffer.RxIn = RxIn_Next;
			}
			else
			{
				// Программный буфер приемника полон, данные потеряны
			  (void)USART2->DR;
			}
    }
    else 
		{
			// Обработчик ошибок USART
			(void)USART2->DR;
		}
  }
	// Прерывание по опустошению буфера передачи 
	// Флаг USART_SR_TXE сбрасывается только записью в регистр USART2->DR 	
	// Флаг передачи данных USART_SR_TС сбрасывается после чтения регистра USART2->SR и записи в регистр USART2->DR
	if(BIT_BAND_REG(USART2->SR, USART_SR_TXE) == SET)
  {
		//проверка на наличие в буфере передатчика USART2 данных для отправки
		if (USART2_Buffer.TxIn == USART2_Buffer.TxOut)
    {
			// Если данных для передачи нет, выключить прерывание по опустошению буфера передачи
      BIT_BAND_REG(USART2->CR1, USART_CR1_TXEIE) = RESET;
    }
    else
    {
			//запись данных из буфера передатчика в USART2
      USART2->DR = USART2_Buffer.TxBuf[USART2_Buffer.TxOut];
			//установка указателя чтения на следующее место
      USART2_Buffer.TxOut = (USART2_Buffer.TxOut + 1) & USART2_Buffer.TxMask;
    }
  }
}
#endif

#if __USART3_USED
/** @brief Обработчик прерывания модуля USART3.

   Обработчик производит прием/передачу символов через модуль USART3, а так же обработку ошибок модуля.
   Прием символов производится в программный буфер RxBuf. 
   Передаваемые символы считываются из программного буфера TxBuf.
 */
void USART3_IRQHandler(void)
{
	// Прерывание по приему байта 
	// Флаг USART_SR_RXNE сбрасывается, как только происходит чтение из регистра USART3->DR
	// Флаги USART_SR_NE,USART_SR_FE, USART_SR_PE, USART_SR_ORE сбрасываются, после чтения регистров USART3->SR и USART3->DR
	if(BIT_BAND_REG(USART3->SR, USART_SR_RXNE) == SET)
  {
		// Проверка приемника USART на наличие ошибок
		if ((USART3->SR & (USART_SR_NE|USART_SR_FE|USART_SR_PE|USART_SR_ORE)) == 0)
    {
			// Вычисление следующего индекса программного буфера
	    uint16_t RxIn_Next = (USART3_Buffer.RxIn + 1) & USART3_Buffer.RxMask;
	
	    if (RxIn_Next != USART3_Buffer.RxOut)
	    {
				// чтение данных из USART3 в буфер приемника
			  USART3_Buffer.RxBuf[USART3_Buffer.RxIn] = USART3->DR;
        // установка указателя записи на следующее место
        USART3_Buffer.RxIn &= RxIn_Next;
			}
			else
			{
				// Программный буфер приемника полон, данные потеряны
			  (void)USART3->DR;
			}
    }
    else 
		{
			// Обработчик ошибок USART
			(void)USART3->DR;
		}
  }
	// Прерывание по опустошению буфера передачи 
	// Флаг USART_SR_TXE сбрасывается только записью в регистр USART3->DR 	
	// Флаг передачи данных USART_SR_TС сбрасывается после чтения регистра USART3->SR и записи в регистр USART3->DR
	if(BIT_BAND_REG(USART3->SR, USART_SR_TXE) == SET)
  {
		//проверка на наличие в буфере передатчика USART3 данных для отправки
		if (USART3_Buffer.TxIn == USART3_Buffer.TxOut)
    {
			// Если данных для передачи нет, выключить прерывание по опустошению буфера передачи
      BIT_BAND_REG(USART3->CR1, USART_CR1_TXEIE) = RESET;
    }
    else
    {
			//запись данных из буфера передатчика в USART3
      USART3->DR = USART3_Buffer.TxBuf[USART3_Buffer.TxOut];
			//установка указателя чтения на следующее место
      USART3_Buffer.TxOut = (USART3_Buffer.TxOut + 1) & USART3_Buffer.TxMask;
    }
  }
}
#endif

/* КОНЕЦ ФАЙЛА
 *************************************************************************************************/
