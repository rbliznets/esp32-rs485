/*!
	\file
	\brief Класс управления RS485 через UART.
	\authors Близнец Р.А. (r.bliznets@gmail.com)
	\version 1.1.0.0
	\date 22.11.2023
*/

#pragma once

#include "sdkconfig.h"
#include "CBaseTask.h"
#include "CSoftwareTimer.h"

#include "esp_pm.h"
#include "driver/uart.h"

#define RS485_TX_BUF (4*1024) ///< Размер буфера UART на передачу.
#define RS485_RX_BUF (4*1024) ///< Размер буфера UART на прием.
#define RS485_EVEN_BUF (20) ///< Размер очереди UART.

#define RS485_DATA_BUF (RS485_RX_BUF) ///< Максимальный размер принимаемого пакета.

#ifdef CONFIG_ESP_TASK_WDT
#define TASK_MAX_BLOCK_TIME pdMS_TO_TICKS((CONFIG_ESP_TASK_WDT_TIMEOUT_S - 1) * 1000 + 500)
#else
#define TASK_MAX_BLOCK_TIME portMAX_DELAY
#endif

#define MSG_END_TASK (0)  ///< Код команды завершения задачи.
#define MSG_SEND_DATA (1) ///< Код команды отправки данных.
#define MSG_485_TIMEOUT (2)

/*
 * Параметры задач
 */
#define RS485TASK_NAME "rs485"		   ///< Имя задачи для отладки.
#define RS485TASK_STACKSIZE (3 * 1024) ///< Размер стека задачи (296).
#define RS485TASK_LENGTH (30)		   ///< Длина приемной очереди задачи.

/// Функция события приема данных.
/*!
 * \param[in] data данные.
 * \param[in] size размер данных.
 */
typedef void onDataRx(char *data, size_t size);

/// @brief Конфигурация драйвера rs-485
struct SRS485Config
{
	onDataRx *onUartDataRx = nullptr; ///< Функция события приема данных

	uint8_t cpu = 1;   ///< Номер ядра процессора.
	uint8_t prior = 3; ///< Приоритет задачи.

	uart_port_t port = UART_NUM_0;					///< Номер UART.
	uart_mode_t mode = UART_MODE_RS485_HALF_DUPLEX; ///< Режим работы
	int8_t pin_tx = 43;								///< Номер вывода TX
	int8_t pin_rx = 44;								///< Номер вывода RX
	int8_t pin_de = 40;								///< Номер вывода DE
#if CONFIG_PM_ENABLE
	uint32_t blockSleep = 0;
#endif
};

/// Класс управления RS485 через UART.
class CRS485Task : public CBaseTask
{
protected:
#if CONFIG_PM_ENABLE
	esp_pm_lock_handle_t mPMLock; ///< флаг запрета на понижение частоты CPU
	CSoftwareTimer *mRS485Timer = nullptr;
	bool mLock = false;
#endif
	SRS485Config mConfig; ///< Конфигурация драйвера rs-485

	QueueSetHandle_t mQueueSet; ///< набор очередей
	QueueHandle_t m_uart_queue; ///< очередь UART
	char mBuf[RS485_DATA_BUF];	///< буфер принятого пакета

	bool mRun = false; ///< Флаг работы UART
	/// Включить UART.
	void initUart();
	/// Выключить UART.
	void deinitUart();

	/// Функция задачи.
	virtual void run() override;

	using CBaseTask::sendCmd;

public:
	/// Конструктор.
	/*!
	  \param[in] cfg Указатель на конфигурация драйвера.
	*/
	CRS485Task(SRS485Config *cfg);

	/// Деструктор.
	virtual ~CRS485Task();

	/// Послать данные в UART.
	/*!
	  \param[in] data данные.
	  \param[in] size размер данных.
	  \param[in] xTicksToWait Время ожидания в тиках.
	  \return true в случае успеха.
	*/
	bool sendData(char *data, size_t size, TickType_t xTicksToWait = 1);
};
