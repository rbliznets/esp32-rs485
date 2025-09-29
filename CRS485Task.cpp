/*!
	\file
	\brief Класс управления RS485 через UART.
	\authors Близнец Р.А. (r.bliznets@gmail.com)
	\version 1.1.0.0
	\date 22.11.2023
*/

#include "CRS485Task.h"
#include "CTrace.h"
#include "driver/gpio.h"
#include "driver/uart_wakeup.h"
#include "esp_sleep.h"
#include <cstring>

CRS485Task::CRS485Task(SRS485Config *cfg) : CBaseTask()
{
	std::memcpy(&mConfig, cfg, sizeof(SRS485Config));
	mQueueSet = xQueueCreateSet(RS485TASK_LENGTH + RS485_EVEN_BUF);

#if CONFIG_PM_ENABLE
	esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "rs485", &mPMLock);
#endif
	CBaseTask::init(RS485TASK_NAME, RS485TASK_STACKSIZE, mConfig.prior, RS485TASK_LENGTH, mConfig.cpu);
}

CRS485Task::~CRS485Task()
{
	sendCmd(MSG_END_TASK);
	do
	{
		vTaskDelay(1);
	}
#if (INCLUDE_vTaskDelete == 1)
	while (mTaskHandle != nullptr);
#else
	while (mTaskQueue != nullptr);
#endif

	vQueueDelete(mQueueSet);
#if CONFIG_PM_ENABLE
	esp_pm_lock_delete(mPMLock);
#endif
}

void CRS485Task::initUart()
{
	if (!mRun)
	{
		uart_config_t uart_config = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = 3,
			.source_clk = UART_SCLK_DEFAULT,
			.flags = {0, 0}};
		int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
		intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

#if CONFIG_PM_ENABLE
		// esp_pm_lock_acquire(mPMLock);
#endif
		ESP_ERROR_CHECK(uart_driver_install(mConfig.port, RS485_RX_BUF, RS485_TX_BUF, RS485_EVEN_BUF, &m_uart_queue, intr_alloc_flags));
		xQueueAddToSet(m_uart_queue, mQueueSet);
		ESP_ERROR_CHECK(uart_param_config(mConfig.port, &uart_config));
		ESP_ERROR_CHECK(uart_set_rx_timeout(mConfig.port, 1));
		if (mConfig.mode == UART_MODE_UART)
		{
			ESP_ERROR_CHECK(uart_set_pin(mConfig.port, mConfig.pin_tx, mConfig.pin_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
			if (mConfig.pin_de >= 0)
			{
				gpio_iomux_in(mConfig.pin_de, 1);
				gpio_iomux_out(mConfig.pin_de, 1, false);
				gpio_set_direction((gpio_num_t)mConfig.pin_de, GPIO_MODE_OUTPUT);
				gpio_set_pull_mode((gpio_num_t)mConfig.pin_de, GPIO_PULLUP_ONLY);
				gpio_set_level((gpio_num_t)mConfig.pin_de, 1);
			}
		}
		else
		{
			ESP_ERROR_CHECK(uart_set_pin(mConfig.port, mConfig.pin_tx, mConfig.pin_rx, mConfig.pin_de, UART_PIN_NO_CHANGE));
		}
		ESP_ERROR_CHECK(uart_set_mode(mConfig.port, mConfig.mode));
		ESP_ERROR_CHECK(uart_flush(mConfig.port));
		mRun = true;

#if CONFIG_PM_ENABLE
		// wakeupConfig();
		uart_wakeup_cfg_t cfg = {UART_WK_MODE_ACTIVE_THRESH,3};
		uart_wakeup_setup(mConfig.port, &cfg);
#endif
		// LOG("CRS485Task Run");
	}
}

void CRS485Task::wakeupConfig()
{
	// ESP_ERROR_CHECK(gpio_sleep_set_direction((gpio_num_t)mConfig.pin_rx, GPIO_MODE_INPUT));
	// ESP_ERROR_CHECK(gpio_sleep_set_pull_mode((gpio_num_t)mConfig.pin_rx, GPIO_PULLUP_ONLY));
	// rtc_gpio_wakeup_enable(gpio_num_t(mConfig.pin_rx), GPIO_INTR_HIGH_LEVEL);
	ESP_ERROR_CHECK(uart_set_wakeup_threshold(mConfig.port, 3));
	ESP_ERROR_CHECK(esp_sleep_enable_uart_wakeup(mConfig.port));
}

void CRS485Task::deinitUart()
{
	if (mRun)
	{
		xQueueRemoveFromSet(m_uart_queue, mQueueSet);

		ESP_ERROR_CHECK(uart_wait_tx_done(mConfig.port, pdMS_TO_TICKS(150)));
		ESP_ERROR_CHECK(uart_driver_delete(mConfig.port));
#if CONFIG_PM_ENABLE
		uart_wakeup_clear(mConfig.port, UART_WK_MODE_ACTIVE_THRESH);
#endif
		vTaskDelay(pdMS_TO_TICKS(10));
		mRun = false;
#if CONFIG_PM_ENABLE
		esp_pm_lock_release(mPMLock);
#endif
		if (mConfig.pin_de >= 0)
			gpio_set_direction((gpio_num_t)mConfig.pin_de, GPIO_MODE_DISABLE);

		// LOG("CRS485Task Stop");
	}
}

void CRS485Task::run()
{
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
	UBaseType_t m1 = uxTaskGetStackHighWaterMark2(nullptr);
#endif
	STaskMessage msg;
	xQueueAddToSet(mTaskQueue, mQueueSet);
	QueueSetMemberHandle_t xActivatedMember;

	initUart();

	for (;;)
	{
		xActivatedMember = xQueueSelectFromSet(mQueueSet, TASK_MAX_BLOCK_TIME);
		if (xActivatedMember == nullptr)
			continue;
		if (xActivatedMember == m_uart_queue)
		{
			uart_event_t event;
			int read_len;
			while (xQueueReceive(m_uart_queue, &event, 0) == pdTRUE)
			{
				switch (event.type)
				{
				case UART_DATA:
					// TDEC("!",event.size);
					while (event.size > 0)
					{
						if (event.size <= sizeof(mBuf))
						{
							read_len = uart_read_bytes(mConfig.port, mBuf, event.size, 0);
						}
						else
						{
							read_len = uart_read_bytes(mConfig.port, mBuf, sizeof(mBuf), 0);
						}
						if (read_len > 0)
						{
							event.size -= read_len;
							if (mConfig.onUartDataRx != nullptr)
								mConfig.onUartDataRx(mBuf, read_len);
						}
						else
						{
							event.size = 0;
						}
					}
					break;
				case UART_FIFO_OVF:
					TRACE_W("CRS485Task: HW FIFO Overflow", event.type, false);
					uart_flush_input(mConfig.port);
					xQueueReset(m_uart_queue);
					break;
				case UART_BUFFER_FULL:
					TRACE_W("CRS485Task: Ring Buffer Full", event.type, false);
					uart_flush_input(mConfig.port);
					xQueueReset(m_uart_queue);
					break;
				case UART_BREAK:
					TRACE_W("CRS485Task: Rx Break", event.type, false);
					break;
				case UART_PARITY_ERR:
					TRACE_W("CRS485Task: Parity Error", event.type, false);
					break;
				case UART_FRAME_ERR:
					TRACE_W("CRS485Task: Frame Error", event.type, false);
					break;
				case UART_PATTERN_DET:
					break;
				case UART_WAKEUP:
					uart_flush(mConfig.port);
					esp_pm_lock_acquire(mPMLock);
					// deinitUart();
					// initUart();
					break;
				default:
					TRACE_WARNING("CRS485Task unknown uart event type", event.type);
					break;
				}
			}
		}
		else if (xActivatedMember == mTaskQueue)
		{
			bool collision_flag = false;
			size_t size;
			while (getMessage(&msg))
			{
				switch (msg.msgID)
				{
				case MSG_SEND_DATA:
					if (mConfig.mode == UART_MODE_RS485_COLLISION_DETECT)
					{
						if(uart_get_buffered_data_len(mConfig.port,&size) == ESP_OK)
						{
							if(size != 0)
							{
								// TDEC("size",size);
								collision_flag = true;
								sendMessageFront(&msg, 0, true);
								break;
							}
						}
						uart_write_bytes(mConfig.port, msg.msgBody, msg.shortParam);
						if (uart_wait_tx_done(mConfig.port, pdMS_TO_TICKS(TASK_MAX_BLOCK_TIME)) == ESP_OK)
						{
							if (uart_get_collision_flag(mConfig.port, &collision_flag) == ESP_OK)
							{
								if (collision_flag)
								{
									TRACE_E("CRS485Task collision", collision_flag, false);
									sendMessageFront(&msg, 0, true);
									break;
								}
							}
						}
						else
						{
							TRACE_E("uart_wait_tx_done", -1, false);
						}
						vPortFree(msg.msgBody);
					}
					else
					{
						uart_write_bytes(mConfig.port, msg.msgBody, msg.shortParam);
						vPortFree(msg.msgBody);
						uart_wait_tx_done(mConfig.port, pdMS_TO_TICKS(TASK_MAX_BLOCK_TIME));
					}
					break;
				case MSG_END_TASK:
					goto endTask;
				default:
					TRACE_WARNING("CRS485Task:unknown message", msg.msgID);
					break;
				}
				if (collision_flag)
					break;
			}
		}
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
		UBaseType_t m2 = uxTaskGetStackHighWaterMark2(nullptr);
		if (m2 != m1)
		{
			m1 = m2;
			TDEC("free rs485 stack", m2);
		}
#endif
	}
endTask:
	deinitUart();
	xQueueRemoveFromSet(mTaskQueue, mQueueSet);
}

bool CRS485Task::sendData(char *data, size_t size, TickType_t xTicksToWait)
{
	assert(data != nullptr);
	assert(size < RS485_TX_BUF);

	STaskMessage msg;

	uint8_t *dt = allocNewMsg(&msg, MSG_SEND_DATA, size, true);
	std::memcpy(dt, data, size);
	return sendMessage(&msg, xTicksToWait, true);
}
