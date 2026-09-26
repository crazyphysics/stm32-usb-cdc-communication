/**
 * @file drv_usb.cpp
 * @author lyh
 * @brief  USB虚拟串口通信初始化与配置流程
 * @version 0.1
 * @date 2026-09-21
 *
 * @copyright
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_usb.h"
#include "usbd_cdc_if.h"
#include "alg_circular_buffer.h"
#include <array>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

USB_Rx_Callback g_rx_callback = nullptr;

volatile bool g_tx_process_flag = false;
volatile bool g_rx_process_flag = false;

static Algorithm::Class_Circular_Buffer<std::array<uint8_t, 64>, USB_TX_BUFFER_SLOT> g_tx_buffer = {};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化USB
 *
 * @param rx_callback 接收回调函数
 */
void USB_Init(USB_Rx_Callback rx_callback)
{
	g_rx_callback = rx_callback;
	g_tx_process_flag = false;
}

/**
 * @brief USB异步发送, 把数据加入发送环形缓冲区
 * @param Buf 发送缓冲区
 * @param Len 缓冲区长度, 大于64会被截断
 * @return USB_Status
 */
USB_Status USB_Transmit_Async(uint8_t* Buf, uint16_t Len)
{
	if (Len > 64) Len = 64;

	std::array<uint8_t, 64> tx_buffer = {};
	memcpy(tx_buffer.data(), Buf, Len);
	if (g_tx_buffer.Push(tx_buffer))
	{
		return USB_Status::Ok;
	}
	return USB_Status::Failed;
}

/**
 *
 * @param Buf 发送缓冲区
 * @param Len 缓冲区长度
 * @return USB_Status
 */
USB_Status USB_Transmit(uint8_t* Buf, uint16_t Len)
{
	auto res = CDC_Transmit_HS(Buf, Len);
	if (res == USBD_OK)
	{
		g_tx_process_flag = true;
	}
	return (res == USBD_OK) ? USB_Status::Ok : USB_Status::Failed;
}

/**
 *
 * @param Buf 发送缓冲区
 * @param Len 缓冲区长度
 * @param TimeOut 超时时间, 毫秒
 * @return USB_Status
 */
USB_Status USB_Transmit_Blocked(uint8_t* Buf, uint16_t Len, uint32_t TimeOut)
{
	// 先置标志位, 防止后面立马完成而失效
	g_tx_process_flag = true;
	auto res = CDC_Transmit_HS(Buf, Len);
	// 发送失败
	if (res != USBD_OK)
	{
		g_tx_process_flag = false;
		return USB_Status::Failed;
	}

	// 忙等待发送结束
	const uint32_t start = HAL_GetTick();
	while (g_tx_process_flag)
	{
		if (HAL_GetTick() - start >= TimeOut)
		{
			g_tx_process_flag = false;
			return USB_Status::Timeout;
		}
	}

	return USB_Status::Ok;
}

void USB_Clear_Transmit_Buffer()
{
	g_tx_buffer.Clear();
}

/**
 * @brief USB的TIM定时器中断发送回调函数, 调用周期即为发送周期
 * @note 实测每次发送间隔至少1ms, 若连续调用必丢包
 *
 */
void TIM_USB_Send_PeriodElapsedCallback()
{
	std::array<uint8_t, 64> tx_buffer = {};

	// 先看发送缓冲区有没有数据
	if (g_tx_buffer.Pop(tx_buffer) == false) return;

	// 发送缓冲区存的数据
	USB_Transmit(tx_buffer.data(), 64);
}


extern "C" void HAL_CDC_TxCpltCallback(void)
{
	g_tx_process_flag = false;
}

extern "C" void HAL_CDC_RxCpltCallback(uint8_t* pRxBuffer, uint16_t Length)
{
	g_rx_process_flag = true;
	if (g_rx_callback != nullptr)
	{
		g_rx_callback(pRxBuffer, Length);
	}
	g_rx_process_flag = false;
}

/************************ COPYRIGHT(C) NEUQ-RoboPioneers **************************/
