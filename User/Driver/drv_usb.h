/**
 * @file drv_usb.h
 * @author lyh
 * @brief  USB虚拟串口通信初始化与配置流程
 * @version 0.1
 * @date 2026-09-21
 *
 * @copyright
 *
 */

#ifndef DRV_USB_H
#define DRV_USB_H

/* Includes ------------------------------------------------------------------*/

#include <cstdint>

/* Exported macros -----------------------------------------------------------*/

// 发送缓冲区的槽位
#define USB_TX_BUFFER_SLOT      10

/* Exported types ------------------------------------------------------------*/


/**
 * @brief USB通信接收回调函数指针
 *
 */
using USB_Rx_Callback = void (*)(uint8_t *buffer, uint16_t length);

enum class USB_Status : uint8_t
{
    Ok = 0,
    Failed,
    Timeout,
};


/* Exported variables --------------------------------------------------------*/


/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化USB
 *
 * @param rx_callback 接收回调函数
 */
void USB_Init(USB_Rx_Callback rx_callback);

/**
 * @brief 非阻塞式发送, 调用后立马返回
 * @param Buf 发送缓冲区
 * @param Len 缓冲区长度
 * @return USB_Status
 */
USB_Status USB_Transmit(uint8_t* Buf, uint16_t Len);

/**
 * @brief 阻塞式发送, 内部会忙等待发送完成标志, 再返回
 * @param Buf 发送缓冲区
 * @param Len 缓冲区长度
 * @param TimeOut 超时时间, 毫秒
 * @return USB_Status
 */
USB_Status USB_Transmit_Blocked(uint8_t* Buf, uint16_t Len, uint32_t TimeOut);

/**
 * @brief 异步发送, 仅把数据加入发送环形缓冲区, 在定时回调执行发送
 * @param Buf 发送缓冲区
 * @param Len 缓冲区长度, 大于64会被截断
 * @return USB_Status
 */
USB_Status USB_Transmit_Async(uint8_t* Buf, uint16_t Len);

/**
 * @brief 清除发送缓冲区
 */
void USB_Clear_Transmit_Buffer();

/**
 * @brief USB的TIM定时器中断发送回调函数, 调用周期即为发送周期
 * @note 实测每次发送间隔至少1ms, 若连续调用必丢包
 *
 */
void TIM_USB_Send_PeriodElapsedCallback();


#endif

/************************ COPYRIGHT(C) NEUQ-RoboPioneers **************************/
