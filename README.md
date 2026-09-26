## USB-CDC驱动收发处理

### 文件位置
- USB_DEVICE/App/usbd_cdc_if.c
- USB_DEVICE/App/usbd_cdc_if.h

### 接口说明

```cpp

// 弱定义以下HAL库风味函数

__weak void HAL_CDC_RxCpltCallback(uint8_t* pRxData, uint16_t Length);
__weak void HAL_CDC_TxCpltCallback(void);

// 在以下地方有调用

static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
  // 先交给应用层处理(回调内部会立即 memcpy 搬走数据), 再重新武装端点。
  // 若先武装再回调, 下一个USB包可能在本回调还在解析时就把同一块
  // UserRxBufferHS 覆盖掉。
  HAL_CDC_RxCpltCallback(Buf, (uint16_t)(*Len));

  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);

  return (USBD_OK);
  /* USER CODE END 11 */
}

static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  // 调用发送完成中断回调
  HAL_CDC_TxCpltCallback();

  /* USER CODE END 14 */
  return result;
}
```

## USB-CDC驱动封装

### 文件位置
- User/Driver/drv_usb.cpp
- User/Driver/drv_usb.h

### 接口说明
- 提供非阻塞发送, 阻塞式发送, 异步发送
  - 非阻塞发送: 调用后立马返回, 不会等到执行结束
  - 阻塞式发送: 内部忙等待发送完成标志位, 再返回
  - 异步发送: 不做发送处理, 仅把数据放进环形缓冲区, 在定时回调里面取用缓冲区数据
- 必须定时调用回调
  - 调用周期即为发送周期
  - 实测每次发送间隔至少1ms, 若连续调用必丢包

## 通信协议交互

### 文件位置
- User/Middleware/Communication_Interface.cpp
- User/Middleware/Communication_Interface.hpp

测试用例
User/Task/tsk_config_and_callback.cpp