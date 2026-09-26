/**
 * @file tsk_config_and_callback.cpp
 * @brief USB CDC 通信测试：与上位机 comm_test_standalone/main.cpp 配对
 *
 * 三组电机各占一帧，两端 id 一一对应：
 *   id=1 底盘 4 轮    id=2 机械臂 4 关节    id=3 腿 4 关节
 *
 * 方向对照（与上位机相反）：
 *   下行 [id=1][4×float velocity]                   16B → 本端 rx（轮子目标速度）
 *        [id=2][id=3][4×{velocity, position}]        32B → 本端 rx（关节目标值）
 *   上行 [id=1][id=2][id=3][4×{velocity, position}]  32B → 本端 tx（电机回传）
 *
 * 下行打包 89B、上行打包 105B，都超过中间件的 kMaxSendSize=64，Send() 会在
 * 帧边界上自动切成多段发；接收侧按包解，不受影响。
 * 注意：本端上行的段 2/段 3 现在会被 CDC_Transmit_HS 的 USBD_BUSY 顶掉
 * （drv_usb 的发送路径还没改），接真板子时只有 id=1 那帧能到上位机。
 *
 * 等价于上位机 Task_Init() 的接线：接底层收发 → Init 中间件 → Register 三帧。
 *
 * 存活判定对齐 2026Robot 的 Class_Orin：接收回调里 Alive_Flag 自增，窗口到点比一次
 * Pre_Alive_Flag，窗口内没涨就判 DISABLE 并抹平下行目标值；只有 ENABLE 才准许运算。
 *
 * 控制周期和判活窗口在 Task_Loop 里按 HAL_GetTick 各自分频：
 *   控制 1ms    —— 运算（现在是回显，将来是控制律）连带上行 Send()，受存活 gate 管
 *   判活 1000ms —— 比一次计数差分，窗口就是这 1000ms
 * 判活不能跟着控制周期走：控制变成 1ms 以后，判活要是也 1ms 比一次，而上位机 100ms
 * 才发一包，就会 99% 的时间误判成断线。
 * 窗口取 1000ms 而不是贴着上位机的 100ms：上位机是 sleep_for(100ms)，实际周期
 * ~100.5ms，窗口跟它一般大就会相位打拍，约每 20s 有一个窗口整好一个包都不落，
 * 误判 DISABLE；而翻回 ENABLE 要等下一个窗口，那 100ms 里 1ms 的控制律整段被 gate 掉。
 * 2026Robot 的 Class_Orin 也是 1000ms 窗口（函数名就写着），对齐它。
 */

#include <cstddef>
#include <cstdint>

#include "tsk_config_and_callback.h"
#include "gpio.h"
#include "drv_usb.h"
#include "Communication_Interface.hpp"
#include "motor_base.hpp"

namespace
{

using Control_Frame::Struct_Motor_Base;
using Control_Frame::USB_Communication_Interface;

constexpr size_t   kJoints    = 4;    // 每组 4 个电机
constexpr size_t   kGroups    = 3;    // 底盘 / 机械臂 / 腿
constexpr uint8_t  kIdChassis = 1;
constexpr uint8_t  kIdArm     = 2;
constexpr uint8_t  kIdLeg     = 3;
constexpr uint32_t kControlPeriodMs = 1;     // 控制/回显周期
constexpr uint32_t kAlivePeriodMs   = 1000;  // 判活窗口，远大于上位机 100ms 的发送周期

// 上行回传槽位：底盘 [kUpChassis..+3] / 臂 / 腿，三块紧挨着排
constexpr size_t kUpChassis = 0;
constexpr size_t kUpArm     = kJoints;
constexpr size_t kUpLeg     = 2 * kJoints;

#pragma pack(push, 1)
// 下行：底盘 4 个轮子的目标速度
struct Struct_Chassis_Down
{
    float velocity[kJoints];
};

// 下行：关节的目标速度 + 目标位置。故意不复用 motor_base.hpp 里的回传结构体，
// 上位机下位机各自按同一份布局写一遍，用来验证两边对内存布局的理解是否一致。
struct Joint_Target
{
    float velocity;
    float position;
};

struct Struct_Joint_Down
{
    Joint_Target target[kJoints];
};

// 上行：三组共 12 个电机的回传
struct Struct_All_Up
{
    Struct_Motor_Base motor[kGroups * kJoints];
};
#pragma pack(pop)

Struct_Chassis_Down Chassis_Down;
Struct_Joint_Down   Arm_Down;
Struct_Joint_Down   Leg_Down;
Struct_All_Up       Up;

constexpr uint8_t kUpSize = sizeof(Struct_Motor_Base) * kJoints;  // 单组回传 32B

uint32_t Last_Control_Tick = 0;
uint32_t Last_Alive_Tick   = 0;
bool     Ready             = false;

// 上位机存活：滑动窗口计数差分（对齐 dvc_nvidiaorin 的 Flag / Pre_Flag）
uint32_t  Alive_Flag     = 0;
uint32_t  Pre_Alive_Flag = 0;
Enum_Alive Alive_Status  = Alive_Status_DISABLE;

// 中间件的发送回调是裸函数指针，只能绑定无捕获函数（与上位机 SendCallback 等价）
int64_t SendCallback(uint8_t * data, size_t length)
{
    return USB_Transmit_Async(data, static_cast<uint16_t>(length)) == USB_Status::Ok
        ? static_cast<int64_t>(length) : -1;
}

// 底层 USB 的接收回调，在 USB 中断里被调用，直接转发给中间件就地解包分发
void RxCallback(uint8_t * data, uint16_t length)
{
    //滑动窗口, 判断上位机是否在线
    Alive_Flag += 1;

    USB_Communication_Interface.Rx_RptlCallback(data, length);
}

// 掉线时把下行目标值抹平，不留旧值（对齐 Orin 在判死分支里清 Rx_Data）
void Invalidate_Control_Data()
{
    Chassis_Down = {};
    Arm_Down     = {};
    Leg_Down     = {};
}

/**
 * @brief 每 1000ms 检测一次上位机链路是否存活，由 Task_Loop 调用
 *
 */
void TIM_1000ms_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过上位机数据
    if (Alive_Flag == Pre_Alive_Flag)
    {
        // 上位机断开连接
        Alive_Status = Alive_Status_DISABLE;
        Invalidate_Control_Data();
    }
    else
    {
        // 上位机保持连接
        Alive_Status = Alive_Status_ENABLE;
    }
    Pre_Alive_Flag = Alive_Flag;
}

}  // namespace

void Task_Init()
{
    USB_Init(RxCallback);
    USB_Communication_Interface.Init(SendCallback);

    // 三帧分别注册，不看短路，三个都试一遍
    const bool ok_chassis = USB_Communication_Interface.Register(
        kIdChassis, &Up.motor[kUpChassis], &Chassis_Down, kUpSize, sizeof(Chassis_Down));
    const bool ok_arm = USB_Communication_Interface.Register(
        kIdArm, &Up.motor[kUpArm], &Arm_Down, kUpSize, sizeof(Arm_Down));
    const bool ok_leg = USB_Communication_Interface.Register(
        kIdLeg, &Up.motor[kUpLeg], &Leg_Down, kUpSize, sizeof(Leg_Down));

    Ready = ok_chassis && ok_arm && ok_leg;
    if (!Ready)
    {
        // 注册失败就常亮；心跳已删，正常跑 LED 一直不亮
        LED_ON();
    }
}

void Task_Loop()
{
    static uint32_t mod10 = 0;

    if (!Ready)
    {
        return;
    }

    const uint32_t now = HAL_GetTick();

    // 判活单独 1000ms 分频，且排在控制 gate 之前，
    // 免得控制周期把判活那一拍吃掉
    if (now - Last_Alive_Tick >= kAlivePeriodMs)
    {
        Last_Alive_Tick = now;
        TIM_1000ms_Alive_PeriodElapsedCallback();
    }

    if (now - Last_Control_Tick < kControlPeriodMs)
    {
        return;
    }
    Last_Control_Tick = now;

    // 1ms 任务
    TIM_USB_Send_PeriodElapsedCallback();

    // 10ms 任务
    if (++mod10 >= 10)
    {
        mod10 = 0;

        // 只有存活才准许运算：断线时回显整段不跑，Up 保持上一拍的旧值。
        // 抹平的是下行目标值，给 gate 之外任何读 Chassis_Down/Arm_Down/Leg_Down 的代码兜底。
        if (Alive_Status == Alive_Status_ENABLE)
        {
            for (size_t k = 0; k < kJoints; ++k)
            {
                // 底盘：速度原样回显（上位机可做位精确比对），位置给固定值
                // （上位机能看出每一帧落到了正确的槽位）
                Up.motor[kUpChassis + k].velocity = Chassis_Down.velocity[k];
                Up.motor[kUpChassis + k].position = static_cast<float>(k + 1) * 1000.0f + 0.25f;

                // 臂 / 腿：目标值原样回显，位精确，能同时验证速度与位置两条链路
                Up.motor[kUpArm + k].velocity = Arm_Down.target[k].velocity;
                Up.motor[kUpArm + k].position = Arm_Down.target[k].position;

                Up.motor[kUpLeg + k].velocity = Leg_Down.target[k].velocity;
                Up.motor[kUpLeg + k].position = Leg_Down.target[k].position;
            }
        }

        // 上行照发：收不到上位机不代表下位机要隐身
        USB_Communication_Interface.Send();
    }
}

/**
 * @brief 获取上位机链路存活状态
 * @return Enum_Alive 存活状态
 */
Enum_Alive Get_Alive_Status()
{
    return Alive_Status;
}

/************************ COPYRIGHT(C) NEUQ-RoboPioneers **************************/
