/**
 * @file tsk_config_and_callback.cpp
 * @brief 下位机通信任务：与上位机（2027Robot）ros2_control 的 hardware interface 配对
 *
 * 上位机把在线帧收敛成两帧，一个结构体一个 id：
 *   id=1 底盘 4 轮      id=3 腿 4 关节
 * 本端注册表与上位机 chassis_system / leg_system 的 Register() 一一对应。
 * 线格式没有长度字段，收侧靠 registry 按 id 查帧长，尺寸对不上就整体错位。
 *
 * 方向对照（下位机视角）：
 *   下行 [id=1][4×float velocity]                      16B → 本端 rx（轮子目标速度）
 *        [id=3][4×float position]                      16B → 本端 rx（腿关节目标位置）
 *   上行 [id=1][4×{velocity,position,alive_flag}]      36B → 本端 tx（底盘电机回传）
 *        [id=3][4×{velocity,position,alive_flag}]      36B → 本端 tx（腿电机回传）
 *
 * 打包：下行 19+19=38B，在中间件 kMaxSendSize=64 以内，上位机 Send() 打一包发。
 *       上行 39+39=78B，超过 64，本端 Send() 会在帧边界切成两段，每段一个 USB 传输。
 *       上位机先 configure 哪个包由 resource_manager 决定，帧序不固定；两侧都按 id
 *       查表逐帧走，所以顺序无所谓。
 *
 * 上层尺寸由下面的 static_assert 卡死：Struct_Motor_Base 一旦漏掉 alive_flag 就是
 * 8B 而不是 9B，4 个电机差 4 字节。这种错要在编译期拦住，别放到板子上靠现象反推。
 *
 * 等价于上位机 Task_Init() 的接线：接底层收发 → Init 中间件 → Register 两帧。
 *
 * 存活判定对齐上位机的 Alive：接收回调里 Alive_Flag 自增，窗口到点比一次
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
 *
 * 腿帧 slot 0 挂了一台真电机当「一条腿」用：C610（M2006 无刷 + 36:1 减速箱），
 * 接在 FDCAN1 上，经典帧 1 Mbps，反馈 ID 0x201。
 *
 * 反馈方向：FDCAN_Init 配 accept-all 滤波 + RX FIFO0 中断，电调 1kHz 推的反馈帧在
 * 中断里交给 dvc_motor_dji 解算，再由 10ms 运算拍写进 Leg_Up.motor[0]
 * （输出轴角度 rad / 角速度 rad/s）。
 *
 * 控制方向：底层跑一个串级位置环，两个环都只有 P（见下面 kAngle_K_P / kOmega_K_P）。
 * 目标角度直接取上位机下发腿帧的 position[0]，单位就是输出轴 rad——上位机发 0.5
 * 就转到 0.5 rad 停住，发 0 就回上电时的位置。1ms 一拍：Set_Target_Angle →
 * C610 的角度环→速度环→电流写进 Tx 缓冲区 → TIM_1ms_CAN_PeriodElapsedCallback 发 0x200 帧。
 * 断链时目标强制给 0，但帧照发：C610 收不到 ~1kHz 控制帧会报错停机。
 * 注意角度是「以本板上电时刻为零点、多圈累加」的，发小角度之前先想清楚电机已经转过几圈，
 * 否则它会一路倒回去。
 *
 * 剩下 3 槽仍是槽位标识常量，所以上位机那 4 个位置里只有第 1 个是活的。
 *
 * 已知限制（drv_usb 本轮未改）：上行每个 39B 的段都会被 USB_Transmit_Async 截断成
 * 固定 64B 补零发出（槽位不存长度），上位机解析完帧读到 id=0 就 break，无害；
 * 但 USBD_BUSY 时槽位已 Pop 出去且不重入队，那一帧静默丢失——腿帧（id=3）可能间歇性到不了。
 */

#include <cstddef>
#include <cstdint>

#include "Communication_Interface.hpp"
#include "drv_can.h"
#include "drv_usb.h"
#include "dvc_motor_dji.h"
#include "gpio.h"
#include "motor_base.hpp"
#include "tsk_config_and_callback.h"

namespace
{

using Control_Frame::Struct_Motor_Base;
using Control_Frame::USB_Communication_Interface;

constexpr size_t kJoints = 4;             // 每组 4 个电机
constexpr uint8_t kIdChassis = 1;         // 与上位机 chassis_id 默认值一致
constexpr uint8_t kIdLeg = 3;             // 与上位机 leg_id 默认值一致
constexpr uint32_t kControlPeriodMs = 1;  // 控制/回显周期
constexpr uint32_t kAlivePeriodMs = 1000; // 判活窗口，远大于上位机 100ms 的发送周期

constexpr uint32_t kMotorIdC610 = 0x201;      // C610 反馈帧的标准帧 ID（电调 ID 1）
constexpr uint32_t kMotorAlivePeriodMs = 100; // C610 反馈是 1kHz，掉线检测窗口取 100ms

// 位置环是串级，两个环都只给 P。参数刻意给得很小，第一次上电先看它动得温柔不温柔：
//   外环 PID_Angle : 误差 rad   → 目标角速度 rad/s   （K_P 单位 1/s）
//   内环 PID_Omega : 误差 rad/s → 电流 A             （C610 内部再 ×1000 变成 ±10000 原始值）
// 两个 Out_Max 是两道互相独立的硬闸：外环最多要 3 rad/s，内环最多给 0.5A。
// 正常跟踪时电流其实由外环卡着：最多 kOmega_K_P × kAngle_Out_Max = 0.3A；
// 0.5A 那道闸只在电机反向、速度误差瞬间变大时才咬得到。C610 自己允许 10A，所以很宽裕。
// 调法（按顺序，一次只动一个）：
//   一动不动 → 加 kAngle_K_P（先别急着加 kOmega_K_P）
//   快到目标时来回冲/超调 → 减 kAngle_K_P
//   抖动/啸叫/发热 → 减 kOmega_K_P
// 注意纯 P 的位置环在到位后没有保持力矩（误差 0 → 电流 0），手拨会偏，这是预期的；
// 要"锁死"得补 I，但 I_Out_Max 和 K_I 必须同时给（alg_pid.cpp 里会算 I_Out_Max/K_I）。
constexpr float kAngle_K_P = 1.0f;     // 1/s
constexpr float kAngle_Out_Max = 3.0f; // rad/s
constexpr float kOmega_K_P = 0.1f;     // A / (rad/s)
constexpr float kOmega_Out_Max = 0.5f; // A

#pragma pack(push, 1)
// 下行：底盘 4 个轮子的目标速度。对应上位机 chassis::Struct_Motor_Tx，16B。
struct Struct_Chassis_Down
{
    float velocity[kJoints];
};

// 下行：腿 4 个关节的目标位置。对应上位机 leg::Struct_Motor_Tx，16B。
// 故意不复用 motor_base.hpp 里的回传结构体：两端各按同一份布局写一遍，
// 用来验证两边对内存布局的理解是否一致。
struct Struct_Leg_Down
{
    float position[kJoints];
};

// 上行：一组 4 个电机的回传。对应上位机 *::Struct_Motor_Rx，36B。
struct Struct_Group_Up
{
    Struct_Motor_Base motor[kJoints];
};
#pragma pack(pop)

// 线格式没有长度字段也没有填充位，收发两侧尺寸必须严格一致。
// 这几条 static_assert 就是当初能挡住 alive_flag 漏拷的那种错。
static_assert(sizeof(Struct_Motor_Base) == 9, "Struct_Motor_Base 必须 9B（velocity+position+alive_flag）");
static_assert(sizeof(Struct_Chassis_Down) == 16, "底盘下行帧必须 16B");
static_assert(sizeof(Struct_Leg_Down) == 16, "腿下行帧必须 16B");
static_assert(sizeof(Struct_Group_Up) == 36, "上行帧必须 36B");

Struct_Chassis_Down Chassis_Down;
Struct_Leg_Down Leg_Down;
Struct_Group_Up Chassis_Up;
Struct_Group_Up Leg_Up;

// 测试数据：接真电机之前用来验证回传链路准不准。
// 两组各留一个「位精确回显」字段和一个「槽位标识常量」字段，上位机侧一眼就能看出
// 数值有没有落对帧、落对槽：
//   底盘 motor[k].velocity = 下行速度原样回显（逐位可对）
//   底盘 motor[k].position = 1000*(k+1) + 0.25 → 1000.25 / 2000.25 / 3000.25 / 4000.25
//   腿   motor[k].position = 下行位置原样回显（逐位可对）
//   腿   motor[k].velocity =  100*(k+1) + 0.50 →  100.50 /  200.50 /  300.50 /  400.50
// 底盘 position 填的是常量，上位机 mecanum 的里程计会跟着乱跳，属于测试数据的预期副作用；
// 想让里程计安静就把底盘那行 position 改成 0.0f。
// 腿的 slot 0 已经被真电机覆盖（见 Task_Loop），所以上面那行只对 k=1..3 成立。
constexpr float kTest_Chassis_Pos_Base = 1000.0f;
constexpr float kTest_Chassis_Pos_Offset = 0.25f;
constexpr float kTest_Leg_Vel_Base = 100.0f;
constexpr float kTest_Leg_Vel_Offset = 0.5f;

uint32_t Last_Control_Tick = 0;
uint32_t Last_Alive_Tick = 0;
bool Ready = false;

// 上位机存活：滑动窗口计数差分（对齐上位机的 Alive_Flag / Pre_Alive_Flag）
uint32_t Alive_Flag = 0;
uint32_t Pre_Alive_Flag = 0;
Enum_Alive Alive_Status = Alive_Status_DISABLE;

// 中间件的发送回调是裸函数指针，只能绑定无捕获函数（与上位机 SendCallback 等价）
int64_t SendCallback(uint8_t *data, size_t length)
{
    return USB_Transmit_Async(data, static_cast<uint16_t>(length)) == USB_Status::Ok
               ? static_cast<int64_t>(length)
               : -1;
}

// 一台 C610 先当「一条腿」用，只取反馈
Class_Motor_DJI_C610 Motor_DJI_C610_0;

void Device_FDCAN1_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{
    switch (FDCAN_RxMessage->Header.Identifier)
    {
        // M2006 + C610 电调反馈
        case kMotorIdC610:
        {
            Motor_DJI_C610_0.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }

        default:
            break;
    }
}

// 底层 USB 的接收回调，在 USB 中断里被调用，直接转发给中间件就地解包分发
void RxCallback(uint8_t *data, uint16_t length)
{
    // 滑动窗口, 判断上位机是否在线
    Alive_Flag += 1;

    USB_Communication_Interface.Rx_RptlCallback(data, length);
}

// 掉线时把下行目标值抹平，不留旧值（对齐上位机在判死分支里清 Rx 数据）
void Invalidate_Control_Data()
{
    Chassis_Down = {};
    Leg_Down = {};
}

/**
 * @brief 每 1000ms 检测一次上位机链路是否存活，由 Task_Loop 调用
 *
 */
void TIM_1000ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过上位机数据
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

} // namespace

void Task_Init()
{
    USB_Init(RxCallback);
    USB_Communication_Interface.Init(SendCallback);

    // 先把电机对象绑好再开 CAN 中断：反过来的话，电调上电就在 1kHz 推反馈，
    // FDCAN_Init 激活 RX 中断的瞬间就可能进回调，那时 Manage_Object 还是空指针。
    Motor_DJI_C610_0.Init(&hfdcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_ANGLE);
    // 串级两个环都只给 P：K_I、I_Out_Max 都留 0。注意 I_Out_Max 非 0 时 alg_pid 会除以 K_I，
    // K_I=0 就是除零 —— 要补积分时两个必须同时给。
    Motor_DJI_C610_0.PID_Angle.Init(kAngle_K_P, 0.0f, 0.0f, 0.0f, 0.0f, kAngle_Out_Max, 0.001f);
    Motor_DJI_C610_0.PID_Omega.Init(kOmega_K_P, 0.0f, 0.0f, 0.0f, 0.0f, kOmega_Out_Max, 0.001f);
    FDCAN_Init(&hfdcan1, Device_FDCAN1_Callback);

    // 两帧分别注册，尺寸一律 sizeof，不写魔数。
    // 与上位机的对应关系：
    //   id=1 底盘  tx 36B（4×Struct_Motor_Base）  rx 16B（4×float velocity）
    //   id=3 腿    tx 36B（4×Struct_Motor_Base）  rx 16B（4×float position）
    const bool ok_chassis = USB_Communication_Interface.Register(
        kIdChassis, &Chassis_Up, &Chassis_Down, sizeof(Chassis_Up), sizeof(Chassis_Down));
    const bool ok_leg = USB_Communication_Interface.Register(
        kIdLeg, &Leg_Up, &Leg_Down, sizeof(Leg_Up), sizeof(Leg_Down));

    // 不看短路，两个都试一遍
    Ready = ok_chassis && ok_leg;
    if (!Ready)
    {
        // 注册失败就常亮；心跳已删，正常跑 LED 一直不亮
        LED_ON();
    }
}

void Task_Loop()
{
    static uint32_t mod10 = 0;
    static uint32_t mod100 = 0;

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

    // 位置环：目标是上位机下发腿帧的 position[0]，单位就是输出轴角度 rad，跟电机自己报的
    // Now_Angle 是同一把尺子（都以本板上电时刻为零点、多圈累加），所以两边语义一致，
    // 上位机那边就算把实测位置填回来当指令，也只会变成"保持当前位置"，不会跑飞。
    // 断链时目标给 0，也就是回上电时的位置；而且不管断没断链这一趟都得跑，
    // 因为 C610 收不到 ~1kHz 控制帧就会报错，帧必须一直发。
    Motor_DJI_C610_0.Set_Target_Angle(
        (Alive_Status == Alive_Status_ENABLE) ? Leg_Down.position[0] : 0.0f);
    Motor_DJI_C610_0.TIM_Calculate_PeriodElapsedCallback();
    TIM_1ms_CAN_PeriodElapsedCallback();

    // 100ms 任务：C610 的接收掉线检测有自己的窗口，别跟着 10ms 的运算分频走
    if (++mod100 >= kMotorAlivePeriodMs / kControlPeriodMs)
    {
        mod100 = 0;
        Motor_DJI_C610_0.TIM_100ms_Alive_PeriodElapsedCallback();
    }

    // 10ms 任务
    if (++mod10 >= 10)
    {
        mod10 = 0;

        // 只有存活才准许运算：断线时回显整段不跑，Up 保持上一拍的旧值。
        // 抹平的是下行目标值，给 gate 之外任何读 Chassis_Down/Leg_Down 的代码兜底。
        for (size_t k = 0; k < kJoints; ++k)
        {
            // 底盘：速度原样回显（上位机可做位精确比对），位置给槽位标识常量
            Chassis_Up.motor[k].velocity = Chassis_Down.velocity[k];
            Chassis_Up.motor[k].position =
                kTest_Chassis_Pos_Base * static_cast<float>(k + 1) + kTest_Chassis_Pos_Offset;
            // alive_flag 现在还没有真电机可依，先写死在线；上位机侧目前也不读它
            Chassis_Up.motor[k].alive_flag = 1;

            // 腿：位置原样回显下行目标，速度给槽位标识常量
            Leg_Up.motor[k].position = Leg_Down.position[k];
            Leg_Up.motor[k].velocity =
                kTest_Leg_Vel_Base * static_cast<float>(k + 1) + kTest_Leg_Vel_Offset;
            Leg_Up.motor[k].alive_flag = 1;
        }

        // slot 0 换成真电机：C610 的角度/角速度直接覆盖上面那两个测试常量。
        // 其余 3 槽保持常量，所以上位机看到 3 个固定值 = 帧没错位，第 1 个值随
        // 手转电机而动 = 反馈真的通了。alive_flag 用电机自己的在线判定，
        // 不再是写死的 1：手拔 CAN 线后 100ms 内会翻成 0。
        Leg_Up.motor[0].velocity = Motor_DJI_C610_0.Get_Now_Omega();
        Leg_Up.motor[0].position = Motor_DJI_C610_0.Get_Now_Angle();
        Leg_Up.motor[0].alive_flag =
            (Motor_DJI_C610_0.Get_Status() == Motor_DJI_Status_ENABLE) ? 1 : 0;

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
