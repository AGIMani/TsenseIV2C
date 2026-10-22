#pragma once

#include "a3/A3Export.h"
#include "a3/Types.h"

namespace a3 {

/** A3.2 机构顺序的 20 个关节，与 a32 几何模型一致。 */
constexpr int kJointNum = 20;

/**
 * 关节角（度）→ 执行器指令。
 * 直线关节输出毫米，为相对 0° 的正行程。四条连杆：四指 MCP；食/中/无名指 PIP 与拇指 MCP（F2）；
 * 四指 DIP 与拇指 DIP（F1）；小拇指 PIP。角度限制在 0°–90°，超出端点按端点计算。0° 为 0 mm，90° 为该连杆行程上限。
 * 下发和反馈都用均匀线性查表。角度→行程的插值误差不超过 1e-9 m，行程→角度不超过 1e-6°。
 * 旋转关节输出度。角度非有限，或旋转关节超出机构范围时返回 false，out 不变。
 *
 * 顺序：拇指旋转块、拇指屈伸、拇指 MCP、拇指 PIP、
 * 食指 MCP/侧摆/PIP/DIP、中指 MCP/侧摆/PIP/DIP、
 * 无名指 MCP/侧摆/PIP/DIP、小拇指 MCP/侧摆/PIP/DIP。
 */
A3_API bool anglesDegToActuator(const float angle_deg[kJointNum], float out[kJointNum]) noexcept;

/**
 * 执行器反馈 → 关节角（度）。直线为 mm，旋转为 deg。
 * 电机反馈是 float32。限位附近允许 1e-5 mm（直线）或 1e-6 rad（旋转）的边界容差，
 * 容差内先钳回行程再反算；超出容差返回 false，angle_deg 不变。
 */
A3_API bool actuatorToAnglesDeg(const float actuator[kJointNum], float angle_deg[kJointNum]) noexcept;

/**
 * 把 20 路执行器指令填进 USB 自研槽和两台外置电机。
 * 直线为 mm，旋转为 deg。映射与 A3.2 左手槽表一致。
 */
A3_API bool actuatorToHand(const float actuator[kJointNum],
                           float self_out[kSelfMotorNum],
                           float mit_out[kMitMotorNum]) noexcept;

}  // namespace a3
