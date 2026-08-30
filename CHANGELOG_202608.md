# as_navigation 控制改动说明（2026-08）

> 本文档记录 2026-08 期间对 as_navigation 包的核心控制改动、新增接口与参数配置，
> 供版本管理与后续调试参考。

---

## 1. navigationSuper 接口重写（src/api_3d.cpp）

### 改动内容
接口整体重写为简洁的跟踪控制器，删除历史迭代中积累的冗余逻辑：

**删除项**
- 加速度前馈项（`acc_ff * a_ref`）——加速段会制造超前/过冲
- 换点新鲜度判定（`fresh_cmd`）——换点后丢弃旧轨迹导致飞机静止等待、前段起步慢
- 帧间斜率限幅窗口（`super_slew_timer_`）
- yaw 收尾逻辑（机头固定 0°，该分支死代码）
- 到点位置保持（rviz 模式专用，实机死代码）
- Z 轴速度 PID（Z 已改位置控制）

**保留/新增**
- 控制律：`v_cmd = kp*err_p + kv*err_v + ki*∫err_p + v_ref`（速度外环 + 参考速度前馈）
- 超时空窗兜底：位置锁存（holdfix，防零速漂移；实测零速空窗会漂 0.4m+）
- 轨迹完成判定（`trajectory_flag == TRAJECTORY_STATUS_COMPLETED` 时保持）
- 到达判定：2D 距离 + 0.15s debounce
- **新增第 6 参数 `stop_at_goal`**：末点到达后切位置锁存，等 v<0.3 再返回 true（供 autoLand 前使用）

### 关键行为
- **动着换点**：换点后旧轨迹尾巴继续跟踪（超时兜底），新轨迹到达无缝切换，前段不再有
  "静止等轨迹"的起步空窗
- **超时兜底**：断联/无轨迹时位置锁存当前点（PX4 位置环），不漂移

## 2. 新增 putShootPlus 接口（A/B 靶点动态选择）

### 用途
raicom 仿真场景打击 A/B 靶任务。原 `putShootSimple` 打靶点 y 值硬编码
（-1.7 或 -2.7 需手动改代码）；`putShootPlus` 根据阶段 0 YOLO 识别的字母自动选择：

```
识别到 "A" → 打靶点 y = -1.7
识别到 "B" → 打靶点 y = -2.7
未识别到  → 默认 y = -1.7（A）
```

### 与 putShootSimple 的差异
- 阶段 2/3 打靶点由 `target_letter` 动态决定（原硬编码）
- 阶段 0 未识别到字母时清空 `target_letter`（防上次残留打错靶）
- 阶段 6 结束时清空字母状态（支持接口再次调用）
- 调用签名、阶段流程与 putShootSimple 完全一致

## 3. competition_3d.cpp 状态机调整

- case 3 使用 `putShootSimple`（当前为硬编码靶点；如需 A/B 动态切换调用 `putShootPlus`）
- case 4/5/6 为 navigationSuper 连续航点
- case 6（末点）传 `stop_at_goal=true`，到达后位置锁存收敛再进 autoLand

## 4. 参数配置

### 4.1 asnav_3d.launch（控制器）
```
super_kp_outer   = 2.6   位置误差→速度修正
super_kv_outer   = 2.2   速度误差阻尼
super_ki_outer   = 0.15  位置误差积分
super_max_vel    = 1.8   XY 速度限幅
super_max_vel_z  = 1.0   Z 速度限幅
super_traj_timeout = 0.5  SUPER 断联超时→位置锁存
super_acc_ff     = 0     加速度前馈（已关闭）
```

### 4.2 click_smooth_ros1.yaml（SUPER 规划器）
```
traj_opt.boundary.max_vel = 1.8   轨迹速度上限
traj_opt.boundary.max_acc = 1.8   轨迹加速度上限
super_planner.corridor_line_max_length = 1.2   走廊长度（0.6 过短导致障碍区走廊生成失败）
super_planner.corridor_bound_dis       = 0.6
super_planner.planning_horizon         = 3.5
super_planner.safe_corridor_line_max_length = 5.0
rog_map.inflation_step   = 3   膨胀 0.3m（≥ robot_r 0.3m）
rog_map.inflation_resolution = 0.1
```

### 4.3 PX4 参数（QGC/参数界面）
```
MPC_XY_VEL_P_ACC = 3.6   速度环 P 增益（速度误差→加速度）
MPC_XY_VEL_I_ACC = 2.2   速度环 I 增益
MPC_XY_VEL_D_ACC = 0.35  速度环 D 增益
MPC_XY_P         = 0.95  位置环 P 增益
MPC_ACC_HOR      = 3.0   水平加速度上限
MPC_JERK_MAX     = 8.0   加加速度上限
MC_ROLL_P / MC_PITCH_P = 7.0   姿态环 P
MC_ROLLRATE_P / MC_PITCHRATE_P = 0.15   角速度环 P
```

### 调参依据（简）
- `MPC_XY_VEL_P_ACC` 从 1.8 → 3.2 → 3.6：速度环带宽不足导致跟踪滞后 0.5~0.9 m/s
  （PX4 ulog 实测），提高到 3.6 后贴合显著改善且无振荡（角速度 std <0.6°/s）
- `corridor_line_max_length` 0.6 → 1.2：0.6 过短，障碍密集区相邻走廊重叠不足，
  走廊生成持续失败→轨迹断供→滑入 backup 刹停→贴障碍
- `inflation_step` 2 → 3：膨胀 0.2m < 桨保 0.3m，轨迹/刹停点贴障碍

---

## 5. 版本记录
- 2026-08-25：navigationSuper 重写（动换点 + 位置锁存兜底）、putShootPlus 新增、
  competition_3d 调整、launch 参数对齐
- 2026-08-26：A/B 靶点动态逻辑（putShootPlus）、PX4 速度环调参记录
