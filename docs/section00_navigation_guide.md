# Section00 导航任务：实现指南

> 基于以下三项资产，构建 `vbot_navigation_section00` 环境。
> - `anymal_c` 导航代码（`anymal_c_np.py` + `cfg.py` + `xmls/scene.xml`）
> - `vbot` 机器人资产（`vbot.xml` + 各关节/传感器定义）
> - `phase1_world_0126_clean` 地图（`section01/0126_C_section01.xml` 碰撞圆柱平台）

---

## 已有资产速览

### anymal_c 导航代码
| 文件 | 内容 |
|------|------|
| `anymal_c/xmls/scene.xml` | 场景 XML：机器人 + 无限平面地形 |
| `anymal_c/cfg.py` | `AnymalCEnvCfg`，含 Asset/Sensor/InitState/Commands |
| `anymal_c/anymal_c_np.py` | 完整 env：54 维观测、位置控制、单目标 reset |

anymal_c 的 XML 结构：
```xml
<mujoco model="anymal_c scene">
  <include file="anymal_c.xml" />        <!-- 机器人 -->
  <worldbody>
    <geom name="ground" type="plane" />  <!-- 无限平面 -->
  </worldbody>
</mujoco>
```

### vbot 机器人资产
- `vbot.xml`：12 个腿部关节（`FR/FL/RR/RL_hip/thigh/calf_joint`），执行器为 `motor` 类型
- **内置可视化辅助 body**（anymal_c 无等价物）：
  - `target_marker`：3 个真实 joint（slide/slide/hinge），占 **DOF 0–2**
  - `robot_heading_arrow` / `desired_heading_arrow`：各一个 freejoint，占 **DOF 22–35**
- 传感器名：`base_linvel`、`base_gyro`、`FR/FL/RR/RL_foot_contact`（脚部接触力）

> **与 anymal_c 关键差异**：anymal_c 的 `target_marker` 和箭头是 `mocap="true"` body，不消耗 DOF；  
> vbot 用真实 joint，导致 base freejoint 从 **DOF 3** 开始，需要显式处理偏移。

### phase1_world_0126_clean 地图
```
phase1_world_0126_clean/
├── section01/
│   ├── 0126_C_section01.xml   ← 碰撞体：半径 25m、高 2m 圆柱平台
│   └── 0126_CP_section01.xml  ← 得分区域标记（视觉用）
├── world.xml                  ← 组合场景（含碰撞体 + 地基）
└── world_CP.xml               ← 组合场景（含得分区域）
```

`0126_C_section01.xml` 核心碰撞体定义：
```xml
<body name="B_PingTai">
  <geom type="cylinder" size="25.0 2.0"   <!-- 半径 25m，高 2m -->
        pos="0 0 0" quat="1 0 0 0" />
</body>
```
地形顶面高度 = `0 + 2.0 = 2.0m`（geom 中心在 Z=0，半高 2.0m，顶面在 Z=2.0m）。  
机器人站在顶面，初始 Z ≈ `2.0 + 0.5（机身离地） ≈ 2.5m`（后经调试设为 1.41m）。

---

## 步骤一：构建场景 XML

**目标文件**：`navigation/vbot/xmls/scene_section001.xml`

### 1.1 对比 anymal_c scene.xml 的差异

| 属性 | anymal_c | vbot section00 | 改动原因 |
|------|----------|----------------|----------|
| 机器人 | `anymal_c.xml` | `vbot.xml` | 换机器人 |
| 地形 | `<geom type="plane">` | `<attach model="C_...">` | 换地形 |
| `<compiler>` | 无 | `meshdir="meshes/"` | 地形有 `.obj` 网格文件 |
| `<statistic>` | `extent="1.2"` | `extent="12"` | 场景半径 25m |
| 传感器 | 无（用 geom 碰撞查询） | `<sensor><contact .../>` | 平台无独立 geom 名，改用 subtree 传感器 |

### 1.2 地形组合机制

地图文件（`0126_C_section01.xml`）与机器人文件（`vbot.xml`）通过 `<model>` + `<attach>` 机制合并进同一场景：

```xml
<asset>
  <!-- 第一步：预加载模型文件并命名 -->
  <model name="section01C" file="0126_C_section01.xml"/>
</asset>
<worldbody>
  <!-- 第二步：实例化到世界，加 prefix 防止命名冲突 -->
  <attach model="section01C" prefix="C_"/>
</worldbody>
```

`prefix="C_"` 的作用：地形内所有 body/geom 名加前缀（`B_PingTai` → `C_B_PingTai`），避免与机器人命名冲突，同时让传感器能精确引用碰撞体。

### 1.3 最终 scene_section001.xml

```xml
<?xml version="1.0" ?>
<mujoco model="vbot navigation - section01">
  <!-- 地形 .obj 网格需要 meshdir，提升到 scene 层统一声明 -->
  <compiler angle="radian" meshdir="meshes/" autolimits="true"/>

  <!-- 引入 vbot 机器人（含 target_marker、箭头、传感器，全部内置） -->
  <include file="vbot.xml" />

  <!-- 调大 extent 以适应 25m 半径场景 -->
  <statistic center="0 0 0.1" extent="12" meansize="0.04" />

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0" />
    <rgba haze="0.15 0.25 0.35 1" />
    <global azimuth="120" elevation="-20" />
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
             width="512" height="3072"/>
    <!-- 加载碰撞地形模型 -->
    <model name="section01C" file="0126_C_section01.xml"/>
  </asset>

  <worldbody>
    <light pos="0 0 10" dir="0 0 -1" directional="true" />
    <!-- attach 地形：prefix="C_" 使所有 body/geom 名变为 C_B_PingTai 等 -->
    <attach model="section01C" prefix="C_"/>
  </worldbody>

  <sensor>
    <!-- 足部与地形平台的接触力（subtree 方式，适用于无独立 geom 名的地形） -->
    <contact name="FR_foot_contact" subtree1="FR_foot" subtree2="C_B_PingTai" data="normal" num="1"/>
    <contact name="FL_foot_contact" subtree1="FL_foot" subtree2="C_B_PingTai" data="normal" num="1"/>
    <contact name="RR_foot_contact" subtree1="RR_foot" subtree2="C_B_PingTai" data="normal" num="1"/>
    <contact name="RL_foot_contact" subtree1="RL_foot" subtree2="C_B_PingTai" data="normal" num="1"/>
    <!-- 基座与地形的碰撞检测（用于终止条件） -->
    <contact name="base_contact" subtree1="base" subtree2="C_B_PingTai" data="normal" num="1"/>
  </sensor>
</mujoco>
```

> **为什么不用 anymal_c 的 `get_geom_index("ground")` 方式？**  
> `0126_C_section01.xml` 的碰撞体在 `<default class="C_Cylinder">` 中定义，没有独立的 geom name。  
> attach 后前缀为 `C_`，但 class 碰撞体无法通过 `get_geom_index` 检索。  
> 改为 XML 传感器 + `get_sensor_value("base_contact")` 更可靠。

---

## 步骤二：适配 cfg.py

**目标**：基于 `AnymalCEnvCfg` 创建 `VBotSection00EnvCfg`。

### 2.1 差异对比

| 配置项 | anymal_c | vbot section00 | 原因 |
|--------|----------|----------------|------|
| `model_file` | `anymal_c/xmls/scene.xml` | `vbot/xmls/scene_section001.xml` | 场景文件替换 |
| `body_name` | `"base"` | `"base"` | 相同 |
| `foot_names` | `["LF_FOOT","RF_FOOT",...]` | `["FR","FL","RR","RL"]` | vbot 足部 geom 名不同 |
| `terminate_after_contacts_on` | `["base"]` | `["collision_middle_box","collision_head_box"]` | vbot 机身碰撞体名不同 |
| `ground_name` | `"ground"`（geom 直接查询） | 废弃，改为 `ground_subtree="C_B_PingTai"` | 地形无独立 geom name |
| `base_linvel` 传感器 | `"base_linvel"` | `"base_linvel"` | 相同 |
| `base_gyro` 传感器 | `"base_gyro"` | `"base_gyro"` | 相同 |
| `action_scale` | `0.06` | `0.25` | vbot 手动 PD，需要更大动作范围 |
| `pos` | `[0,0,0.5]` | `[0,-11.5,1.41]` | 起点在圆台边缘 |
| `max_episode_steps` | `700` | `4000` | 圆台半径 25m，需要更长时间 |

### 2.2 关键配置片段

```python
@registry.envcfg("vbot_navigation_section00")
@dataclass
class VBotSection00EnvCfg(VBotStairsEnvCfg):
    model_file: str = os.path.dirname(__file__) + "/xmls/scene_section001.xml"
    max_episode_seconds: float = 40.0
    max_episode_steps: int = 4000

    @dataclass
    class InitState:
        pos = [0.0, -11.5, 1.41]            # 起点：圆台边缘 Y=-11.5m
        pos_randomization_range = [-0.5, -0.5, 0.5, 0.5]
        default_joint_angles = { ... }       # vbot 关节名：FR_hip_joint 等

    @dataclass
    class ControlConfig:
        action_scale = 0.25                  # 手动 PD 需要更大范围

    init_state: InitState = field(default_factory=InitState)
    control_config: ControlConfig = field(default_factory=ControlConfig)
```

---

## 步骤三：改写 env Python 文件

**目标文件**：`navigation/vbot/vbot_section00_np.py`  
**起点**：复制 `anymal_c_np.py` 后逐项改写。

---

### 改动 A：`__init__` — DOF 偏移处理

**问题根源**：

| | anymal_c.xml | vbot.xml |
|--|--|--|
| `target_marker` | `mocap="true"`，**不占 DOF** | 3 个真实 joint（slide×2 + hinge），占 **DOF 0–2** |
| 方向箭头 | `mocap="true"`，不占 DOF | 2 × freejoint，占 **DOF 22–35** |
| **base freejoint 起点** | **DOF 0** | **DOF 3** |

anymal_c 的 reset 直接操作 `dof_pos[:, 0:3]`（base XYZ），vbot 必须操作 `dof_pos[:, 3:6]`。

```python
def _find_target_marker_dof_indices(self):
    """target_marker 占据 DOF 0-2，base freejoint 从 DOF 3 开始"""
    self._target_marker_dof_start = 0
    self._target_marker_dof_end = 3
    self._base_quat_start = 6    # base 四元数：DOF 6–9
    self._base_quat_end = 10

def _find_arrow_dof_indices(self):
    """robot_heading_arrow: DOF 22–28，desired_heading_arrow: DOF 29–35"""
    self._robot_arrow_dof_start = 22
    self._robot_arrow_dof_end = 29
    self._desired_arrow_dof_start = 29
    self._desired_arrow_dof_end = 36
```

**途径点系统**（anymal_c 无此机制）：

```python
# anymal_c：每次 reset 生成随机单目标点
# vbot section00：固定途径点列表，课程学习逐步解锁
self.waypoints = [
    np.array([0.0, 0.0, 1.41], dtype=np.float32),  # 终点：圆台中心
]
# 每个环境独立记录历史最远到达点（课程学习状态）
self.env_max_reached_waypoint = np.zeros(self._num_envs, dtype=np.int32)
```

---

### 改动 B：`_init_contact_geometry` — 地形 geom 获取方式

```python
# anymal_c 原写法（适用于命名平面地形）：
self.ground_index = self._model.get_geom_index("ground")
self.termination_contact = np.array([[base_idx, self.ground_index]], dtype=np.uint32)

# vbot section00 改为（地形无独立 geom 名，改用 XML sensor）：
# 足部接触力：直接读传感器
self.foot_sensor_names = ["FR_foot_contact", "FL_foot_contact",
                           "RR_foot_contact", "RL_foot_contact"]
# 基座碰撞终止：读 base_contact 传感器
self.base_contact_sensor = "base_contact"
```

---

### 改动 C：`apply_action` — 控制模式

```python
# anymal_c：引擎内置 PD（XML 里配了 kp/kv，直接给目标角度）
actions_scaled = actions * 0.06
state.data.actuator_ctrls = self.default_angles + actions_scaled

# vbot section00：Python 层手动 PD 力矩（XML 里 kp/kv 未设置）
def _compute_torques(self, actions):
    target_pos = self.default_angles + actions * self._cfg.control_config.action_scale
    current_pos = self.get_dof_pos(self._state.data)  # 当前关节角
    current_vel = self.get_dof_vel(self._state.data)  # 当前关节速度
    kp, kv = 80.0, 6.0
    torques = kp * (target_pos - current_pos) - kv * current_vel
    # 按关节类型 clip：髋±17 N·m，大腿±17 N·m，小腿±34 N·m
    return np.clip(torques, -self.torque_limits, self.torque_limits)
```

---

### 改动 D：`update_state` — 观测维度扩展（54 → 67 维）

anymal_c 54 维结构不变，vbot section00 新增 **13 维**：

| 新增 | 维度 | 来源 | 说明 |
|------|------|------|------|
| `contact_force` | **+12** | `get_sensor_value("FR/FL/RR/RL_foot_contact")` | 四足接触力（XYZ 各 1） |
| `distance_improvement_normalized` | **+1** | `(prev_dist - cur_dist) / max_dist` | 与接近奖励同步 |

```python
# 读足部接触力（12 维）
foot_forces = []
for name in self.foot_sensor_names:
    f = self._model.get_sensor_value(name, data)  # [num_envs, 1]
    foot_forces.append(f)
contact_force = np.concatenate(foot_forces, axis=1)  # [num_envs, 4] → 归一化后接入obs

obs = np.concatenate([
    noisy_linvel,           # 3
    noisy_gyro,             # 3
    projected_gravity,      # 3
    noisy_joint_angle,      # 12
    noisy_joint_vel,        # 12
    last_actions,           # 12
    command_normalized,     # 3
    position_error_norm,    # 2
    heading_error_norm,     # 1
    distance_norm,          # 1
    reached_flag,           # 1
    stop_ready_flag,        # 1
    contact_force_norm,     # 12  ← 新增
    dist_improvement_norm,  # 1   ← 新增
], axis=-1)
# 总计 67 维
```

---

### 改动 E：`_compute_reward` — 完整奖励体系

anymal_c 只有 3 项奖励；vbot section00 扩展为完整体系：

| 类别 | 奖励项 | 说明 |
|------|--------|------|
| **导航核心** | `velocity_tracking_lin` (+1.5) | 线速度跟踪 |
| | `velocity_tracking_ang` (+0.3) | 角速度跟踪 |
| | `approach_reward` (±1.0) | 每靠近 1m 奖励，远离惩罚 |
| | `arrival_bonus` (+10.0) | 首次到达一次性奖励 |
| | `stop_bonus` (浮动) | 到达后停稳奖励 |
| **步态质量** | `feet_air_time` (+) | 正确步态节律 |
| | `feet_stumble` (-) | 惩罚足部绊跌 |
| **稳定性惩罚** | `lin_vel_z` (-2.0) | 垂直速度惩罚 |
| | `ang_vel_xy` (-0.05) | 俯仰/侧倾角速度 |
| | `torques` (-1e-5) | 力矩惩罚 |
| | `action_rate` (-0.001) | 动作平滑惩罚 |
| **停滞惩罚** | 连续 N 步未进展 | -2 → -5 → -10（阶梯式） |
| **终止惩罚** | 各终止条件 | -100 各项 |

---

### 改动 F：`reset` — 课程学习途径点系统

```python
# anymal_c：随机位置，随机目标偏移，每次独立
robot_init_pos = random(pos_range)
target = robot_init_pos + random(offset_range)

# vbot section00：课程学习
# 1. 读取该环境历史最远途径点
waypoint_idx = self.env_max_reached_waypoint[env_idx]

# 2. 目标固定为当前课程的途径点（不随机）
target = self.waypoints[waypoint_idx]

# 3. base 位置写入 DOF 3:6（而非 anymal_c 的 0:3）
dof_pos[:, 3:6] = robot_init_xyz
```

---

### 改动 G：`_compute_terminated` — 终止条件从 1 项 → 4 项

| 条件 | anymal_c | vbot section00 | 检测方式 |
|------|----------|----------------|----------|
| 基座碰地 | ✅ contact_query | ✅ `get_sensor_value("base_contact")` | XML sensor |
| 关节碰地 | ❌ | ✅ | contact_query |
| XY 速度 > 1e8 | ❌ | ✅ 防数值爆炸 | 数值检查 |
| 侧翻 > 60° | ✅ 75° | ✅ 60°（更严格） | 投影重力向量 |

---

## 步骤四：注册 RL 配置

在 `motrix_rl/src/motrix_rl/cfgs.py` 的 `navigation` 类中添加：

```python
@rlcfg("vbot_navigation_section00")
@dataclass
class VBotNavigationSection00PPOConfig(PPOCfg):
    seed: int = 42
    num_envs: int = 4096
    play_num_envs: int = 1
    max_env_steps: int = 1024 * 60_000
    learning_rate: float = 3e-4
    rollouts: int = 48
    learning_epochs: int = 6
    mini_batches: int = 32
    discount_factor: float = 0.99
    policy_hidden_layer_sizes: tuple[int, ...] = (512, 256, 128)
    value_hidden_layer_sizes: tuple[int, ...] = (512, 256, 128)
```

---

## 步骤五：注册环境

在 `navigation/vbot/__init__.py` 中添加导入：

```python
from .vbot_section00_np import VBotSection00Env
```

并确保 `navigation/__init__.py` 存在（空文件即可），否则 Python 无法识别 `navigation` 为包。

---

## 步骤六：训练与验证

```bash
cd MotrixLab-main

# 查看场景（验证 XML 加载正确）
uv run python scripts/view.py --env vbot_navigation_section00

# 开始训练
uv run python scripts/train.py --env vbot_navigation_section00

# 推理测试
uv run python scripts/play.py --env vbot_navigation_section00
```

---

## 关键改动总览

```
vbot_section00_np.py
    │
    ├─ [XML]    scene.xml → scene_section001.xml
    │             • anymal_c.xml → vbot.xml
    │             • <geom type="plane"> → <attach model="C_" section01C>
    │             • 新增 <sensor><contact subtree.../> × 5
    │
    ├─ [CFG]    AnymalCEnvCfg → VBotSection00EnvCfg
    │             • foot_names: LF_FOOT → FR（vbot 命名）
    │             • terminate_on: base → collision_middle/head_box
    │             • ground_name 废弃 → ground_subtree（subtree 方式）
    │             • action_scale: 0.06 → 0.25（手动 PD）
    │             • pos: [0,0,0.5] → [0,-11.5,1.41]（圆台边缘）
    │
    └─ [ENV]    anymal_c_np.py → vbot_section00_np.py
                  • __init__：DOF 偏移（base 从 DOF 3）+ 途径点系统
                  • _init_contact_geometry：geom 查询 → subtree sensor
                  • apply_action：位置控制 → 手动 PD 力矩
                  • update_state：54 维 → 67 维（+12 接触力，+1 距离改善）
                  • _compute_reward：3 项 → 完整奖励体系
                  • reset：随机单目标 → 课程学习（改动 F）
                  • _compute_terminated：1 项 → 4 项终止条件（改动 G）
```
