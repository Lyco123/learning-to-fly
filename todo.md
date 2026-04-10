# TODO
将原本的训练框架的输出修改为机体角速度和推力加速度

## 主要工作
- [ ] 阅读原始论文`2311.13081v2.pdf`，了解原始框架
- [ ] 阅读代码，了解该markdown中的主要基本知识
- [ ] 将后续的剩余任务完成，修改目前的强化学习仿真框架，actor的观测维度为位置、速度、姿态，critic的特权观测维度在actor的观测维度基础上添加三轴角速度指令和推力加速度指令，以及外扰力和外扰力矩。


## 主要基本知识

parameters/dynamics:这里主要是存储飞行器的model参数，保持不变即可
parameters/init：这里主要存放初始状态的分布参数，**需要修改**
parameters/reward_functions：奖励函数设计，**重点修改的内容**，主要和其参数有关
parameters/termination:终止条件，一般不用修改
metrics.h这个文件定义了评价指标，一般不用修改
multicopter.h：bring-up了整个仿真环境的配置，**可能需要修改**，包括：
- 创建多旋翼无人机的物理仿真模型
- 定义强化学习环境的状态空间和动作空间
- 配置不同的仿真参数和随机化策略
- 支持各种飞行任务的仿真需求
operation_gneric.h：用于定义仿真迭代的计算，**重点修改的部分**，因为需要额外添加控制器和混控

**奖励函数的设计**

```text
weighted_cost = position * position_cost 
              + orientation * orientation_cost 
              + linear_velocity * linear_vel_cost 
              + angular_velocity * angular_vel_cost 
              + linear_acceleration * linear_acc_cost 
              + angular_acceleration * angular_acc_cost 
              + action * action_cost
```

目前的奖励函数设计是没什么问题的，主要的问题是如果改为ctbr，那么其动作维度存在倾向性，即期望倾转角速度，期望偏航角速度和期望推力加速度的权重是不同的，需要分开给予单独的权重如weight_tilt_vel, weight_yaw_vel, weight_linear_acc


**初始条件的随机化**
主要修改`parameters/init`下的相关文件，控制输入变成了最大角速度指令的边界，以及最大推力加速度指令的边界，注意保留normalization的参数（等同于相对rpm参数）

**仿真环境的配置**
主要修改`operation_genric.h`，添加两个重要的文件，`rate_controller.h`和`mixer.h`，其中`rate_controller.h`用于将期望的角速度指令转换为力矩控制指令，`mixer.h`用于将力矩指令和推力加速度指令转换为归一化的电机输入指令，用于仿真的迭代，注意在parameter中保存设置角速度控制器和混控器的参数设置



## 注意的点
Crazyfile的机架配置从右上角为一号电机开始，顺时针方向分别为一号电机、二号电机、三号电机、四号电机，电机的旋转方向分别为逆时针、顺时针、逆时针和顺时针，因此在混控器的设计中予以注意。


整个环路为策略的输出的三轴期望角速度→角速度控制器→力矩指令→混控器→归一化的电机输入指令

飞行器的dynamic负责将四个电机的归一化指令通过油门曲线的参数值转换为电机转速，进而输出实际作用在刚体动力学上的力矩


增加单元测试环节，测试从角速度控制器环路到整个多旋翼飞行器的三轴动态响应输出是否符合预期


单元测试输出log