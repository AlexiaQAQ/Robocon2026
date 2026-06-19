# AGENTS - tof_1for4 工程：4组软件I2C + 4个VL53L1X + HEX协议输出

> **⚠️ 工作流铁律 (必须严格遵守)**
> 1. **分析→拆解任务→更新AGENTS清单→执行一步→更新AGENTS(划掉已完成)→下一步**，永远一步一步来
> 2. **每次做任何代码修改后，必须立即更新AGENTS**，不是汇报时才更新，而是每次改动都更新
> 3. **AGENTS更新 = 去除过时/错误/误导性信息 + 添加新的必要信息**，不是只追加
> 4. **永远相信AGENTS而非压缩后的上下文**：Trae上下文压缩可能产生错误总结，AGENTS记录真实状态
> 5. **AGENTS必须包含**：当前在做什么、刚刚做了什么、目标是什么、下一步做什么、测试数据位置
> 6. **被任何形式中断后，接手的AI靠读AGENTS即可无缝继续**，这是AGENTS存在的核心意义
> 7. **禁止跳步执行**：未完成当前步骤就擅自开始下一步 = 必然导致代码混乱

**工程路径**: `C:\Users\admin\Desktop\tof\tof_1for4\LED\MDK-ARM\tof_1for4.uvprojx`
**芯片**: STM32F103C8T6 (Cortex-M3, 64KB Flash, 20KB RAM)
**框架**: HAL库 + FreeRTOS V10.3.1 + CMSIS-RTOS V2

---

## 一、目标

4组软件I2C读取4个VL53L1X距离 + USART1输出HEX协议帧 (10字节大端序)

---

## 二、4组软件I2C引脚映射

| 通道 | SDA | SCL | 对应ToF |
|------|-----|-----|---------|
| CH1 | PB10 | PB11 | front_front |
| CH2 | PB0 | PB1 | front_back |
| CH3 | PA6 | PA7 | back_front |
| CH4 | PA4 | PA5 | back_back |

---

## 三、核心问题: VL53L1_StartMeasurement返回-4 (VL53L1_ERROR_CONTROL_INTERFACE)

### 现象
- 4个ToF全部: WaitBoot=0, DataInit=0, StaticInit=0, SetDistMode=0, TimingBudget=0, InterPeriod=0, **StartMeas=-4**
- 所有初始化步骤都成功，只有最后一步失败

### 关键事实
- **VL53L1_StartMeasurement内部只调用一次VL53L1_WriteMulti（写~135字节到寄存器0x0001）**, 不涉及读操作！
- 所以读操作的Repeated Start修复对StartMeasurement本身无效
- 但时钟拉伸(SCL_WAIT_HIGH)对135字节长序列写入有效

### 已尝试但未解决的修复
1. ~~I2C软复位(9脉冲+Stop+100ms)~~ — 无效
2. ~~InterPeriod从20ms改为30ms~~ — 无效
3. ~~距离模式改为LONG~~ — 无效
4. ~~Repeated Start(读操作改用SW_I2C_Read)~~ — 文件曾被还原未生效，本次重写确认生效

### 当前修改 (2026-06-18 14:52 编译通过)
| 文件 | 修改内容 | 对StartMeasurement的作用 |
|------|---------|------------------------|
| `vl53l1_platform.c` | 重写: 所有读操作用SW_I2C_Read(Repeated Start), 删除旧_I2CRead, 加调试追踪[WM][RM][RB][RW][RDW][I2C#] | 间接: 确保DataInit等步骤读到正确数据 |
| `sw_i2c.c` | SCL_WAIT_HIGH时钟拉伸支持(所有SCL_HIGH替换) | **直接**: 135字节写入时等待从机SCL释放 |

### 当前任务清单

- [x] ~~**Step 1**: 重写`vl53l1_platform.c` - 读操作SW_I2C_Read + 调试追踪~~ ✅
- [x] ~~**Step 2**: `sw_i2c.c`添加SCL_WAIT_HIGH时钟拉伸~~ ✅
- [x] ~~**Step 3**: 编译验证~~ ✅ 0错误0警告, Flash=30500B, RAM=12368B
- [ ] **Step 4**: 烧录 + 串口分析数据
- [ ] **Step 5**: 根据串口调试信息定位具体失败点并修复
- [ ] **Step 6**: 去掉调试输出，只保留HEX协议帧
- [ ] **Step 7**: 优化读取频率接近30Hz

---

## 四、当前执行状态

**正在执行**: Step 4 — 烧录 + 串口分析数据
**刚刚完成**: Step 1-3 (platform.c重写+时钟拉伸+编译通过)
**固件**: `C:\Users\admin\Desktop\tof\tof_1for4\LED\MDK-ARM\tof_1for4\tof_1for4.hex`
**下一步**: 用户烧录后提供串口输出

### 期望的串口输出解读
- `[WM] FAIL idx=0x0001 len=135` → WriteMulti(StartMeasurement的核心135字节写入)失败
- `[I2C#N] Wr FAIL len=137` → 第N次I2C写操作失败(137=地址2字节+数据135字节)
- `[RB] FAIL idx=XXXX` → RdByte失败(某寄存器读取失败)
- 如果`StartMeas=0` → 修复成功!
- 如果仍为`-4`但有`[WM]`或`[I2C#]`消息 → 可精确定位失败操作

---

## 五、经验教训

- **VL53L1_StartMeasurement只调用WriteMulti(写135字节)**, 不涉及读操作! 修读操作不能直接解决此问题
- **软件I2C必须处理时钟拉伸(Clock Stretching)** — VL53L1X可能拉低SCL, 主控必须等待
- **Edit工具可能不生效!** 之前多次Edit操作文件被还原, 必须用Write工具重写整个文件确认修改持久化
- TOF传感器需要2000ms初始化延迟
- 4个VL53L1X共用I2C地址0x29, 每个必须在独立I2C总线上
- 参考工程使用硬件I2C1 + XSHUT硬件复位(PB4) + LONG模式 + InterPeriod 25ms
