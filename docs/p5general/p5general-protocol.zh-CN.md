# P5 General（PS5 认证加密狗）USB 协议规范

本文档由 GP2040-CE 固件源码逆向推导而来，描述 P5 General（下称"P5 General 加密狗"或"加密狗"）在 USB 协议层面的完整行为：设备枚举形态、认证握手（challenge–response）、运行时数据流与时序。目标是使读者能在任意平台（例如 Linux + raw_gadget / 独立 USB 主机栈）上从零实现一个与 P5 General 对接的驱动。

所有结论均标注来源 `文件:行号`（路径相对于 GP2040-CE 仓库根目录）。标注 **[代码]** 的条目为代码直接证据；标注 **[推断]** 的条目为从代码结构推出的合理结论，均已显式标明。代码中无法确定的事项集中列于第 10 节，本文档不对它们做任何猜测性描述。

---

## 目录

1. [系统拓扑](#1-系统拓扑)
2. [枚举阶段：呈现给 PS5 的 USB 设备形态](#2-枚举阶段呈现给-ps5-的-usb-设备形态)
3. [HID 报告总览与报告描述符解析](#3-hid-报告总览与报告描述符解析)
4. [输入报告 0x01 逐字节格式](#4-输入报告-0x01-逐字节格式)
5. [认证握手（控制传输）](#5-认证握手控制传输)
6. [运行时数据流：输入报告的签名流水线](#6-运行时数据流输入报告的签名流水线)
7. [加密狗侧对接契约（主机实现指南）](#7-加密狗侧对接契约主机实现指南)
8. [DualSense 主机输入路径](#8-dualsense-主机输入路径)
9. [时序、重试与失败路径汇总](#9-时序重试与失败路径汇总)
10. [未能从代码确定的事项](#10-未能从代码确定的事项)
11. [配置语义与用户可见行为](#11-配置语义与用户可见行为)
12. [实现核对清单](#12-实现核对清单)
13. [源码依据清单](#13-源码依据清单)

**一句话概述**：GP2040-CE 固件在 P5GENERAL 输入模式下，向 PS5 冒充一枚 P5 General 加密狗形态的 HID 手柄（VID 0x2B81 / PID 0x0101），同时用板载 PIO USB 主机口驱动一枚真实的 P5 General 加密狗，把 PS5 下发的认证质询原样转发给加密狗、把加密狗的应答原样回给 PS5，并且每一个发往 PS5 的 64 字节输入报告都必须先送到加密狗、再由加密狗返回（其末尾 8 字节 `hash` 字段由加密狗填写）后才能发出。

---

## 1. 系统拓扑

```
                 USB 总线 A（RP2040 硬件 USB，device 角色，full speed）
  PS5 主机  ◄──────────────────────────────────────────────►  GP2040 板
  (USB host)        枚举到 "P5General" HID 手柄              (USB device,
             VID 0x2B81 / PID 0x0101                          core0 运行 TinyUSB device 栈)
             IN EP 0x82 (中断, 64B, 1ms)
             OUT EP 0x01 (中断, 64B, 6ms)
             控制端点: 特征报告 0x03/0xF0/0xF1/0xF2
                                                                │
                                                                │ 按键/摇杆状态 (板载 GPIO/ADC)
                                                                ▼
                                                     P5GeneralDriver::process()
                                                     组装 64B P5GenerorReport
                                                                │
                 USB 总线 B（PIO USB，host 角色，full speed）    │
  P5 General 加密狗 ◄───────────────────────────────────────────┘
  (USB device,      枚举匹配 VID 0x2B81 / PID 0x0101
   插在 GP2040       IN EP: 加密狗 → GP2040（持续轮询, 64B 中断报告）
   的 PIO 口上)      OUT EP: GP2040 → 加密狗（64B 输入报告状态数据）
                    控制端点: SET_REPORT(F0) / GET_REPORT(F1) / GET_REPORT(F2)
```

拓扑结论（均为 **[代码]** 证据）：

- GP2040 板同时扮演两个 USB 角色：对 PS5 是 **USB device**（`src/gp2040.cpp:255` `tud_init(TUD_OPT_RHPORT)` 启动 TinyUSB 设备栈；设备描述符由 `P5GeneralDriver::get_descriptor_device_cb` 提供，`src/drivers/p5general/P5GeneralDriver.cpp:319-322`）；对加密狗是 **USB host**（`src/gp2040.cpp:258` → `USBHostManager::start()`，`src/usbhostmanager.cpp:14-25` 用 `tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, ...)` + `tuh_init(BOARD_TUH_RHPORT)` 在 PIO USB 口上启动 TinyUSB 主机栈，`headers/tusb_config.h:46` 定义 `BOARD_TUH_RHPORT=1`）。
- 加密狗在主机口上的识别条件只有一个：`VID == 0x2B81 && PID == 0x0101`（`src/drivers/p5general/P5GeneralAuthUSBListener.cpp:91-99`）。任何枚举出该 VID/PID 的 HID 接口实例即被认定为加密狗（第一个匹配者生效，`dongle_ready = true`）。
- PS5 看到的设备与加密狗具有**相同的 VID/PID**（设备描述符字节 `0x81, 0x2B` / `0x01, 0x01`，小端即 0x2B81/0x0101，`headers/drivers/p5general/P5GeneralDescriptors.h:133-134`）。**[推断]** 因此 GP2040 的设备侧是加密狗对主机侧接口的功能克隆：PS5 把 GP2040 当成直接插上的 P5 General 设备，而真实的签名运算全部发生在插在主机口上的加密狗内。GP2040 源码中 `#define P5GENERAL_VENDOR_ID 0x28B1`（`P5GeneralDescriptors.h:14`）与描述符字节不一致（描述符实际编码 0x2B81，且主机侧匹配也用 0x2B81），该宏未被任何匹配逻辑引用，属源码层面的笔误；线上真值为 0x2B81。
- 真 DualSense（索尼 054c/0CE6）不在 P5 认证链路中。`DualsensePS5Host` 是通用"手柄直插主机口"插件（`src/addons/gamepad_usb_host_listener.cpp:49-50`）的一个分支，与认证监听器并存、各自认领不同 VID/PID（认证监听器只认 0x2B81/0x0101，DualsensePS5Host 只认 0x054c/0x0CE6，`src/hosts/DualsensePS5Host.cpp:3-13`）。PIO 主机口一次接一台设备；两台并存需要外部集线器，代码未涉及。见第 8 节。
- 双核分工：core0 跑主循环——读按键、组装报告、`inputDriver->process(gamepad)`、`tud_task()`、`USBHostManager::process()`（即 `tuh_task()`）（`src/gp2040.cpp:249-313`，其中 277、304、308 行）；core1 跑辅助循环——`inputDriver->processAux()`，即认证状态机轮询（`src/gp2040aux.cpp:53-64`）。两侧通过 `P5GeneralAuthData` 共享结构交换数据，无锁（实现细节见第 9 节）。

---

## 2. 枚举阶段：呈现给 PS5 的 USB 设备形态

以下描述符由 TinyUSB 设备栈在标准 `GET_DESCRIPTOR` 请求时回调返回（路由：`src/usbdriver.cpp:74-100` 的 `tud_descriptor_*_cb` → `DriverManager::getDriver()` 的对应虚函数 → 本驱动的静态数组）。

### 2.1 设备描述符（18 字节）

来源：`headers/drivers/p5general/P5GeneralDescriptors.h:124-140`。

| 偏移 | 字段 | 值 | 说明 |
|---|---|---|---|
| 0 | bLength | 0x12 | |
| 1 | bDescriptorType | 0x01 | Device |
| 2-3 | bcdUSB | 0x0200 | USB 2.00 |
| 4 | bDeviceClass | 0x00 | 类定义在接口 |
| 5 | bDeviceSubClass | 0x00 | |
| 6 | bDeviceProtocol | 0x00 | |
| 7 | bMaxPacketSize0 | 0x40 | 控制端点 64 字节 |
| 8-9 | idVendor | **0x2B81** | 字节序 `81 2B` |
| 10-11 | idProduct | **0x0101** | 字节序 `01 01` |
| 12-13 | bcdDevice | 0x0001 | |
| 14 | iManufacturer | 0x01 | 字符串 "Activtor"（原文如此） |
| 15 | iProduct | 0x02 | 字符串 "P5General" |
| 16 | iSerialNumber | 0x00 | 无序列号 |
| 17 | bNumConfigurations | 0x01 | |

字符串表（`P5GeneralDescriptors.h:111-122`）：索引 0 = 语言 0x0409（美式英语）；1 = `Activtor`；2 = `P5General`；3 = `0.1`。字符串描述符运行时由 `getStringDescriptor()` 组装为 USB UTF-16LE 形式（`headers/drivers/shared/driverhelper.h:4-25`；`P5GeneralDriver.cpp:313-317` 为路由入口）。

### 2.2 配置描述符（41 字节，单配置单接口双端点）

来源：`headers/drivers/p5general/P5GeneralDescriptors.h:236-278`。

| 段 | 字段 | 值 |
|---|---|---|
| Configuration | wTotalLength | 41 (0x29) |
| | bNumInterfaces | 1 |
| | bConfigurationValue | 1 |
| | bmAttributes | 0x80（总线供电；**未**置远程唤醒位 D5） |
| | bMaxPower | 0xFA = 500 mA |
| Interface #0 | bAlternateSetting | 0 |
| | bNumEndpoints | 2 |
| | bInterfaceClass / SubClass / Protocol | 0x03 / 0x00 / 0x00（HID，无子类/协议） |
| | iInterface | 0 |
| HID | bcdHID | 0x0111 (1.11) |
| | bCountryCode | 0 |
| | bNumDescriptors / 类型 / 长度 | 1 / 0x22 (Report) / **165 (0xA5)** |
| Endpoint | bEndpointAddress | **0x82**（IN，端点 2） |
| | bmAttributes | 0x03（中断） |
| | wMaxPacketSize | 64 |
| | bInterval | **1**（full speed 下即 1 ms 轮询） |
| Endpoint | bEndpointAddress | **0x01**（OUT，端点 1） |
| | bmAttributes | 0x03（中断） |
| | wMaxPacketSize | 64 |
| | bInterval | **6**（full speed 下即 6 ms） |

报告描述符长度 165 字节经独立核对与数组实际字节数一致 **[代码]**（`P5GeneralDescriptors.h:142-223`，去除注释后恰为 165 字节）。

注意一个实现级事实：配置描述符未声明远程唤醒支持，但驱动在总线挂起时会调用 `tud_remote_wakeup()`（`src/drivers/p5general/P5GeneralDriver.cpp:110-112`）。**[推断]** 该调用在未置 D5 位的主机上的实际效果未在代码中验证。

### 2.3 报告率

- 面向 PS5 的输入报告：受 IN 端点 `bInterval=1`（1 ms）约束，理论最高 1000 Hz；实际节奏由"状态变化 → 加密狗签名 → 发送"流水线决定（第 6 节），空闲（无输入变化）时**不发送任何输入报告** **[代码]**（`P5GeneralDriver.cpp:229-242`：报告内容与上次相同且 4 次重复发完后返回 false，不产生任何传输）。
- 面向加密狗：主机侧以 TinyUSB 中断 IN 轮询模型持续收报告（挂载即调用 `tuh_hid_receive_report`，每收到一份立即再次排队，`src/usbhostmanager.cpp:108-114`、`141-148`），节奏由加密狗自身的端点 `bInterval` 决定（其描述符本代码不读取，见第 10 节）。

---

## 3. HID 报告总览与报告描述符解析

报告描述符（`headers/drivers/p5general/P5GeneralDescriptors.h:142-223`）声明了两个顶层 Application Collection：

**Collection 1：Generic Desktop / Game Pad**

| Report ID | 类型 | 长度（线上，含 ID 字节） | 声明的 Usage | 驱动是否实现 |
|---|---|---|---|---|
| 0x01 | Input | 64 | 见 3.1 逐项 | 是（第 4、6 节） |
| 0x02 | Output | 48 | Vendor Page 0xFF00 Usage 0x23，47 字节 | **否**——OUT 端点数据被驱动丢弃（`P5GeneralDriver.cpp:297-299` 对非 Feature 类型直接 return） |
| 0x03 | Feature | 48 | Vendor Page 0xFF00 Usage 0x2821，47 字节 | 是——返回静态 47 字节 `output_0x03`（第 5.2 节） |
| 0xE0 | Feature | 3 | Vendor Page 0xFF80 Usage 0x57，2 字节 | **否**——驱动无该分支，`GET_REPORT` 走 STALL（`P5GeneralDriver.cpp:269-289` 无匹配分支返回 -1；TinyUSB 对返回值 0 的断言导致 STALL，`lib/tinyusb/src/class/hid/hid_device.c:305-306`） |

**Collection 2：Vendor Page 0xFFF0 / Usage 0x40**（认证专用）

| Report ID | 类型 | 长度（线上，含 ID 字节） | Usage | 用途 |
|---|---|---|---|---|
| 0xF0 | Feature | 64 | 0x47，63 字节 | 认证质询载入（SET） |
| 0xF1 | Feature | 64 | 0x48，63 字节 | 读取签名/nonce（GET） |
| 0xF2 | Feature | 16 | 0x49，15 字节 | 读取签名状态（GET） |

### 3.1 输入报告 0x01 的位级布局（报告描述符视角）

`P5GeneralDescriptors.h:144-192` 依次声明（报告 ID 0x01 之后）：

1. 6 个 8 位绝对轴（Usage X, Y, Z, Rz, Rx, Ry，逻辑 0–255）→ 字节 1–6；
2. Vendor Page 0xFF00 Usage 0x20，8 位 ×1 → 字节 7（结构体的 `reportCounter`）；
3. 4 位 Hat switch（逻辑 0–7，物理 0–315°，Null State）；
4. Button 页 Usage 1–14，1 位 ×14；
5. Vendor Page 0xFF00 Usage 0x21，1 位 ×14（纯填充位，覆盖字节 10 剩余 6 位 + 字节 11 全部 8 位）；
6. Vendor Page 0xFF00 Usage 0x22，8 位 ×52 → 字节 12–63。

位 packing **[代码]**（描述符顺序 + `P5GenerorReport` 位域，`P5GeneralDescriptors.h:62-109`）：字节 8 = hat(bit0–3) ‖ west/south/east/north(bit4–7)；字节 9 = l1/r1/l2/r2/select/start/l3/r3；字节 10 bit0–1 = home/touchpad，bit2–7 为 Usage 0x21 填充；字节 11 = Usage 0x21 填充（结构体字段 `data_11`）。

---

## 4. 输入报告 0x01 逐字节格式

结构体定义：`headers/drivers/p5general/P5GeneralDescriptors.h:62-109`（`packed, aligned(1)`，总长 64 字节）。填充逻辑：`src/drivers/p5general/P5GeneralDriver.cpp:127-227`。

| 偏移 | 长度 | 字段 | 语义 | 来源 |
|---|---|---|---|---|
| 0 | 1 | report_id | 常量 0x01 | `P5GeneralDriver.cpp:52` |
| 1 | 1 | left_stick_x | 8 位，中位 0x80（`P5GENERAL_JOYSTICK_MID`），由 16 位内部状态右移 8 位截断 | `P5GeneralDriver.cpp:156` |
| 2 | 1 | left_stick_y | 同上 | :157 |
| 3 | 1 | right_stick_x | 同上 | :158 |
| 4 | 1 | right_stick_y | 同上 | :159 |
| 5 | 1 | left_trigger | 模拟扳机 0–255；无模拟扳机硬件时数字 L2 按下 = 0xFF | `P5GeneralDriver.cpp:160-166` |
| 6 | 1 | right_trigger | 同上 | 同上 |
| 7 | 1 | reportCounter | **GP2040 从不递增**（初始化为 0 后不再写）；DualSense 主机路径用它判重（第 8 节） | `P5GeneralDriver.cpp:51-66`（初始化后无写点） |
| 8 | 4b+4b | dpad ‖ west,south,east,north | hat 0–7 / 0x0F 中性（`P5GENERAL_HAT_*`，`P5GeneralDescriptors.h:18-26`） | `P5GeneralDriver.cpp:129-145` |
| 9 | 8 bit | l1,r1,l2,r2,select,start,l3,r3 | 位序即此顺序 | `P5GeneralDriver.cpp:147-153` |
| 10 | 2 bit + 6 bit | home(bit0), touchpad(bit1) ‖ 填充(bit2–7) | | `P5GeneralDriver.cpp:154-155` |
| 11 | 1 | data_11 | 恒 0（仅出现在初始化的零值里） | 结构体零值 |
| 12–15 | 4 | auth_seq_number | 恒 0，驱动不写 | 同上 |
| 16–21 | 6 | gyroscope | 3 × int16；写入时对内部 aux 状态做字节交换（`(lo<<8)\|(hi>>8)`，`P5GeneralDriver.cpp:169-173`），即线上字节序与 aux 状态相反 | :169-173 |
| 22–27 | 6 | accelerometer | 同上 | :176-180 |
| 28–29 | 2 | data_28_29 | 恒 0 | 结构体零值 |
| 30–31 | 2 | data_30_31_0x001a | **固定 0x001A**（小端线上字节 `1A 00`），初始化时写入一次 | `P5GeneralDriver.cpp:64` |
| 32–35 | 4 | touchpad_data.p1 | 见下 | `PS4Descriptors.h:107-129`（TouchpadXY/TouchpadData，经 `P5GeneralDescriptors.h` 引入）；`P5GeneralDriver.cpp:183-207` |
| 36–39 | 4 | touchpad_data.p2 | 见下 | 同上 |
| 40–55 | 16 | data_40_55 | 恒 0 | 结构体零值 |
| 56–63 | 8 | hash | **GP2040 从不写**。该字段与共享结构的 `hash_pending_buffer` / `hash_finish_buffer` 命名一致，**[推断]** 由加密狗在返回报告中填写，是 PS5 校验的逐报告认证数据（第 6、10 节） | 结构体零值 + 数据流（`P5GeneralAuthUSBListener.cpp:115-120`） |

TouchpadXY（每个触点 4 字节，`PS4Descriptors.h:107-124`，经 `P5GeneralDescriptors.h` 引入）：

| 字节 | 位 | 语义 |
|---|---|---|
| +0 | bit0–6 | counter：触点计数器，每次新触点 +1，上限 128 回绕（`P5GENERAL_TP_MAX_COUNT`，`P5GeneralDriver.cpp:209-226`） |
| +0 | bit7 | unpressed：1 = 未按下 |
| +1 | – | X 低 8 位 |
| +2 | 低 4 位 / 高 4 位 | X 高 4 位 / Y 低 4 位 |
| +3 | – | Y 高 8 位 |

X/Y 为 12 位，分辨率 1920 × 943（`P5GENERAL_TP_X_MAX/ Y_MAX`，`P5GeneralDescriptors.h:50-53`）。触摸板按键仿真：无真实触摸传感器时，A2 = 触摸板按下（中心）、A3 = 左、A4 = 右（`P5GeneralDriver.cpp:182-193`）；`switchTpShareForDs4` 选项交换 Select/触摸板按键语义（`P5GeneralDriver.cpp:150-155`）。

---

## 5. 认证握手（控制传输）

### 5.1 控制传输编码

所有认证交互都是 HID 类请求、接口Recipient，经过 TinyUSB 封装为标准控制传输 **[代码]**：

| 方向 | bmRequestType | bRequest | wValue | wIndex | wLength | 场景 |
|---|---|---|---|---|---|---|
| PS5 → GP2040（数据 OUT） | 0x21 | 0x09 (SET_REPORT) | 0x03F0（Feature << 8 \| 0xF0） | 0（接口号） | 64 | 质询载入 |
| PS5 → GP2040（数据 IN） | 0xA1 | 0x01 (GET_REPORT) | 0x0303 | 0 | 48 | 读定义 |
| PS5 → GP2040（数据 IN） | 0xA1 | 0x01 | 0x03F1 | 0 | 64 | 读签名/nonce |
| PS5 → GP2040（数据 IN） | 0xA1 | 0x01 | 0x03F2 | 0 | 16 | 读签名状态 |
| GP2040 → 加密狗（数据 OUT） | 0x21 | 0x09 | 0x03F0 | 加密狗接口号 | 64 | 质询转发 |
| GP2040 → 加密狗（数据 IN） | 0xA1 | 0x01 | 0x03F1 | 同上 | 64 | 取签名/nonce |
| GP2040 → 加密狗（数据 IN） | 0xA1 | 0x01 | 0x03F2 | 同上 | 16 | 取签名状态 |

编码依据：设备侧 `lib/tinyusb/src/class/hid/hid_device.c:289-331`（GET_REPORT 预置 1 字节报告 ID 再回调、SET_REPORT 在 ACK 阶段剥掉匹配的报告 ID 字节后回调，:299-303、:324-327）；主机侧 `lib/tinyusb/src/class/hid/hid_host.c:240-311`（`tuh_hid_get_report`/`tuh_hid_set_report` 构造的 bmRequestType 分别为 IN/A1 与 OUT/21，wValue = `tu_u16(report_type, report_id)`，wIndex = 对端接口号）；Feature 类型值 = 3（`lib/tinyusb/src/class/hid/hid.h:86-89`）。设备侧控制缓冲 64 字节（`CFG_TUD_HID_EP_BUFSIZE` 默认值，`lib/tinyusb/src/class/hid/hid_device.h:46-48`），因此 SET_REPORT 的 wLength 不得超过 64。

**数据阶段字节布局（全部特征报告）**：线上第 0 字节 = 报告 ID，其后为净荷。设备侧回调看到的 buffer 已被 TinyUSB 剥去 ID 字节（SET）/在 ID 字节之后写入（GET）。

### 5.2 Feature 0x03：设备定义（静态应答）

PS5 `GET_REPORT(Feature, 0x03)` → 线上 48 字节 = `[0x03][47 字节 output_0x03]`。`output_0x03`（`src/drivers/p5general/P5GeneralDriver.cpp:19-26`，47 字节）：

```
21 28 03 C3 00 2C 56
01 00 D0 07 00 80 04 00
00 80 0D 0D 84 00 00 00
00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00
```

**[代码]**：字节 0–1 = `21 28`，与报告描述符中该报告的 Usage 0x2821 的小端表示一致；应答长度受 `reqlen >= 47` 约束，正常路径（wLength=48）恰好 47 字节（`P5GeneralDriver.cpp:270-276`）。**[推断]**：其余字段的语义（设备能力/配置位图）与 PS4 驱动的同号特征报告结构同源——PS4 的 `output_0x03`/`controllerConfig`（`src/drivers/ps4/PS4Driver.cpp:29-46`、`:283-297`）开头两字节同样是 Usage 回显（`21 27` ↔ Usage 0x2721），且 `0D 0D`、`0D 84` 等字节在两者中位置对应；P5 版本应为同类"控制器能力声明"。逐位含义代码不可考。

### 5.3 Feature 0xF0：质询载入（PS5 → 设备 → 加密狗）

1. PS5 `SET_REPORT(Feature, 0xF0)`，数据 64 字节：`[0xF0][63 字节质询净荷]`。
2. 设备侧回调要求 `bufsize == 63`（TinyUSB 已剥去 ID 字节；等价于线上 wLength = 64），否则静默忽略（`P5GeneralDriver.cpp:301-304`）；且状态机必须处于 `p5g_auth_idle`，否则同样忽略（`:305`）。
3. 接受后：`auth_buffer = [0xF0][63 字节净荷]`（64 字节），状态 → `p5g_auth_send_f0`（`P5GeneralDriver.cpp:306-309`）。
4. core1 轮询到 `send_f0`：把 `auth_buffer` 原样 64 字节作为 `SET_REPORT(Feature, 0xF0)` 发给加密狗（`P5GeneralAuthUSBListener.cpp:44-49`、`:79-82`）——即**逐字节原样转发，不校验、不修改**（对比：PS4 驱动会先验证净荷尾部 CRC32，`src/drivers/ps4/PS4Driver.cpp:893-899`；P5 路径虽包含 `CRC32.h` 但从未使用，`P5GeneralAuthUSBListener.cpp:5`）。状态 → `send_f0_wait`。
5. 加密狗 ACK（主机侧 `set_report_complete`，成功时 len = 64 ≠ 0 才分发，`src/usbhostmanager.cpp:151-154`）→ 状态机读 `auth_buffer[1]`（净荷第 1 字节，下称 **类型字节**）与 `auth_buffer[3]`（净荷第 3 字节）分派（`P5GeneralAuthUSBListener.cpp:129-160`）：

| auth_buffer[1] | f1_num（后续 F1 取回次数配额） | 自动 F2（500 ms 后） | 下一状态 |
|---|---|---|---|
| 0x01（且 auth_buffer[3] == 3） | 4 | 是 | `recv_f2_delay_500mS` |
| 0x01（且 auth_buffer[3] != 3） | 4 | 否 | `idle`（等待 PS5 读 F1） |
| 0x02 | 0 | 是 | `recv_f2_delay_500mS` |
| 0x03 | 1 | 是 | `recv_f2_delay_500mS` |
| 其它 | 0 | 否 | `idle` |

**[推断]**（依据 PS4 同构协议）：类型字节对应认证阶段号（nonce_id），0x01 = nonce 载入、0x02/0x03 = 后续签名交换；净荷第 2 字节（auth_buffer[2]）在 PS4 协议中为 nonce 页号。P5 代码只消费上述两处分支条件，不解释其含义。

### 5.4 Feature 0xF1：签名/nonce 读取（取回-应答流水线）

- PS5 `GET_REPORT(Feature, 0xF1)` → 线上 64 字节 = `[0xF1][63 字节]`，内容取自共享 `auth_buffer`（跳过其第 0 字节，`P5GeneralDriver.cpp:277-282`）。**应答是同步的**：控制传输 SETUP 阶段直接返回 `auth_buffer` 当前内容，同时（若状态为 idle）把状态置为 `recv_f1`，请求 core1 到加密狗取**下一份** F1（`P5GeneralDriver.cpp:279-281`）。
- core1 见 `recv_f1` 且 `f1_num > 0`：向加密狗发 `GET_REPORT(Feature, 0xF1, wLength=64)`，状态 → `recv_f1_wait`，`f1_num--`；若 `f1_num == 0`：直接回 `idle`，不取（`P5GeneralAuthUSBListener.cpp:50-59`）。
- 加密狗返回 64 字节（`[0xF1][63 字节]`）→ `get_report_complete` 把整个 64 字节写回 `auth_buffer`，状态 → `idle`（`P5GeneralAuthUSBListener.cpp:171-178`）。
- 净效果是**滞后一次取数**：PS5 的第 N 次 GET 拿到的是为第 N−1 次请求取回的数据；第 1 次拿到的是 `auth_buffer` 的旧内容（开机后为全零，`src/drivers/p5general/P5GeneralAuth.cpp:12`）。
- f1_num 耗尽后再来的 GET(F1) 只会重复返回最后一份取回数据（每次仍会短暂进入 `recv_f1` 再回 `idle`）。
- **[推断]**（PS4 同构）：F1 净荷布局为 `[nonce_id][chunk#][0][56 字节签名块][CRC32]`（PS4 侧 19 块 × 56 字节，`src/drivers/ps4/PS4Driver.cpp:823-842`）。P5 的 f1_num 只有 4 或 1，暗示 P5 的签名块数更少或取回策略不同；净荷语义以加密狗为准，代码不可考。

### 5.5 Feature 0xF2：签名状态（500 ms 自动取回）

- 满足自动 F2 条件的质询被加密狗 ACK 后，记录 `auth_recv_f2_us = 当前时刻 + 500 000 µs`，状态 → `recv_f2_delay_500mS`（`P5GeneralAuthUSBListener.cpp:147-149`）。
- core1 轮询到时后：`GET_REPORT(Feature, 0xF2, wLength=16)` → 状态 `recv_f2_wait` → 完成后 `auth_buffer` 被 16 字节（`[0xF2][15 字节]`）覆盖，回 `idle`（`P5GeneralAuthUSBListener.cpp:60-70`、`:179-186`）。
- PS5 `GET_REPORT(Feature, 0xF2)` → 线上 16 字节 = `[0xF2][15 字节]`，取自 `auth_buffer+1`（`P5GeneralDriver.cpp:283-288`）。注意该分支同样把状态置为 `recv_f1`（与 F1 分支同一段代码，`P5GeneralDriver.cpp:285-287`）；若此时 `f1_num > 0`，会额外触发一次对加密狗的 F1 取回——这是代码的既有行为。
- **[推断]**（PS4 同构）：F2 净荷 `[nonce_id][状态][9 字节零][CRC32]`，状态 0 = 签名就绪、16 = 仍在计算（`src/drivers/ps4/PS4Driver.cpp:844-851`）。

### 5.6 状态机全表

状态定义：`headers/drivers/p5general/P5GeneralAuth.h:6-15`。迁移来源见上节标注。

```
idle ──(SET_REPORT F0 接受)──► send_f0 ──(core1 发出主机 SET F0)──► send_f0_wait
  ▲                                                                        │
  │                                                          (加密狗 ACK，按类型分派)
  │                                                                        │
  │        ┌── 不满足自动 F2 条件 ◄───────────────────────────────────────┤
  │        ▼                                                               ▼
  ├──(PS5 GET F1/F2，且 idle)──► recv_f1 ──f1_num>0──► recv_f1_wait ──   recv_f2_delay_500mS
  │                                  │                      │              │(到时)
  │                              f1_num==0                  │(主机 GET F1 完成)   ▼
  │                                  │                      ▼            recv_f2
  └──────────────────────────────────┘                      idle   ──(发出主机 GET F2)──► recv_f2_wait
                                                                              │(主机 GET F2 完成)
                                                                              ▼
                                                                            idle
```

进入/退出条件汇总：

| 状态 | 进入条件 | 退出条件 |
|---|---|---|
| idle | 初始 / 各完成点 / f1_num=0 的 recv_f1 | 收到 F0（SET_REPORT）或 PS5 读 F1/F2（get_report 回调内武装） |
| send_f0 | 设备侧接受 F0 | core1 轮询周期内即迁移 |
| send_f0_wait | 主机 SET_REPORT(F0) 已提交 | 加密狗 ACK（成功） |
| recv_f1 | PS5 GET(F1) 或 GET(F2) 且状态为 idle | core1 轮询周期内分派 |
| recv_f1_wait | 主机 GET_REPORT(F1) 已提交 | 加密狗返回数据（成功） |
| recv_f2_delay_500mS | F0 ACK 且满足自动 F2 条件 | `getMicro() >= auth_recv_f2_us`（500 ms） |
| recv_f2 | 延迟到期 | core1 轮询周期内即迁移 |
| recv_f2_wait | 主机 GET_REPORT(F2) 已提交 | 加密狗返回数据（成功） |

---

## 6. 运行时数据流：输入报告的签名流水线

数据通路（全部 **[代码]**）：

```
core0: 按键状态 → P5GeneralReport(64B)
   │  状态有变化（或重复配额未用完）时
   ▼  hash_pending_buffer = 报告副本; hash_pending = true        (P5GeneralDriver.cpp:229-242)
core1: process() 看到 hash_pending 且 OUT 端点空闲
   │  tuh_hid_send_report(dev, inst, report_id=0, buf, 64)       (P5GeneralAuthUSBListener.cpp:38-41)
   ▼  → 加密狗中断 OUT 端点：64 字节原始数据（无额外 ID 前缀，首字节即 0x01）
加密狗内部处理（黑盒）
   ▼
主机持续轮询加密狗中断 IN 端点 → 64 字节报告
   │  report_received: 若 !hash_ready，hash_finish_buffer = 报告  (P5GeneralAuthUSBListener.cpp:115-120)
   ▼  hash_ready = true；若上一份尚未发走则丢弃本份
core0: process() 看到 hash_ready 且 tud_hid_ready()
   │  tud_hid_report(0, hash_finish_buffer, 64)                  (P5GeneralDriver.cpp:114-121)
   ▼  → PS5 的 IN 端点 0x82：64 字节原样转发（report_id=0 表示不再加前缀，首字节 0x01 即线上 ID）
```

关键规则：

- **加密狗未就绪前不产生任何输出**：`dongle_ready == false` 时 `process()` 直接返回（`P5GeneralDriver.cpp:106-108`），PS5 侧收不到任何输入报告。
- **变化驱动 + 4 次重复**：报告与上一份不同时置 `diff_report_repeat = 4`（`P5GeneralDriver.cpp:233`）；内容不变但重复配额 > 0 时继续发送同一份（`:235-239`）；配额耗尽且无变化则不发（`:240-242`）。空闲时线上静默。
- **发送节流**：`hash_pending == true`（有待签名报告）期间 core0 不再产生新报告（`P5GeneralDriver.cpp:123-125`）；`hash_ready == true` 且上一份未成功发出（端点忙）期间同理（`:114-121`）。
- **丢弃规则**：加密狗的 IN 报告在上一份 `hash_finish_buffer` 尚未成功发给 PS5 时直接丢弃（`P5GeneralAuthUSBListener.cpp:116-119`）——流水线背压靠丢弃而非排队。
- **挂起唤醒**：总线挂起时每轮尝试 `tud_remote_wakeup()`（`P5GeneralDriver.cpp:110-112`）。
- **控制流完整性**：发往 PS5 的输入报告**永远是加密狗返回的字节**，GP2040 不做任何改写（`P5GeneralDriver.cpp:115` 直接发送 `hash_finish_buffer`）。**[推断]**：加密狗在返回前会（至少）填写末 8 字节 `hash` 字段，可能同时改写 `reportCounter`/`auth_seq_number` 等字段——GP2040 自己从不维护这些字段，若 PS5 要求它们单调变化，只能由加密狗完成（见第 10 节）。
- **PS5 → 设备的输出报告（震动/LED 等）被丢弃**：OUT 端点数据经 TinyUSB 以 `HID_REPORT_TYPE_OUTPUT` 进入 `set_report`，驱动对非 Feature 类型直接返回（`lib/tinyusb/src/class/hid/hid_device.c:389-398`；`P5GeneralDriver.cpp:297-299`）。**[推断]** 该驱动不支持把震动/LED 转发给加密狗或本地执行。

---

## 7. 加密狗侧对接契约（主机实现指南）

在任意平台充当"GP2040 角色"、驱动一枚真实 P5 General 加密狗所需的全部契约 **[代码]**（实现在 `src/drivers/p5general/P5GeneralAuthUSBListener.cpp`）：

1. **枚举与识别**：枚举 USB 设备，匹配 `VID == 0x2B81 && PID == 0x0101`，取其 HID 接口（类 0x03）。代码不解析报告描述符、不检查 Usage 页（对比 PS4 监听器会检查 0xFFF0/0xF3，`src/drivers/ps4/PS4AuthUSBListener.cpp:90-105`——P5 监听器无此检查）。第一个匹配即锁定（`P5GeneralAuthUSBListener.cpp:84-100`）。
2. **中断 IN**：持续提交 64 字节轮询；每份到达的报告即"待发往主机的输入报告"（挂载时与每份报告后都要重新排队，`src/usbhostmanager.cpp:108-114`、`141-148`）。
3. **中断 OUT**：写入 64 字节原始输入报告（无独立报告 ID 前缀；缓冲首字节 0x01 即报告 ID）。发送前确认端点空闲（`tuh_hid_send_ready` → `!usbh_edpt_busy(ep_out)`，`lib/tinyusb/src/class/hid/hid_host.c:373-377`、`379-404`）。
4. **控制传输（全部 Feature、Recipient=接口）**：
   - `SET_REPORT` F0：`0x21, 0x09, wValue=0x03F0, wIndex=接口号, wLength=64`，数据 = 上游质询原样 64 字节；
   - `GET_REPORT` F1：`0xA1, 0x01, wValue=0x03F1, wIndex=接口号, wLength=64`，期望 `[0xF1][63B]`；
   - `GET_REPORT` F2：`0xA1, 0x01, wValue=0x03F2, wIndex=接口号, wLength=16`，期望 `[0xF2][15B]`。
5. **状态机与时序**：按第 5.6 节表格驱动；唯一的显式时序参数是 F0 成功后 **500 ms** 再取 F2；F1 的取回配额 4/1/0 由质询类型字节决定。
6. **拔出处理**：设备移除即置 `dongle_ready = false`、清地址（`P5GeneralAuthUSBListener.cpp:102-113`）；注意**状态机不复位**（`resetHostData()` 为空，`:30-32`），见第 9 节失败路径。

若要在 Linux 上以 raw_gadget 实现**设备侧**（面向 PS5），除第 2 节的全部描述符字节外还需实现：GET_REPORT(0x03/0xF1/0xF2) 的应答来源与 STALL 语义（未实现的报告 ID 一律 STALL）、SET_REPORT(0xF0) 的接受条件（wLength=64、状态 idle）、IN 端点 1 ms 服务、OUT 端点接收后静默丢弃，以及挂起时的远程唤醒尝试。

---

## 8. DualSense 主机输入路径

`DualsensePS5Host`（`src/hosts/DualsensePS5Host.cpp`、`headers/hosts/DualsensePS5Host.h`）是通用主机口手柄插件的 PS5 分支，不属于认证链路：

- 匹配：VID 0x054C、PID 0x0CE6（DualSense 无线适配器有线形态，`DualsensePS5Host.cpp:3-13`）。
- 报告解析：`report[0] == 0x01` 时按 `P5GenerorReport` 结构整块解读 64 字节（`:32-35`）——**[代码]** 真 DualSense 的输入报告 0x01 与 `P5GenerorReport` 布局重合（同结构可直接 memcpy）；**[推断]** P5 General 协议即 DualSense HID 方言的派生。
- 判重：仅当 `reportCounter` 变化时更新状态（`:37-70`）。
- 映射：摇杆 `map(v, 0..255, 0..65535)` 线性放大到 16 位内部状态（`:38-41`、`:91-93`）；扳机 8 位直通；按键与 hat 按位映射到 GP2040 内部掩码（`:45-69`）。
- 输出能力声明：`hasAnalogTriggers/hasLeftAnalogStick/hasRightAnalogStick = true`（`:76-80`）。
- 该主机路径不向 DualSense 写任何输出报告（`set_report_complete`/`get_report_complete` 为空实现，`DualsensePS5Host.h:24-25`）。

---

## 9. 时序、重试与失败路径汇总

| 项 | 数值/行为 | 来源 |
|---|---|---|
| F2 自动取回延迟 | 500 ms（`getMicro() + 500*1000` µs） | `P5GeneralAuthUSBListener.cpp:148` |
| F1 取回配额 | 类型 0x01 → 4；0x03 → 1；0x02/其它 → 0 | `:133-142` |
| 设备→PS5 IN 端点 bInterval | 1 ms | `P5GeneralDescriptors.h:270` |
| 设备→PS5 OUT 端点 bInterval | 6 ms | `:277` |
| 重复发送配额 | 每次状态变化后 4 份相同报告 | `P5GeneralDriver.cpp:233-239` |
| 触摸计数器回绕 | 128 | `P5GeneralDescriptors.h:55`；`P5GeneralDriver.cpp:209-226` |
| `P5GENERAL_KEEPALIVE_US` | 5000 µs，**已定义未使用**（无任何引用点） | `P5GeneralDriver.cpp:8` |
| 主机侧控制传输重试 | **无**。每次 GET/SET 只提交一次 | `P5GeneralAuthUSBListener.cpp:74-82` |
| 传输失败后果 | 主机侧完成回调以 len=0 上报时被 `usbhostmanager` 过滤不分发（`src/usbhostmanager.cpp:151-154`、`158-161`），状态机停留在 `*_wait`，**无超时恢复**；后续 F0 也不被接受（需要 idle） | 同上 + `P5GeneralDriver.cpp:305` |
| 加密狗拔出 | `dongle_ready=false`、地址清 0xFF；**状态机不复位**（`resetHostData()` 空），若卡在 `*_wait` 重插后仍卡死 | `P5GeneralAuthUSBListener.cpp:102-113`、`:30-32` |
| SET_REPORT(F0) 长度错误 | wLength ≠ 64（回调 bufsize ≠ 63）静默忽略 | `P5GeneralDriver.cpp:302-304` |
| SET_REPORT(F0) 状态非 idle | 静默忽略 | `:305` |
| 未实现的特征报告 GET | 设备 STALL（驱动返回 -1 → TinyUSB 断言失败） | `P5GeneralDriver.cpp:290`；`hid_device.c:305-306` |
| OUT 端点（PS5 输出报告） | 收到即丢弃 | `P5GeneralDriver.cpp:297-299` |
| GET(F1)/GET(F2) 读到陈旧数据 | 见 5.4"滞后一次取数" | `P5GeneralDriver.cpp:277-288` |
| 并发 | `auth_buffer`/`hash_*` 在 core0（USB 设备栈回调）与 core1（监听器轮询）间无锁共享；报告流水线为单槽（pending/finish 各一） | `P5GeneralAuth.h:17-29` |

---

## 10. 未能从代码确定的事项

以下各项在 GP2040-CE 源码中不可考，本文档不做猜测：

1. **加密狗的签名算法与密钥**：一切密码运算在加密狗内部完成，GP2040 只做字节搬运。密钥形态、算法（对称 MAC / 非对称签名 / 其它）完全不可见。
2. **`hash[8]` 字段的密码学语义**：仅知它由加密狗填写（GP2040 恒不写该字段、报告必须经加密狗往返）。是 CRC、截断 MAC 还是序列绑定的签名无法确定；字段名是唯一线索。
3. **质询净荷（F0 63 字节）的内部布局**：代码只消费净荷第 1 字节（类型）与第 3 字节（==3 判据）。其余 61 字节的语义、是否含 CRC32（PS4 同构协议在尾部有 CRC32，`src/drivers/ps4/PS4Driver.cpp:893-899`，但 P5 路径不校验）、nonce 长度与分块方式均不可考。
4. **F1/F2 应答净荷的内部布局**：以 64/16 字节黑盒透传。PS4 的 `[id][chunk][0][56B][CRC32]` / `[id][state][pad][CRC32]` 布局只是类比推断，未经 P5 侧证实。
5. **PS5 侧的轮询次数与判定逻辑**：控制台读 F1 几次、何时认为认证成功/失败、类型字节的协议含义（0x01/0x02/0x03 对应哪个认证阶段）——这些属于 PS5 固件行为。
6. **加密狗自身的 USB 描述符**：代码只按 VID/PID 认狗，不读其设备/配置/报告描述符（不似 PS4 监听器解析 Usage 页）。其端点号、bInterval、接口数、字符串均需实测枚举获得。
7. **加密狗 IN 报告的节奏与内容关系**：其自发报告率、对每份 OUT 报告是否一一对应回一份 IN 报告、除 `hash` 外还改写哪些字段（如 `reportCounter`、`auth_seq_number`）不可考。
8. **report 0xE0（2 字节特征报告）与 report 0x02（47 字节输出报告）的协议语义**：描述符声明了它们，驱动未实现（0xE0 读则 STALL，0x02 收则丢弃）。PS5 是否使用、格式如何不可考。
9. **加密狗对报告内容字段的最低要求**：GP2040 恒不递增 `reportCounter`/`auth_seq_number`，若协议要求单调性，只能由加密狗在返回时改写——是否如此不可考。
10. **克隆身份与真实加密狗描述符的逐字节一致性**：VID/PID 一致有代码证据（0x2B81/0x0101 同时出现在设备描述符与主机匹配处）；bcdDevice、字符串、端点表是否与真实加密狗完全一致未在本仓库中验证。
11. **`CRC32.h` 的包含关系**：`P5GeneralAuthUSBListener.cpp:5` 包含了 CRC32 头文件但全文件无任何调用——P5 协议是否在任何环节使用 CRC32 不可考。
12. **远程唤醒有效性**：描述符 bmAttributes=0x80 未声明远程唤醒，而代码调用 `tud_remote_wakeup()`；PS5 是否响应未验证。

---

## 11. 配置语义与用户可见行为

- **模式枚举**：`INPUT_MODE_P5GENERAL = 16`（`proto/enums.proto:159`）；`DriverManager::setup` 在该模式下实例化 `P5GeneralDriver`（`src/drivermanager.cpp:62-64`）。
- **启用条件**：认证子系统要求外设块 USB0 使能（`P5GeneralAuth::available()` → `PeripheralManager::isUSBEnabled(0)`，`src/drivers/p5general/P5GeneralAuth.cpp:18-20`）；PIO 主机栈仅在"USB0 使能且存在监听器"时启动（`src/usbhostmanager.cpp:16-24`）。Web 配置将该模式标记为"可选 USB、认证 USB"，说明文字明确"需要 USB 主机连接与 P5General 才能在 PS5 通用模式下正确认证"（`www/src/Data/InputBootModes.ts:193-200`；`www/src/Locales/en/SettingsPage.jsx:85-86`）。
- **监听器注册链**：`GP2040Aux::setup()` 调用 `initializeAux()`（创建认证驱动与监听器），并把 `get_usb_auth_listener()` 返回的监听器推入 `USBHostManager`（`src/gp2040aux.cpp:29-37`）；core1 循环持续调用 `processAux()` 驱动状态机（`:53-64`）。
- **UI/LED**：状态栏显示 "P5G"，认证完成标志 `getAuthSent()` 恒为 false（":AS" 永不显示，`headers/drivers/p5general/P5GeneralDriver.h:37`；`src/display/ui/screens/ButtonLayoutScreen.cpp:288-293`）；玩家 LED 动画复用 PS4 样式（`src/addons/neopicoleds.cpp:305-307`）；主菜单名称 "P5 General"（`headers/display/ui/screens/MainMenuScreen.h:25`）。
- **配置保存门控**：`Storage::save` 的"PS4/PS5 + USB 认证时禁存盘"规则不含 P5GENERAL 分支（`src/storagemanager.cpp:44-54`）；`getDongleAuthRequired()` 恒 true（`P5GeneralDriver.cpp:100-103`）但当前无调用方在 P5 模式下触达它。
- **构建**：三个 P5 源文件列入固件构建（`CMakeLists.txt:219-221`）。

---

## 12. 实现核对清单

实现一个与 P5 General 对接的驱动（GP2040 角色）时逐项核对：

- [ ] 设备侧（面向 PS5）：按第 2 节字节精确复现设备/配置/HID/报告描述符与字符串表。
- [ ] 设备侧：GET_REPORT(0x03) 返回第 5.2 节 47 字节静态数据；未实现的特征 ID 一律 STALL。
- [ ] 设备侧：SET_REPORT(0xF0) 仅在 wLength=64 且状态机 idle 时接受。
- [ ] 设备侧：GET(F1)/GET(F2) 返回缓冲当前内容并同时武装下一次取回（滞后一次语义）。
- [ ] 设备侧：IN 端点只发加密狗返回的字节；OUT 端点数据丢弃；挂起时尝试远程唤醒。
- [ ] 主机侧（面向加密狗）：VID/PID 0x2B81/0x0101 识别；中断 IN 持续轮询；中断 OUT 原样 64 字节写入。
- [ ] 主机侧：三类 Feature 控制传输按第 5.1 节编码；F0 转发不做任何校验/改写。
- [ ] 状态机：按第 5.6 节迁移表；500 ms F2 延迟；f1_num = 4/1/0。
- [ ] 报告流水线：变化检测 + 4 次重复；单槽 pending/finish；hash_ready 未清空前丢弃新到达的加密狗报告。
- [ ] 失败路径：明确决定是否为 `*_wait` 增加超时（原实现没有，见第 9 节）。

---

## 13. 源码依据清单

P5 直接相关（逐行阅读）：

- `src/drivers/p5general/P5GeneralDriver.cpp` / `headers/drivers/p5general/P5GeneralDriver.h`
- `src/drivers/p5general/P5GeneralAuth.cpp` / `headers/drivers/p5general/P5GeneralAuth.h`
- `src/drivers/p5general/P5GeneralAuthUSBListener.cpp` / `headers/drivers/p5general/P5GeneralAuthUSBListener.h`
- `headers/drivers/p5general/P5GeneralDescriptors.h`（278 行描述符逐字节核对）
- `src/hosts/DualsensePS5Host.cpp` / `headers/hosts/DualsensePS5Host.h`
- `src/drivermanager.cpp`

支撑框架：

- `headers/gpdriver.h`、`headers/usblistener.h`、`headers/drivers/shared/gpauthdriver.h`、`headers/gphost.h`
- `src/usbdriver.cpp`（tud_* 回调 → 驱动路由）、`src/usbhostmanager.cpp`（tuh_* 回调 → 监听器分发）
- `src/gp2040.cpp`（core0 主循环）、`src/gp2040aux.cpp`（core1 辅循环与监听器注册）
- `src/addons/gamepad_usb_host_listener.cpp`（DualSense 主机路径挂接）、`headers/addons/gamepad_usb_host_listener.h`（轮询周期 3 ms）
- `headers/drivers/ps4/PS4Descriptors.h`（PSSensor/TouchpadData 结构）、`src/drivers/ps4/PS4Driver.cpp`、`src/drivers/ps4/PS4AuthUSBListener.cpp`（协议类比）
- `lib/tinyusb/src/class/hid/hid_device.c`、`hid_device.h`、`hid_host.c`、`hid_host.h`、`hid.h`（控制传输编码与回调语义）
- `headers/tusb_config.h`、`headers/drivers/shared/driverhelper.h`、`src/gamepad.cpp`（getMicro/getMillis）
- `src/peripheralmanager.cpp`、`lib/PicoPeripherals/peripheral_usb.h`（PIO USB 使能与默认配置）
- `src/storagemanager.cpp`、`proto/enums.proto`、`CMakeLists.txt`、`www/src/Data/InputBootModes.ts`、`www/src/Locales/en/SettingsPage.jsx`、`www/src/Locales/zh-CN/SettingsPage.jsx`、`src/addons/neopicoleds.cpp`、`src/display/ui/screens/ButtonLayoutScreen.cpp`、`headers/display/ui/screens/MainMenuScreen.h`

---

## 14. 实测补充（真实硬件）

下列事实来自本仓库在 Jetson Orin + 真 P5 General 加密狗 + PS5 上的实测，直接回答第 10 节的部分未决项。复现路径：`sudo python3 scripts/test/p5g_dongle_probe.py`（加密狗侧）+ 以 `-M p5g` 启动后端并观察控制台（PS5 侧）。

| 项 | 实测结论 | 对应第 10 节条目 |
|---|---|---|
| 加密狗自身的描述符 | **与本文档第 2、3 节的克隆逐字节一致**：报告描述符恰 165 字节（逐字节相同）、配置节 41 字节 wTotalLength=0x29、bMaxPower=0xFA、bcdHID=1.11、端点 0x82 IN 64B bInterval=1 与 0x01 OUT 64B | §10.6 |
| 内核绑定 | 枚举后由 `hid-generic` 绑定 → `hidraw` 节点 + 一个 evdev 摇杆节点（主机实现须按 VID/PID 把该 evdev 节点排除出人手输入通道） | §10.6 |
| Feature 0x03 应答 | 48 字节，前 47 字节与本文档第 5.2 节的静态表**逐字节相同**，第 48 字节为 0 | §10.8 |
| Feature 0xF1 / 0xF2（认证前） | 通路成立；净荷全零（未开始认证，与"开机后 auth_buffer 全零"一致） | §10.4 |
| OUT → IN 关系 | **严格 1:1**：每份写入中断 OUT 的 64 字节报告引回恰好一份 64 字节 IN 报告；加密狗**从不自发推送**（静止 3 秒零报告），故空闲时线上静默由协议两侧共同保证 | §10.7 |
| 往返时延 | 均值 2.8 ms、最快 1.5 ms（8/8 应答）⇒ 持续输入下的报告率上界 ≈356 Hz | §10.7 |
| 加密狗回填的字段 | 末 8 字节 `hash` 被填写；**`auth_seq_number`（字节 12–15）也被加密狗改写**（每次往返不同）——本文档第 4 节第 7 字节 `reportCounter` 恒 0 的形态下，单调性由加密狗在返回时完成，符合 §10.9 的推断方向 | §10.2、§10.9 |
| PS5 侧流程 | 枚举成功 → 读 `GET_REPORT(0x03)` → **认证握手在首份实体手柄输入（按 PS 键）后才发起**：4 轮类型 0x01 质询（f1 配额 4）+ 1 轮类型 0x02 + F2 签名状态取回 → 手柄分配、输入直达主机 | §10.5 |
| 一句话结论 | 加密狗是纯应答式的签名黑盒：内容以本文档第 4 节的报告形态提交，返回的报告即线上应发内容 | §10.1–§10.3 |
