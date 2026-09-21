# aimbot WebUI

从局域网任意设备的浏览器管理 Jetson 上的 aimbot —— 启动/停止、参数编辑（含热参数即时下发）、
模型转换与编译、实时日志、温度/帧率监控。**UI 是编排者不是替代者**：SSH 手跑 game 脚本、
`compile.sh` / `convert.sh` / `setup_mouse.sh`、aimbot 命令行接口全部保持原样，WebUI 只按既有
约定编排它们。

```
浏览器 (PC / 手机, 局域网)
   │ http://<jetson-ip>/  (密码登录; token 直达/兜底)
   ▼
webui/server.py (FastAPI, systemd root 服务)
   ├─ 结构化发现: scripts/game/*.sh · engine/ · onnx/ · /dev/v4l/by-id/
   ├─ 实例管理: jetson_clocks → setup_mouse.sh → bin/aimbot (与 game 脚本完全同构)
   ├─ 热参数: UDP 127.0.0.1:47700 → 运行中的 aimbot 即时生效
   └─ 运维任务: convert.sh / compile.sh 异步执行, 日志流回传
```

## 输出模式与真机拓扑

三个模式互斥（`-M`），各自独占 UDC，一次运行只居其一 —— 页面的「输出模式」下拉写进 profile 的
`OUTPUT_MODE`，与脚本、CLI 是同一条语义：

| 模式 | 对端 | 本机呈现 | 读什么 | 设备选择 VAR |
|---|---|---|---|---|
| `hid` | PC 游戏 | 通用 USB 鼠标 | 鼠标 | `MOUSE_KEYWORD`（`-D`；空 = 任意 `*-event-mouse` 取字典序首个） |
| `pad` | PC（XInput） | 微软有线 360 手柄 `0x045E/0x028E` | 手柄 | `PAD_KEYWORD`（`-P`；空 = 任意 `*-event-joystick`） |
| `p5g` | PS5 | P5 General 手柄形态 `0x2B81/0x0101` | 手柄 | 同上，另需真加密狗插本机 USB 口签名 |

- 手柄模式（`pad`/`p5g`）共用整条输入/合并/标定链：物理手柄按键/摇杆/扳机模拟量 1:1 透传，
  控制律期望速度按逐轴有效满偏屏速折算成右摇杆注入并与人类通道径向合并。`PAD_TRIG_THR`（`-T`，
  热参 `padthr`）只门控「扳机算不算按下」这一个判定，模拟量本身不过阈值也不 reshape；
  `PAD_DUMP=y` 加 `--pad-dump`，把合并后的逻辑态打进日志（诊断注入用）。
- hid 不碰手柄，pad/p5g 不读鼠标（手柄插着时它的辅助鼠标接口也会匹配 `-D` 的空模式，
  要人手通道干净就在 `MOUSE_KEYWORD` 里点名真鼠标）。**换模式 = 改这一项后重新【启动】** ——
  它决定整条输出后端，是冷参数。
- 标定回写按输出模式分流：hid 写 `HID_L_EST`，pad 写 `PAD_L_EST`，p5g 写 `P5G_L_EST` —— 三格
  延迟互不覆盖；`-l` 取本模式那一格（三格都在脚本里，只有本模式那一格被消费）。
- **每套输出各占一组 5 个值**（延迟 1 + 拉枪倍率 4）：参数页按模式分组呈现，切【输出模式】就是
  整套换（显示的、编辑的、保存的都是那一槽），标定也只落进本槽。分槽是必须的 —— 同一个倍率数
  在两套口径下含义不同（见下一节），延迟也各是自己的物理量（p5g 的回路还含一跳加密狗签名往返）。

## 部署（Jetson）

```bash
# 1. 依赖 (需要网络; 离线见下节)
cd <deploy-root>/webui
sudo python3 -m pip install -r requirements.txt

# 2. 安装并启动 systemd 服务 (生成 unit、开机自启)
sudo bash deploy/install.sh

# 3. 打开
#    token 打印在服务日志里 (首次设置密码要用):
sudo journalctl -u aimbot-webui -n 20 --no-pager
#    浏览器访问  http://<jetson-ip>/  → 首次凭 token 设置登录密码, 之后密码登录
```

手动试跑（不装服务）：`cd webui && sudo python3 server.py`。

要求：JetPack 自带的 python3 (≥3.8) 即可，无需 venv（这是服务端，不是 arena）。

### 更新（已装服务的机器）

覆盖 `webui/` 下的代码（`app/` `static/` `server.py`；**保留 `webui/data/`** —— token、密码、
profile 参数、历史都在里面），然后 `sudo systemctl restart aimbot-webui` 即可。
页面与静态资源带 `Cache-Control: no-cache`，浏览器普通刷新就是最新界面，不用清缓存。
依赖没变（requirements.txt 未改）时不用重装。

### 离线安装

在有网的 x86 机器上下载 aarch64 轮子后拷到 Jetson：

```bash
pip download -r requirements.txt \
    --platform manylinux2014_aarch64 --only-binary=:all: -d wheels/
# Jetson 上:
sudo python3 -m pip install --no-index --find-links wheels/ -r requirements.txt
```

## 权限模型（为什么服务以 root 跑）

现有流程里，`jetson_clocks`、USB 鼠标通道准备（`setup_mouse.sh`：载入 raw_gadget、腾空 UDC、
`/dev/raw-gadget` 权限）、采集卡节点全部在 `sudo` 之下 —— 手动 SSH 跑 game 脚本就是 root 环境。
WebUI 要 1:1 复刻该环境，只有两条路：

1. **服务直接以 root 跑（采用）**：零 sudoers 维护；部署目录改名/移动后不需要任何额外配置；
   行为与手动跑完全一致。
2. 替代方案（更小权限面）：服务跑普通用户，sudoers 里只放行两条固定路径 ——
   `<部署根>/scripts/setup_mouse.sh` 与 `jetson_clocks`（NOPASSWD）。**aimbot 本身不加 sudo
   前缀**（它和手动跑一样缺不了 /dev/input 的读权限，见下），所以服务用户必须属于 `input`
   组，否则手柄/鼠标节点一律读不到（实测：不在 input 组时鼠标节点报"不可读"、`-D` 直接失败）。
   代价：换部署根要同步改 sudoers；`/dev/raw-gadget` 权限由 setup_mouse.sh 每次 launch
   `chmod 666` 给出，通常没问题。这正是服务以 root 跑最省事的理由。

暴露面 = 局域网 + 密码门槛（token 兜底）。不需要暴露时，在设置页把监听改成 `127.0.0.1`。

## 安全（极简）

- **密码登录为主**：首次打开页面要求凭 token 设置密码（证明你有权配置这台 Jetson），
  之后日常输密码即可。密码只存 PBKDF2-SHA256 哈希（`webui/data/config.json`，纯标准库）。
- **token 保留为万能凭证**：登录成功后服务端把 token 发给浏览器存 localStorage，
  之后所有 API 仍走 `X-WebUI-Token` 头（WS 走 `?token=`），鉴权管道只有一条。token 的用途：
  ① URL `?token=xxx` 直达；② 忘记密码时登录（登录后在设置页改密码）；③ 首次设密码。
  token 首次启动自动生成，打印在启动横幅；设置页可显示/复制/一键重置（重置不影响密码）。
- 除首页/静态资源/登录端点外所有 API 要求 token；登录/设密失败有固定 1s 延迟。
- 热参数通道只绑 `127.0.0.1`，局域网摸不到。
- 没有更多了 —— 这是单用户局域网工具，不上重型认证（明文 HTTP 传输与 token 时代一致）。

## 配置发现模型

- **部署根**在设置页指定（默认 = webui/ 的上一层），一切发现按目录结构确定性推导，不递归扫全盘：
  - 游戏 profile = `<根>/scripts/game/*.sh`（只读解析顶部 `VAR=value` 块）；
  - 模型 = `<根>/engine/**/*.engine`；转换源 = `<根>/onnx/**/*.onnx`；
  - 采集卡 = `/dev/v4l/by-id/*-video-index0`，`Hagibis`/`Asus` 别名按固件
    `resolve_cam_device` 同规则（小写子串、唯一命中）测试后才进下拉；
  - 采集输出目录默认建议 `dataset/<游戏名>/`。
- **模型/采集卡/输出目录都是下拉**（发现结果里选），设备匹配子串与数值项是输入框；
  部署根本身也是输入框。
- 参数面按组落在脚本 VAR 上（每组一个 VAR，`app/discover.py` 的 `SCRIPT_VARS` 是唯一映射表）：
  - **输出模式与设备** = `OUTPUT_MODE` / `MOUSE_KEYWORD` / `PAD_KEYWORD` / `PAD_TRIG_THR` / `PAD_DUMP`
    （见上一节；`-D`/`-P`/`-T`/`--pad-dump` 的来源）；
  - **延迟与拉枪倍率 — hid / pad / p5g**（三组，每组 5 个值，只有当前输出模式那一组可见）：
    `HID_L_EST` / `PAD_L_EST` / `P5G_L_EST`（环路延迟 ms，标定回写目标，也可手填；固件带宽
    `wn` 由它导出，所以它是唯一进运行态的标定量）与 `<模式>_SPDX` / `_SPDY` / `_ADS_SPDX` /
    `_ADS_SPDY`（四个逐轴整数倍率，`100` = 基线、越大越快、有意义带 `5..2000`，热参
    `spdx`/`spdy`/`adsspdx`/`adsspdy`）。倍率的口径按槽不同：hid 槽基线 `1.0 px/count`，
    pad/p5g 槽基线 `满偏屏速 3000 px/s` —— 所以两套的数值不可直接互换，换算才是分槽的理由。
- 标定值（环路延迟）**永远以脚本为权威**：每次启动现场解析；`-S` 指向 profile 脚本本身，
  固件标定回写照旧落进脚本 —— SSH 侧与 UI 侧看到同一份标定。三个输出模式各有自己的延迟 VAR
  （`HID_L_EST` / `PAD_L_EST` / `P5G_L_EST`），三格独立、互不覆盖；`-l` 与「运行」页卡片显示的
  都是本模式那一格（p5g 未标定过时可借 pad 格的值当起点，标一次就各归各的）。
  触发：hid = 鼠标双侧键长按 5 秒；手柄模式 = **L3+R3 长按 5 秒**，运行中还有 WebUI 侧的入口 ——
  「运行」页的「**开始标定**」按钮（`POST /api/instance/calib`，只在**本 WebUI 启动且固件支持热参的
  pad/p5g 实例**上可点；认领实例不盲发、hid 模式按钮置灰并写明用双侧键），它发的就是热参
  `padcalib=1`（消费一次即清；标定进行中到达的请求记一行丢弃、绝不重入）。
  标定期间固件只做测量，结论看日志：成功 = 点头 + 只回写本输出模式的延迟格（结果随之回显到
  参数页本槽的延迟行与「运行」页卡片）；失败 = 摇头 + 原因，绝不写编造的值（屏幕完全静止/
  无响应/尾迹被停顿截断都会如实失败）。日志里 `[标定]` 行**照常进页面日志流**（逐段读数与结论
  都在这场运行里），只有 60s 一行的报告率观测（`[USB-HID]`/`[PAD-USB]`）与 `[SAVE]`/`[AI FPS]`
  被页面流隐藏。
- **脚本 = 唯一事实源**: profile 参数就是 game 脚本头部的 `VAR=value` 块, 每次扫描现场解析,
  没有独立参数存储。脚本把手改量写成带守卫的默认值 (`CLASS_ID="${CLASS_ID:-0}"`, 模板约定),
  少写任何一行也能起 —— WebUI 读到的是守卫里的**有效值** (不是字面量), 保存时把值写回同一行。
  网页【保存】= 原子写回脚本:
  - 提交体是**补丁**: 只校验并写回它点名的键, 没提到的 VAR 保持脚本现值 (不会把其余项打回默认);
    UI 侧只提交**本输出槽**的改动 (外加本次真的改过的其它项) —— 切模式后原样保存, 另两槽的
    10 个 VAR 逐字不动; 另两槽未保存的编辑切槽时会提示不会被提交;
  - 只改目标变量的值 —— 行内注释、引号风格、其余每一行逐字保留, 执行位不变;
    原行是 `${VAR:-…}` 守卫的照原样保回 (新值落在默认位上), 路径类值往返写成 `$ROOT/<相对>`;
  - 手上没有值的项 (未选模型等) 不写 —— 绝不把用户的模型路径抹成空串;
  - 三个延迟格 (`HID_L_EST`/`PAD_L_EST`/`P5G_L_EST`) 既是可编辑参数, 也是固件标定回写的目标
    (一回事: 解析出来就是那一格的值); `<模式>_SPDX`/`SPDY`/`ADS_SPDX`/`ADS_SPDY` 四对倍率是
    手动项, WebUI 可改可热发, 固件不写。
  SSH 改脚本 → 打开的页面 ~5 秒自动跟随（表单有未保存改动时不覆盖 —— 后保存者胜）。
- **复制 profile** = 完整复制脚本文件（文件名按弹窗输入, 参数随源）。

## 实例管理与启动语义

- 单实例不变量。状态机 `stopped → starting → running → exited`（退出码/信号/异常标记可查）。
- **【启动】= 保存当前设置 → 清掉所有在跑实例（`/proc/*/exe` 精确匹配 `<根>/bin/aimbot`，
  含 SSH 手跑的；SIGTERM → 5s 超时 SIGKILL）→ `jetson_clocks` → `setup_mouse.sh` → aimbot**。
  `jetson_clocks` 失败只警示继续（与脚本行为一致）；`setup_mouse.sh` 失败中止并把 stderr 显示出来。
  “重启” = 再点一次【启动】。
- **【保存】**只持久化。有未保存改动时按钮高亮、切页/切 profile 提示。
- **孤儿认领**：WebUI 自身重启时扫描已存活的 aimbot 并认领为 running（认领实例无日志，
  可停止/接管），绝不把存活实例显示成“已停止”诱导双开。
- 启动序列逐步显示 ✓/✗/耗时；`preview=y` 且未接屏时固件报错会如实显示在日志里，改回即可。

## 热参数通道（固件侧实现见 `src/io/hotctl.cu` 的 `hotctl_thread`）

- 协议：UDP `127.0.0.1:47700`（固件头部常量 `HOT_CTL_PORT`），数据报 UTF-8 `key=value;key=value`。
- 白名单（固件侧强制钳制，不信任发送方）：

| key | 含义 | 对应 CLI | 钳制 |
|---|---|---|---|
| `t` | 置信度阈值 | `-t` | 0–1 |
| `y` | 瞄准高度偏移 % | `-y` | 0–100 |
| `x` | 速度上限 px/s | `-x` | 100–20000 |
| `spdx` / `spdy` | 拉枪速度倍率, 逐轴 (100 = 基线, 与有效灵敏度成反比); 作用于运行中实例**当前输出模式**那一槽 | `--spd` | 1–10000 |
| `adsspdx` / `adsspdy` | ADS 键 (右键 / LT) 按住时的同一对 | `--ads-spd` | 1–10000 |
| `fov` | FOV 半径 px | `-r` | 10–1000 |
| `padthr` | 手柄触发阈值 % 满量程 (RT/LT 两键共享; 只门控自瞄触发判定, 扳机模拟量仍 1:1 透传) | `-T` | 0–100 |
| `k` | 触发键模式 | `-k` | fire / ads / both |
| `aim` | 鼠标接管 (0=纯透传, 不注入移动; 检测/采集照常) | `-a` | 0 / 1 |
| `cap_fire` | 开火截图源开关 | `-e` | 0 / 1 |
| `cap_det` | 检测截图源开关 | `-e` | 0 / 1 |
| `cap_auto` | 定时截图源开关 | `-e` | 0 / 1 |

- 每次应用固件打印 `[热参] t=0.55`（UI 从日志流看到回执）；未知 key 忽略；启动时打印
  `✅ 热参数通道: 127.0.0.1:47700 (t/y/x/fov/padthr/spdx/spdy/adsspdx/adsspdy/k/aim/cap_*)` ——
  WebUI 以该行为准判断固件能力，旧二进制（无此行/二进制探测失败）热参 UI 置灰，**不盲发 UDP**。
  倍率热参只有一套：保存时按**运行中实例的模式**取槽（实例跑 pad 时改 pad 槽 → 发 `spdx`；
  改 hid/p5g 槽 → 只是脚本改动，下次【启动】生效）；改 `output_mode`（冷参数）时本次不外发。
- 结构性常量（PM/ζ/I gate/CUSUM…）是编译期原则常量，**不进白名单**；白名单里的是人手输入（延迟与四个速度倍率），律本身没有可调常数。
- UI 上 🔥 = 热参数（保存即下发运行中实例），❄ = 冷参数（下次【启动】生效）。

## 目录

```
webui/
├── server.py            入口 (python3 server.py)
├── app/                 config / discover / proc / tasks / telemetry / api
├── static/              index.html + app.js + style.css (自托管, 零外链, 双主题, 手机可用)
├── deploy/install.sh    生成 systemd unit 并启用 (路径从脚本自身解析)
├── requirements.txt     fastapi / uvicorn / websockets (纯 Python 轮子)
└── data/                运行时状态: config.json · history.json (gitignore)
```

注意：服务必须单进程运行（systemd unit 就是单 worker），实例/任务管理是进程内单例。

## 验收清单（Jetson 上）

1. `sudo bash webui/deploy/install.sh` → `systemctl status aimbot-webui` 正常；
2. `journalctl -u aimbot-webui` 里拿到 token，浏览器打开 → 首次凭 token 设置密码 →
   退出重进用密码登录；「改用 token 登录」也能进；错误密码会被拒；
3. 设置页确认部署根正确、扫描摘要数量符合（profile/engine/onnx/采集卡）；
4. 参数页改动置信度 → 【保存】→【启动】：步骤条三步全绿，日志出现
   `✅ 热参数通道: …`（`[AI FPS]` 与 `[SAVE]` 不再出现在页面日志流,
   FPS 显示在卡片、读数进「AI FPS 历史」面板、截图计数进「截图 本次」卡片;
   「模型架构」「输入尺寸」两卡片从启动日志的 `模型:` 行抓取, 每次启动刷新）;
5. 运行中再改置信度 →【保存】：toast 显示已下发，日志出现 `[热参] t=…`；
6. 【停止】→ 状态“已退出”；SSH 手动跑 game 脚本 → WebUI 显示“认领”而非“已停止”
   （认领实例「截图 本次」显示 —，「截图 文件夹」照常计数）；
7. 按【启动】清掉 SSH 实例并以 UI 设置接管；
8. convert / compile 任务能跑完且有日志/退出码；compile 在实例运行中时被拒绝并说明原因；
9. 手机浏览器（同局域网）打开：布局单列、按钮可点、日志可滚；
10. 复制 profile → 新脚本出现在 `scripts/game/`（内容与源一致），SSH 直接跑它照常工作；
11. 「截图 文件夹」与输出目录联动: 改 profile 的输出目录 (含绝对路径) → ~10s 后卡片路径跟随变化;
    重新【启动】→「截图 本次」归零、「AI FPS 历史」清空;
12. 保存写回脚本: 网页改 Y_OFFSET/采集开关 →【保存】→ `cat` 脚本对应行已变且注释原样;
    SSH 改脚本 Y_OFFSET → ~5s 后网页表单跟随（无未保存改动时）;
    启动实例运行中改置信度 →【保存】→ 日志出现 `[热参] t=…`（差量自动下发）。
13. **输出模式与三套槽**: 参数页「输出模式」选 pad/p5g → 出现「手柄匹配子串 -P / 手柄触发阈值 -T /
    合并态调试输出 --pad-dump」，hid → 只出现「鼠标匹配子串 -D」；「延迟与拉枪倍率」三组里只有
    当前模式那一组显示（5 行：延迟 + 四个倍率）；【保存】后脚本对应行变化，另两槽的 10 个 VAR
    逐字不动（`diff` 只该出现本槽的行）；「运行」页的「启动命令预览」里随之出现 `-M pad -P … -T 6`
    （hid 是 `-D …`），`-l` 与四个倍率都换成新槽的值；改「延迟 L」→【保存】→ 脚本本槽的延迟行变化
    （守卫写法保留），SSH 跑脚本时 `-l` 跟着变。
14. **标定按钮**: 跑着 pad/p5g 实例时「运行」页的「开始标定」可点 → 日志出现
    `[热参] padcalib=1 (请求手柄标定)` 与随后的 `[标定] …` 行，成功时 `[标定] 已回写 … (PAD_L_EST /
    P5G_L_EST)` 并回显到本槽的延迟行；hid 模式下按钮置灰并写明触发方式是鼠标双侧键长按。

## 已知边界

- 本仓库的 Windows 开发镜像只用于改代码；WebUI 的运行行为（绑定 80、/proc、udev、tegrastats）
  以 Jetson 实机验收为准。
- 视频预览（P2）未做：采集卡 NV12 1080p120 已占 ~3Gbps USB，二次 v4l2 打开会 EBUSY/抢帧；
  将来做应走固件内抽帧，而不是再开一路采集。
- GPU 占用依赖 `tegrastats`（Jetson 自带），读不到时 UI 隐藏该卡片。
