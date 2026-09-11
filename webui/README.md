# aimbot WebUI

从局域网任意设备的浏览器管理 Jetson 上的 aimbot —— 启动/停止、参数编辑（含热参数即时下发）、
模型转换与编译、实时日志、温度/帧率监控。**UI 是编排者不是替代者**：SSH 手跑 game 脚本、
`compile.sh` / `convert.sh` / `setup_mouse.sh`、aimbot 命令行接口全部保持原样，WebUI 只按既有
约定编排它们。

```
浏览器 (PC / 手机, 局域网)
   │ http://<jetson-ip>/?token=…
   ▼
webui/server.py (FastAPI, systemd root 服务)
   ├─ 结构化发现: scripts/game/*.sh · engine/ · onnx/ · /dev/v4l/by-id/
   ├─ 实例管理: jetson_clocks → setup_mouse.sh → bin/aimbot (与 game 脚本完全同构)
   ├─ 热参数: UDP 127.0.0.1:47700 → 运行中的 aimbot 即时生效
   └─ 运维任务: convert.sh / compile.sh 异步执行, 日志流回传
```

## 部署（Jetson）

```bash
# 1. 依赖 (需要网络; 离线见下节)
cd /mnt/TF/aimbot/webui
sudo python3 -m pip install -r requirements.txt

# 2. 安装并启动 systemd 服务 (生成 unit、开机自启)
sudo bash deploy/install.sh

# 3. 打开
#    token 打印在服务日志里:
sudo journalctl -u aimbot-webui -n 20 --no-pager
#    浏览器访问  http://<jetson-ip>/?token=<token>
```

手动试跑（不装服务）：`cd webui && sudo python3 server.py`。

要求：JetPack 自带的 python3 (≥3.8) 即可，无需 venv（这是服务端，不是 arena）。

### 离线安装

在有网的 x86 机器上下载 aarch64 轮子后拷到 Jetson：

```bash
pip download -r requirements.txt \
    --platform manylinux2014_aarch64 --only-binary=:all: -d wheels/
# Jetson 上:
sudo python3 -m pip install --no-index --find-links wheels/ -r requirements.txt
```

## 权限模型（为什么服务以 root 跑）

现有流程里，`jetson_clocks`、USB gadget 配置（写 `/sys/kernel/config/usb_gadget`）、
`/dev/hidg0`、采集卡节点全部在 `sudo` 之下 —— 手动 SSH 跑 game 脚本就是 root 环境。
WebUI 要 1:1 复刻该环境，只有两条路：

1. **服务直接以 root 跑（采用）**：零 sudoers 维护；部署目录改名/移动后不需要任何额外配置；
   行为与手动跑完全一致。
2. 替代方案（更小权限面）：服务跑普通用户，sudoers 里只放行三条固定路径 ——
   `<部署根>/scripts/setup_mouse.sh`、`jetson_clocks`、`<部署根>/bin/aimbot`（NOPASSWD）。
   代价：换部署根要同步改 sudoers；aimbot 直接 exec 时若 `/dev/hidg0` 权限不足仍会失败
   （setup_mouse.sh 每次 launch 都会 `chmod 666`，通常没问题）。

暴露面 = 局域网 + token 门槛。不需要暴露时，在设置页把监听改成 `127.0.0.1`。

## 安全（极简）

- 首次启动自动生成 token（`webui/data/config.json`），打印在启动横幅；页面存 localStorage。
- 除首页/静态资源外所有 API 要求 `X-WebUI-Token` 头（WS 走 `?token=`）；设置页可一键重置。
- 热参数通道只绑 `127.0.0.1`，局域网摸不到。
- 没有更多了 —— 这是单用户局域网工具，不打算上重型认证。

## 配置发现模型

- **部署根**在设置页指定（默认 = webui/ 的上一层），一切发现按目录结构确定性推导，不递归扫全盘：
  - 游戏 profile = `<根>/scripts/game/*.sh`（只读解析顶部 `VAR=value` 块）；
  - 模型 = `<根>/engine/**/*.engine`；转换源 = `<根>/onnx/**/*.onnx`；
  - 采集卡 = `/dev/v4l/by-id/*-video-index0`，`Hagibis`/`Asus` 别名按固件
    `resolve_cam_device` 同规则（小写子串、唯一命中）测试后才进下拉；
  - 采集输出目录默认建议 `dataset/<游戏名>/`。
- **所有路径/设备输入都是下拉**，没有手输路径的入口（部署根本身除外）。
- 标定值 `S_EST`/`L_EST` **永远以脚本为权威**：每次启动现场解析；`-S` 指向 profile 脚本本身，
  固件标定回写照旧落进脚本 —— SSH 侧与 UI 侧看到同一份标定。
- **profile 参数**持久化在 `webui/data/profiles/<脚本名>.json`（含 `display_name`），首次扫描以
  脚本值播种；此后 UI 保存值与脚本值分叉即“漂移”，页面提示并可一键“采用脚本值”。
  脚本脱离 UI 手动 SSH 跑永远照常工作 —— WebUI 从不改写脚本。
- **复制 profile** = 完整复制脚本文件 + 新建 profile JSON（显示名/文件名按弹窗输入，都有默认值）。

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

## 热参数通道（固件侧实现见 `src/aimbot.cu` 的 `hotctl_thread`）

- 协议：UDP `127.0.0.1:47700`（固件头部常量 `HOT_CTL_PORT`），数据报 UTF-8 `key=value;key=value`。
- 白名单（固件侧强制钳制，不信任发送方）：

| key | 含义 | 对应 CLI | 钳制 |
|---|---|---|---|
| `t` | 置信度阈值 | `-t` | 0–1 |
| `y` | 瞄准高度偏移 % | `-y` | 0–100 |
| `x` | 速度上限 px/s | `-x` | 100–20000 |
| `fov` | FOV 半径 px | `-r` | 10–1000 |
| `k` | 触发键模式 | `-k` | fire / ads / both |

- 每次应用固件打印 `[热参] t=0.55`（UI 从日志流看到回执）；未知 key 忽略；启动时打印
  `✅ 热参数通道: 127.0.0.1:47700 (t/y/x/fov/k)` —— WebUI 以该行为准判断固件能力，
  旧二进制（无此行/二进制探测失败）热参 UI 置灰，**不盲发 UDP**。
- 结构性常量（PM/ζ/I gate/CUSUM…）是编译期原则常量，**不进白名单**，与“无手调魔法数字”哲学一致。
- UI 上 🔥 = 热参数（保存即下发运行中实例），❄ = 冷参数（下次【启动】生效）。

## 目录

```
webui/
├── server.py            入口 (python3 server.py)
├── app/                 config / discover / proc / tasks / telemetry / api
├── static/              index.html + app.js + style.css (自托管, 零外链, 双主题, 手机可用)
├── deploy/install.sh    生成 systemd unit 并启用 (路径从脚本自身解析)
├── requirements.txt     fastapi / uvicorn / websockets (纯 Python 轮子)
└── data/                运行时状态: config.json · profiles/*.json · history.json (gitignore)
```

注意：服务必须单进程运行（systemd unit 就是单 worker），实例/任务管理是进程内单例。

## 验收清单（Jetson 上）

1. `sudo bash webui/deploy/install.sh` → `systemctl status aimbot-webui` 正常；
2. `journalctl -u aimbot-webui` 里拿到 token，PC 浏览器打开并登录；
3. 设置页确认部署根正确、扫描摘要数量符合（profile/engine/onnx/采集卡）；
4. 参数页改动置信度 → 【保存】→【启动】：步骤条三步全绿，日志出现
   `✅ 热参数通道: …` 与 `[AI FPS]`；
5. 运行中再改置信度 →【保存】：toast 显示已下发，日志出现 `[热参] t=…`；
6. 【停止】→ 状态“已退出”；SSH 手动跑 game 脚本 → WebUI 显示“认领”而非“已停止”；
7. 按【启动】清掉 SSH 实例并以 UI 设置接管；
8. convert / compile 任务能跑完且有日志/退出码；compile 在实例运行中时被拒绝并说明原因；
9. 手机浏览器（同局域网）打开：布局单列、按钮可点、日志可滚；
10. 复制 profile → 新脚本出现在 `scripts/game/`（内容与源一致 + JSON 元数据），SSH 直接跑它照常工作。

## 已知边界

- 本仓库的 Windows 开发镜像只用于改代码；WebUI 的运行行为（绑定 80、/proc、udev、tegrastats）
  以 Jetson 实机验收为准。
- 视频预览（P2）未做：采集卡 NV12 1080p120 已占 ~3Gbps USB，二次 v4l2 打开会 EBUSY/抢帧；
  将来做应走固件内抽帧，而不是再开一路采集。
- GPU 占用依赖 `tegrastats`（Jetson 自带），读不到时 UI 隐藏该卡片。
