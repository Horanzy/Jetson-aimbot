/* aimbot WebUI 前端 (无构建, 原生 JS)。
   数据面: 启动时 GET /api/state 全量, 之后单条 WebSocket 增量 (日志按 seq, FPS/扫描按游标, 遥测 2s)。
   认证面: 日常密码登录 (首次凭 token 设密码), 登录换回 token 后与旧管道完全一致。
   参数面: 脚本 = 唯一事实源 —— 表单从脚本解析值播种, 【保存】写回脚本后回读刷新;
   SSH 改脚本经 WS 的 scan 推送自动跟随 (编辑中的表单不被覆盖)。 */
"use strict";

/* ================= 工具 ================= */
const $ = s => document.querySelector(s);
const $$ = s => Array.from(document.querySelectorAll(s));
const esc = s => String(s == null ? "" : s).replace(/[&<>"']/g,
  c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

function toast(msg, kind) {
  const el = document.createElement("div");
  el.className = "toast" + (kind ? " " + kind : "");
  el.textContent = msg;
  $("#toasts").appendChild(el);
  setTimeout(() => el.remove(), 6000);
}

function fmtSize(n) {
  if (n == null) return "—";
  if (n > (1 << 30)) return (n / (1 << 30)).toFixed(1) + " GB";
  if (n > (1 << 20)) return (n / (1 << 20)).toFixed(1) + " MB";
  return (n / (1 << 10)).toFixed(0) + " KB";
}
function fmtDur(sec) {
  if (sec == null) return "—";
  sec = Math.max(0, Math.floor(sec));
  const h = Math.floor(sec / 3600), m = Math.floor((sec % 3600) / 60), s = sec % 60;
  const mm = String(m).padStart(2, "0"), ss = String(s).padStart(2, "0");
  return h > 0 ? h + ":" + mm + ":" + ss : mm + ":" + ss;
}
function fmtClock(ts) {
  if (ts == null) return "—";
  const d = new Date(ts * 1000);
  const p = x => String(x).padStart(2, "0");
  return p(d.getMonth() + 1) + "-" + p(d.getDate()) + " " + p(d.getHours()) + ":" + p(d.getMinutes()) + ":" + p(d.getSeconds());
}
function fmtDate(ts) {
  if (ts == null) return "—";
  const d = new Date(ts * 1000);
  const p = x => String(x).padStart(2, "0");
  return d.getFullYear() + "-" + p(d.getMonth() + 1) + "-" + p(d.getDate()) + " " + p(d.getHours()) + ":" + p(d.getMinutes());
}
function sameVal(a, b) {
  if (typeof a === "number" && typeof b === "number") return Math.abs(a - b) < 1e-6;
  return a === b;
}

/* ================= 全局状态 ================= */
const S = {
  token: localStorage.getItem("wb_token") || "",
  state: null,
  selected: localStorage.getItem("wb_profile") || "",
  params: null, savedParams: null,
  dirty: false,
  ws: null, wsSeq: 0, wsBackoff: 1000,
  logBuf: [], follow: true,
  fpsHist: [],                        // 本次运行 [AI FPS] 读数 [[ts, fps]] (会话级, 重启清空)
  capFolder: null, capFolderName: "", capDir: "",
  theme: localStorage.getItem("wb_theme") || "dark",
  taskLogs: {}, taskOpen: {},
  openInfo: new Set(),                // 已展开 ⓘ 的参数 key (跨重渲染保持)
};

const RUNNING = ["starting", "running", "stopping"];

async function api(path, opts) {
  opts = opts || {};
  opts.headers = Object.assign({ "X-WebUI-Token": S.token }, opts.headers || {});
  if (opts.body && typeof opts.body !== "string") {
    opts.body = JSON.stringify(opts.body);
    opts.headers["Content-Type"] = "application/json";
  }
  const r = await fetch(path, opts);
  if (r.status === 401) {
    if ($("#tokenModal").classList.contains("hidden")) initAuth();
    throw new Error("需要登录");
  }
  if (!r.ok) {
    let msg = "HTTP " + r.status;
    try { msg = (await r.json()).detail || msg; } catch (e) { /* keep */ }
    throw new Error(msg);
  }
  return r.json();
}

/* ================= 参数定义 (ⓘ 文案与热/冷标记) ================= */
/* ---- 三套输出各占一组 5 个值 (延迟 1 + 拉枪倍率 4) ----
   切换【输出模式】= 显示/编辑另一槽的这 5 个值; 保存只提交本槽 (savePatch), 标定回写也只
   落进本槽 (固件的 HID_L_EST / PAD_L_EST / P5G_L_EST 三选一)。分槽是必须的: 同一个倍率数
   在两套口径下含义不同 (下面的 base), 延迟也各是自己的物理量 (p5g 还多一跳加密狗签名往返)。 */
const MODE_SLOT = [
  { mode: "hid", title: "hid — USB 鼠标 → PC", lvar: "HID_L_EST",
    base: "基线 = 1.0 px/count (counts 直接进游戏, 游戏内鼠标灵敏度是另一半)" },
  { mode: "pad", title: "pad — XInput 手柄 → PC", lvar: "PAD_L_EST",
    base: "基线 = 满偏屏速 3000 px/s (控制律的注入按它折算摇杆偏转)" },
  { mode: "p5g", title: "p5g — P5 General → PS5", lvar: "P5G_L_EST",
    base: "基线 = 满偏屏速 3000 px/s (与 pad 同一条注入/合并链, 线上那端是 PS5)" },
];
const SLOT_SPD = [
  { a: "spd_x", label: "速度倍率 X — 腰射", hot: "spdx",
    info: "拉枪速度倍率, 与有效灵敏度成反比 (有效灵敏度 = 基线/(倍率/100)): 调大 = 同样的期望屏幕速度发出更多 counts / 更大的摇杆偏转 = 屏幕跟得更快。整数步进 (105 / 109), 100 = 基线; 有意义的带是 5..2000, 越界会被固件夹取。本槽的基线见组标题。" },
  { a: "spd_y", label: "速度倍率 Y — 腰射", hot: "spdy",
    info: "Y 轴同一刻度 (100 = 基线)。与 X 分开是因为同一灵敏度下垂直与水平的屏速比是游戏属性 —— 俯仰灵敏度常更低, 于是这里通常要给更大的值; 一个总倍率会把两轴绑死。" },
  { a: "ads_spd_x", label: "速度倍率 X — ADS", hot: "adsspdx",
    info: "ADS 键 (右键 / LT) 按住期间的 X 倍率, 同一刻度 (100 = 基线)。按住的那一拍整套切换, 与腰射那对互不影响; 开镜灵敏度常设得更低, 故这里通常比腰射更大。" },
  { a: "ads_spd_y", label: "速度倍率 Y — ADS", hot: "adsspdy",
    info: "ADS 键按住期间的 Y 倍率, 同一刻度 (100 = 基线)。" },
];
const MODE_SLOT_KEYS = {};
function modeSlotRows() {
  const out = [];
  MODE_SLOT.forEach(sl => {
    const group = "延迟与拉枪倍率 — " + sl.title;
    MODE_SLOT_KEYS[sl.mode] = new Set(["l_" + sl.mode].concat(
      SLOT_SPD.map(sp => sl.mode + "_" + sp.a)));
    out.push({ group: group, key: "l_" + sl.mode, label: "延迟 L (" + sl.lvar + ")",
      type: "num", min: 0, max: 120, step: 0.1, unit: "ms", hot: false, showIfMode: [sl.mode],
      info: "本输出模式的环路延迟 ms —— 标定成功就写进这一格, 也可手填。控制律带宽由它导出 (wn=(90°−PM)π/180/L), 所以它是唯一进运行态的标定量, 越准越好; 另两套输出的延迟各有自己的槽, 标定与保存都不会碰。缺行/缺值时固件按 60 兜底。❄ 冷参数 (下次【启动】生效)。" });
    SLOT_SPD.forEach(sp => out.push({ group: group, key: sl.mode + "_" + sp.a,
      label: sp.label, type: "num", min: 1, max: 10000, step: 1, hot: true, showIfMode: [sl.mode],
      info: sp.info + " 本槽 " + sl.base + "。🔥 热参数 (" + sp.hot + "), 当前实例跑的就是本模式时保存即生效。" }));
  });
  return out;
}

const PARAM_DEFS = [
  { group: "输出模式与设备", key: "output_mode", label: "输出模式", type: "select", options: ["hid", "pad", "p5g"], hot: false,
    labels: { hid: "hid — USB 鼠标 → PC", pad: "pad — XInput 手柄 → PC", p5g: "p5g — P5 General → PS5" },
    info: "输出通道, 三选一互斥, 各自独占 UDC。hid: 通用 USB 鼠标, 游戏内鼠标灵敏度生效 (读鼠标, 用 -D 选设备)。pad: 物理手柄全透传 + 控制律注入右摇杆, 本机呈现微软有线 360 手柄 (0x045E/0x028E), 宿主 XInput 直接认。p5g: 同一输入/合并/标定链面向 PS5, 本机呈现 P5 General 手柄 (0x2B81/0x0101), 且需要真加密狗插在本机 USB 口作签名协处理器 —— 每份报告经它签名后才上线, 缺席时线上不产生任何报告。手柄模式读手柄、不读鼠标; hid 读鼠标、不碰手柄。❄ 冷参数 (它决定整条后端), 下次启动生效。" },
  { group: "输出模式与设备", key: "mouse_keyword", label: "鼠标匹配子串 -D", type: "text", hot: false, showIfMode: ["hid"],
    info: "按 /dev/input/by-id 的子串挑鼠标 (留空 = 任一 *-event-mouse, 取字典序首个)。注意: 手柄插着时它的辅助鼠标接口也匹配空模式, 想让人手通道干净就在这里点名真鼠标。多个命中会被固件判为错误并列出候选。❄ 冷参数, 下次启动生效。" },
  { group: "输出模式与设备", key: "pad_keyword", label: "手柄匹配子串 -P", type: "text", hot: false, showIfMode: ["pad", "p5g"],
    info: "按 /dev/input/by-id 的子串挑手柄 (留空 = 任一 *-event-joystick 节点; P5 General 加密狗自身的节点已被固件排除)。手柄的额外按键可能挂在同一个 USB 设备的第二个接口上, 固件按 VID:PID 找齐兄弟节点一并读取, 所以这里点名手柄本体即可。❄ 冷参数, 下次启动生效。" },
  { group: "输出模式与设备", key: "pad_trig_thr", label: "手柄触发阈值 -T", type: "num", min: 0, max: 100, step: 1, unit: "%", hot: true, showIfMode: ["pad", "p5g"],
    info: "手柄扳机 (RT/LT 两键共享) 的自瞄触发门槛, 按满量程百分比。它只门控「自瞄认为扳机按下了没有」, 再与 -k 的 fire/ads/both 语义一起决定开火/开镜; 扳机模拟量本身 1:1 透传给游戏, 不过阈值也不 reshape。默认 6 = 该手柄扳机实测 flat 15/255 的上取整 (低于它的读数只在噪声里抖)。手柄换型后按新 flat 调。🔥 热参数 (padthr), 保存即生效。" },
  { group: "输出模式与设备", key: "pad_dump", label: "合并态调试输出 --pad-dump", type: "bool", hot: false, showIfMode: ["pad", "p5g"],
    info: "开启后每 ≥50ms 在日志打一行合并后的逻辑态 (rx/ry = 人手 + 注入, 带 fire/ads/aim_gate)。这是分辨「注入没到游戏」还是「律根本没下命令」的唯一手段; 标定期打印的就是激励波形。只想正常玩时关掉。❄ 冷参数, 下次启动生效。" },
  { group: "瞄准", key: "model", label: "模型 engine", type: "engine", hot: false,
    info: "TensorRT engine 模型 (engine/ 下选择)。engine 与本机 GPU/TRT 版本绑定, 换机或升级 TRT 后需重新 convert; onnx 比对应 engine 新时「模型与运维」页会提示过期。冷参数, 下次启动生效。" },
  { group: "瞄准", key: "conf", label: "置信度阈值", type: "num", min: 0, max: 1, step: 0.01, hot: true,
    info: "检测框采纳门槛 (0–1)。调低: 更远/遮挡/侧面目标也会被锁定, 误检与抢锁风险上升; 调高: 只锁高置信目标, 残局可能丢锁。🔥 热参数, 保存即下发运行中实例。" },
  { group: "瞄准", key: "y_offset", label: "瞄准高度偏移", type: "num", min: 0, max: 100, step: 1, unit: "%", hot: true,
    info: "瞄准点在检测框内的垂直位置: 0=脚, 50=躯干中心, 65=胸颈, 100=头 (依模型标注框)。爆头线抬高数值。🔥 热参数, 保存即生效。" },
  { group: "瞄准", key: "max_speed", label: "速度上限", type: "num", min: 100, max: 20000, step: 50, unit: "px/s", hot: true,
    info: "输出指令的速度钳制。调大: 甩枪更快、远距离转移更利落, 失配时抖动/过冲风险上升; 调小: 稳, 但跟不动快速目标。🔥 热参数, 保存即生效。" },
  { group: "瞄准", key: "aim_key", label: "触发键", type: "select", options: ["fire", "ads", "both"], hot: true,
    labels: { fire: "左键 fire", ads: "右键 ads", both: "任一键 both" },
    info: "按住哪个键才自瞄: fire=左键, ads=右键, both=任一。🔥 热参数, 保存即生效。" },
  { group: "瞄准", key: "aim_enabled", label: "鼠标接管", type: "bool", hot: true,
    info: "关闭后固件只透传真实鼠标, 不注入任何移动; 检测与采集照常运行 —— 模型未完善、只想采数据时的形态, 随时可再打开恢复控制输出。🔥 热参数, 保存即生效。" },
  { group: "瞄准", key: "fov", label: "FOV 半径", type: "num", min: 10, max: 1000, step: 5, unit: "px", hot: true,
    info: "FOV 同时是目标筛选圈与积分器启动边界 (一值两用)。几何关系: 模型输入是 1080p 画面中心裁剪出的 640×640, 模型像素与屏幕像素 1:1, 检测范围为以准星为中心 ±320px (对角约 452px), 因此调到 452 以上没有额外效果。调大: 更远/更偏的目标进入筛选圈, 多目标抢锁风险上升; 调小: 只锁准星附近。🔥 热参数, 保存即生效。" },
  // 三套输出各占一组 5 个值 (延迟 1 + 拉枪倍率 4): 由 MODE_SLOT 生成 —— 切换【输出模式】
  // 显示/编辑的就是本槽那一组, 保存也只提交本槽 (见 savePatch)
  ...modeSlotRows(),
  { group: "瞄准", key: "class_id", label: "目标类别 ID", type: "num", min: 0, max: 255, step: 1, hot: false,
    info: "锁定哪个检测类别 (依模型标签, 如 0=头, 1=身)。❄ 冷参数, 下次启动生效。" },
  { group: "瞄准", key: "cam_fps", label: "采集帧率", type: "select", options: [120, 60], hot: false,
    info: "采集卡帧率。控制律增益按实测帧周期归一, 手感不随帧率变化; 60fps 省带宽但延迟滤波更钝。❄ 冷参数, 下次启动生效。" },
  { group: "瞄准", key: "cam_dev", label: "采集卡", type: "camera", hot: false,
    info: "采集卡设备。Hagibis/Asus 是按 /dev/v4l/by-id 名字做唯一子串匹配的别名 (与固件同一规则); /dev/videoN 是具体节点。插拔设备后到设置页重新扫描。❄ 冷参数, 下次启动生效。" },
  { group: "瞄准", key: "preview", label: "预览窗口", type: "bool", hot: false,
    info: "在 Jetson 接屏幕/桌面会话时显示检测画面 (框选 + 瞄准点)。没有接屏时开启会导致启动报错 —— 日志会如实显示, 改回关闭再启动即可。❄ 冷参数, 下次启动生效。" },
  { group: "训练数据采集", key: "capture_enabled", label: "开启采集", type: "bool", hot: false, captureSwitch: true,
    info: "开启后按三源自动截图: 开火瞬间 (fire) / 检测到目标 (det) / 定时 (auto), 分别写入输出目录的子目录, 每个源可单独开关。关闭 = 纯自瞄, 不写盘。❄ 冷参数, 下次启动生效。" },
  { group: "训练数据采集", key: "cap_fire", label: "开火源 fire", type: "bool", hot: true, showIf: "capture_enabled",
    info: "开火瞬间截图: 按住左键期间按「开火截图间隔」连拍。关闭后该源不落盘。🔥 热参数, 保存即生效。" },
  { group: "训练数据采集", key: "cap_det", label: "检测源 det", type: "bool", hot: true, showIf: "capture_enabled",
    info: "YOLO 检测命中进入 FOV 即拍 (按「检测/定时冷却」间隔) —— 正样本的主要来源; 模型不准时可关掉只留背景帧。🔥 热参数, 保存即生效。" },
  { group: "训练数据采集", key: "cap_auto", label: "定时源 auto", type: "bool", hot: true, showIf: "capture_enabled",
    info: "定时截图: 按「定时截图间隔」的随机 0.5x~1.5x 拍, 无目标也拍 (受冷却限制) —— 背景/困难负样本来源。🔥 热参数, 保存即生效。" },
  { group: "训练数据采集", key: "capture_dir", label: "输出目录", type: "dataset", hot: false, showIf: "capture_enabled",
    info: "截图输出根目录, 默认建议 dataset/<游戏名>/ (自动创建 fire/ det/ auto/ 子目录)。❄ 冷参数, 下次启动生效。" },
  { group: "训练数据采集", key: "fire_ms", label: "开火截图间隔", type: "num", min: 50, max: 60000, step: 50, unit: "ms", hot: false, showIf: "capture_enabled",
    info: "按住开火键时两张 fire 截图的最小间隔。调小样本密但重复帧多。❄ 冷参数。" },
  { group: "训练数据采集", key: "auto_s", label: "定时截图间隔", type: "num", min: 1, max: 3600, step: 1, unit: "s", hot: false, showIf: "capture_enabled",
    info: "auto 定时截图的基础间隔 (实际随机 0.5x~1.5x 抖动, 避免与游戏节奏同步)。❄ 冷参数。" },
  { group: "训练数据采集", key: "cooldown_ms", label: "检测/定时冷却", type: "num", min: 0, max: 60000, step: 50, unit: "ms", hot: false, showIf: "capture_enabled",
    info: "det 与 auto 截图共用的最小间隔 (fire 不受限)。防止对同一目标连拍爆盘。❄ 冷参数。" },
  { group: "训练数据采集", key: "jpeg_q", label: "JPEG 质量", type: "num", min: 1, max: 100, step: 1, hot: false, showIf: "capture_enabled",
    info: "截图 JPEG 质量 (1–100)。训练用 90+ 即可; 调小省盘。❄ 冷参数。" },
];
const HOT_KEYS = PARAM_DEFS.filter(p => p.hot).map(p => p.key);

/* ================= 主题 ================= */
function applyTheme() {
  document.documentElement.dataset.theme = S.theme;
  $("#themeBtn").textContent = S.theme === "dark" ? "☀" : "☾";
  $("#themeSel").value = S.theme;
  localStorage.setItem("wb_theme", S.theme);
}
function toggleTheme() {
  S.theme = S.theme === "dark" ? "light" : "dark";
  applyTheme();
}

/* ================= 认证 (密码为主, token 万能兜底) ================= */
const Auth = { mode: "password" };   // password | token | setup

function showAuthModal(mode) {
  Auth.mode = mode || "password";
  $("#tokenModal").classList.remove("hidden");
  $("#tokenErr").classList.add("hidden");
  const conf = {
    setup: { title: "首次设置", btn: "设置并登录",
      hint: "输入 token 验证身份, 并为这台 WebUI 设置登录密码 (至少 4 位)。token 打印在服务日志: journalctl -u aimbot-webui",
      tok: true, pass: true, pass2: true, sw: "" },
    password: { title: "登录", btn: "登录",
      hint: "输入密码登录 WebUI。",
      tok: false, pass: true, pass2: false, sw: "改用 token 登录" },
    token: { title: "登录", btn: "登录",
      hint: "输入 token 登录 (万能凭证; URL ?token=… 直达等效)。登录后可在设置页改密码。",
      tok: true, pass: false, pass2: false, sw: "改用密码登录" },
  }[Auth.mode];
  $("#authTitle").textContent = conf.title;
  $("#authHint").textContent = conf.hint;
  $("#authTokenRow").classList.toggle("hidden", !conf.tok);
  $("#authPassRow").classList.toggle("hidden", !conf.pass);
  $("#authPass2Row").classList.toggle("hidden", !conf.pass2);
  $("#btnAuthSwitch").classList.toggle("hidden", !conf.sw);
  $("#btnAuthSwitch").textContent = conf.sw;
  $("#btnTokenOk").textContent = conf.btn;
  $(Auth.mode === "password" ? "#authPass" : "#authToken").focus();
}

function authFail(msg) {
  $("#tokenErr").textContent = msg;
  $("#tokenErr").classList.remove("hidden");
}

async function submitAuth() {
  let r;
  try {
    if (Auth.mode === "setup") {
      const pw = $("#authPass").value;
      if (pw.length < 4) return authFail("密码至少 4 位");
      if (pw !== $("#authPass2").value) return authFail("两次输入的密码不一致");
      r = await fetch("/api/auth/setup", { method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ token: $("#authToken").value.trim(), password: pw }) });
    } else if (Auth.mode === "password") {
      r = await fetch("/api/auth/login", { method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ password: $("#authPass").value }) });
    } else {
      r = await fetch("/api/auth/login", { method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ token: $("#authToken").value.trim() }) });
    }
  } catch (err) { return authFail("网络错误: " + err.message); }
  if (!r.ok) {
    let msg = "登录失败 (HTTP " + r.status + ")";
    try { msg = (await r.json()).detail || msg; } catch (e) { /* keep */ }
    return authFail(msg);
  }
  const doc = await r.json();
  S.token = doc.token;
  localStorage.setItem("wb_token", doc.token);
  $("#tokenModal").classList.add("hidden");
  boot();
}

async function initAuth() {
  let hasPw = true;
  try {
    hasPw = (await (await fetch("/api/auth/mode")).json()).has_password;
  } catch (e) { /* 探测失败回退密码模式 */ }
  showAuthModal(hasPw ? "password" : "setup");
}

/* ================= 状态获取 ================= */
async function fetchState() {
  const st = await api("/api/state");
  S.state = st;
  const logs = st.logs || [];
  const fresh = logs.filter(l => l[0] > S.wsSeq);
  if (fresh.length) { appendLogs(fresh); S.wsSeq = fresh[fresh.length - 1][0]; }
  else if (!S.logBuf.length) { appendLogs(logs); S.wsSeq = logs.length ? logs[logs.length - 1][0] : 0; }
  S.fpsHist = st.fps_hist || [];
  if (!S.params) selectProfile(pickDefaultProfile(), { force: true });
  renderAll();
  renderFpsPanel();
  pollCapFolder();
}

function pickDefaultProfile() {
  const profs = profsSafe();
  if (profs.find(p => p.file === S.selected)) return S.selected;
  const running = S.state.instance.profile;
  if (running && profs.find(p => p.file === running)) return running;
  return profs.length ? profs[0].file : "";
}
function profsSafe() { return (S.state && S.state.scan.profiles) || []; }
function curProfile() { return profsSafe().find(p => p.file === S.selected) || null; }

function selectProfile(stem, opts) {
  opts = opts || {};
  if (!opts.force && S.dirty &&
      !confirm("当前 profile 有未保存改动, 切换将丢弃。继续?")) {
    $("#profileSel").value = S.selected;       // 回弹
    return;
  }
  S.selected = stem || "";
  localStorage.setItem("wb_profile", S.selected);
  const p = curProfile();
  S.params = Object.assign({}, blankParams(), p ? p.script_params : {});
  S.savedParams = Object.assign({}, S.params);
  S.dirty = false;
  renderParams();
  renderAll();
  pollCapFolder();
}
function blankParams() {
  const o = {};
  PARAM_DEFS.forEach(d => {
    o[d.key] = d.type === "num" ? (d.step < 1 ? 0.5 : 0) : (d.type === "bool" ? false : "");
  });
  return o;
}

/* ================= 渲染: 顶栏 / 横幅 ================= */
function renderAll() {
  renderTopbar();
  renderRunTab();
  renderParamsMeta();
  renderOps();
  renderSettings();
}

function renderTopbar() {
  const st = S.state;
  const profs = profsSafe();
  const selHtml = profs.map(p =>
    `<option value="${esc(p.file)}"${p.file === S.selected ? " selected" : ""}>${esc(p.display_name)}</option>`).join("");
  const sel = $("#profileSel");
  if (sel.dataset.sig !== profs.map(p => p.file + p.display_name).join("|")) {
    sel.innerHTML = selHtml || "<option value=''>（无 profile）</option>";
    sel.dataset.sig = profs.map(p => p.file + p.display_name).join("|");
    sel.value = S.selected;
  }
  $("#profileTitle").textContent = "游戏 profile — " + (curProfile() ? curProfile().display_name : "（无）");

  const inst = st.instance, tel = st.telemetry;
  const chips = [];
  const stMap = { stopped: ["", "● 已停止"], starting: ["warn", "◐ 启动中…"], running: ["ok", "● 运行中 " + fmtDur(inst.started_at ? Date.now() / 1000 - inst.started_at : null)], stopping: ["warn", "◐ 停止中…"], exited: [inst.abnormal ? "err" : "", "■ 已退出 " + exitText(inst)] };
  const sm = stMap[inst.state] || ["", inst.state];
  chips.push(`<span class="chip ${sm[0]}"><b>${sm[1]}</b></span>`);
  if (inst.adopted) chips.push(`<span class="chip warn">认领</span>`);
  if (inst.state === "running" && inst.profile && inst.profile !== S.selected)
    chips.push(`<span class="chip warn">运行的是「${esc(inst.display_name || inst.profile)}」</span>`);
  if (inst.fps != null) chips.push(`<span class="chip accent">FPS <b>${inst.fps}</b></span>`);
  if (tel) {
    if (tel.cpu != null) chips.push(`<span class="chip">CPU <b>${tel.cpu}%</b></span>`);
    if (tel.gpu != null) chips.push(`<span class="chip">GPU <b>${tel.gpu}%</b></span>`);
    if (tel.mem) chips.push(`<span class="chip">内存 <b>${tel.mem.percent}%</b></span>`);
    if (tel.soc_temp != null) {
      const cls = tel.soc_temp >= 85 ? "err" : tel.soc_temp >= 70 ? "warn" : "";
      chips.push(`<span class="chip ${cls}">SoC <b>${tel.soc_temp.toFixed(0)}°C</b></span>`);
    }
  }
  $("#liveChips").innerHTML = chips.join("");

  $("#btnStart").disabled = RUNNING.includes(inst.state);
  $("#btnStop").disabled = !RUNNING.includes(inst.state);
  $("#btnSave").classList.toggle("dirty", S.dirty);
  $("#btnSave2").classList.toggle("dirty", S.dirty);
  $("#dirtyHint").classList.toggle("hidden", !S.dirty);
  $("#btnRevert").classList.toggle("hidden", !S.dirty);
  $("#logDownload").href = "/api/logs/current?token=" + encodeURIComponent(S.token);

  // 全局横幅: 根目录问题 / 异常退出 / 选中未生效
  const banners = [];
  if (st.scan.status !== "ok") banners.push(["err", "部署根不可用: " + st.scan.error + " — 到「设置」页修正"]);
  else (st.scan.warnings || []).forEach(w => banners.push(["", "⚠ " + w]));
  if (inst.state === "exited" && inst.abnormal)
    banners.push(["err", "上次异常退出: " + exitText(inst) + " — 详情见下方日志/步骤条"]);
  if (inst.state === "running" && inst.profile && inst.profile !== S.selected)
    banners.push(["", `运行中的是「${esc(inst.display_name || inst.profile)}」; 当前选中「${esc(curProfile() ? curProfile().display_name : S.selected)}」未生效 — 按【启动】应用`]);
  const gb = $("#globalBanner");
  if (banners.length) {
    gb.innerHTML = banners.map(b => `<div class="banner ${b[0]}" style="margin-bottom:6px">${b[1]}</div>`).join("");
    gb.classList.remove("hidden");
  } else gb.classList.add("hidden");
}

function exitText(inst) {
  if (inst.exit_signal != null) return "信号 " + inst.exit_signal;
  if (inst.exit_code != null) return "退出码 " + inst.exit_code;
  return inst.error ? "启动失败" : "";
}

/* ================= 渲染: 运行页 ================= */
function renderRunTab() {
  const inst = S.state.instance;
  // 步骤条
  const steps = inst.steps || [];
  const sb = $("#stepsBar");
  if (steps.length) {
    sb.classList.remove("hidden");
    sb.innerHTML = steps.map(s => {
      const icon = { ok: "✓", fail: "✗", warn: "⚠", running: "◐", pending: "·" }[s.status] || "·";
      return `<div class="step ${s.status}"><div class="st-name">${icon} ${esc(s.name)}</div>` +
             `<div class="st-detail" title="${esc(s.detail)}">${esc(s.detail || "")}${s.ms ? " · " + s.ms + "ms" : ""}</div></div>`;
    }).join("");
  } else sb.classList.add("hidden");

  // 状态卡
  const tel = S.state.telemetry, p = curProfile();
  const model = S.params && S.params.model ? String(S.params.model).split("/").pop() : "（未选）";
  // 运行中的实例有自己的模式与标定回执; 没在跑就看当前选中的 profile
  const runProf = (inst.state === "running" && inst.profile && !inst.adopted)
    ? profsSafe().find(x => x.file === inst.profile) : null;
  const calibProf = runProf || p;
  const mode = (calibProf && calibProf.script_params && calibProf.script_params.output_mode)
    || (S.params && S.params.output_mode) || "hid";
  // 延迟: 本模式那一槽 (三套输出各一格), 与固件回写的 VAR 同一条规则
  const slot = MODE_SLOT.find(sl => sl.mode === mode) || MODE_SLOT[0];
  const lKey = "l_" + slot.mode;
  const lVar = slot.lvar;
  const scriptL = calibProf && calibProf.script_params ? calibProf.script_params[lKey] : null;
  const calib = inst.calib_live && inst.calib_live.l != null ? inst.calib_live
              : (scriptL != null ? { l: scriptL } : null);
  const soc = tel && tel.soc_temp != null ? tel.soc_temp : null;
  const cap = inst.capture;
  const capTotal = cap ? cap.fire + cap.det + cap.auto : null;
  const capSmall = cap ? "fire " + cap.fire + " · det " + cap.det + " · auto " + cap.auto
                       : (inst.adopted ? "认领实例无日志" : "");
  const mi = inst.model_info;
  const MODE_LABEL = { hid: "hid — 鼠标 → PC", pad: "pad — XInput 手柄 → PC",
                       p5g: "p5g — P5 General → PS5" };
  const cards = [
    { k: "输出模式", v: esc(mode), small: mode === "hid" ? "-D 选鼠标" : "-P/-T 选手柄",
      t: MODE_LABEL[mode] || mode },
    { k: "模型 (选中)", v: esc(model) },
    { k: "模型架构", v: mi ? esc(mi.arch) : "—", small: mi && mi.classes != null ? mi.classes + " 类" : "", t: "每次启动从固件控制台输出抓取; v8 与 v11 在输出层不可区分, 固件打印 YOLOv8/11" },
    { k: "输入尺寸", v: mi ? mi.size : "—", small: mi ? "px" : "", t: "模型输入分辨率, 每次启动从固件控制台输出抓取" },
    { k: "帧率 (自动识别)", v: inst.fps != null ? inst.fps : "—", small: "fps", cls: inst.fps >= 100 ? "ok" : (inst.fps != null ? "warn" : "") },
    { k: "截图 本次", v: capTotal != null ? capTotal : "—", small: capSmall, t: "本次运行固件累计截图; 重新【启动】归零" },
    { k: "截图 文件夹", v: S.capFolder != null ? S.capFolder : "—", small: S.capFolderName || "输出目录", t: S.capDir || "输出目录内全部截图 (10s 刷新)" },
    { k: "CPU", v: tel && tel.cpu != null ? tel.cpu : "—", small: "%" },
    { k: "GPU", v: tel && tel.gpu != null ? tel.gpu : "—", small: "%" },
    { k: "内存", v: tel && tel.mem ? tel.mem.percent : "—", small: tel && tel.mem ? "· " + tel.mem.used_mb + "MB" : "" },
    { k: "SoC 温度", v: soc != null ? soc.toFixed(0) : "—", small: "°C", cls: soc >= 85 ? "err" : soc >= 70 ? "warn" : "" },
    { k: "标定延迟 L (" + esc(lVar) + ")", v: calib ? calib.l : "—",
      small: "ms · " + (inst.calib_live ? "运行中回执" : "脚本值"),
      t: "本输出模式的延迟: " + lVar + " (三套输出各一格, 互不覆盖)。标定只出这一个量, 由它导出控制律带宽; 手感由「延迟与拉枪倍率」那组的四个倍率调 — 固件不写它们。也可在参数页手填。" },
    { k: "热参数通道", v: inst.state === "running" ? (inst.hot_capable ? "已启用" : "不可用") : (S.state.scan.binary && S.state.scan.binary.hot_capable ? "固件支持" : "固件不支持"), small: inst.hot_port ? ":" + inst.hot_port : "", cls: (inst.state === "running" && !inst.hot_capable) ? "warn" : (inst.hot_capable ? "ok" : "") },
  ];
  $("#statusCards").innerHTML = cards.map(c =>
    `<div class="stat ${c.cls || ""}"${c.t ? ` title="${esc(c.t)}"` : ""}><div class="k">${c.k}</div><div class="v">${c.v}<small>${c.small || ""}</small></div></div>`).join("");

  // 运行横幅
  const rb = $("#runBanner");
  if (inst.state === "exited" && inst.error) {
    rb.textContent = "✗ " + inst.error;
    rb.className = "banner err";
  } else if (inst.state === "exited" && inst.abnormal) {
    rb.textContent = "✗ 异常退出: " + exitText(inst) + " — 若反复出现, 把下方日志拉到底看最后几十行";
    rb.className = "banner err";
  } else if (inst.state === "running" && inst.adopted) {
    rb.textContent = "该实例不是本 WebUI 启动的 (认领): 日志不可见; 按【启动】以当前设置接管";
    rb.className = "banner";
  } else rb.classList.add("hidden"), rb.className = "banner hidden";

  // 标定卡片: 触发方式按模式给 —— hid 只有鼠标双侧键长按, 手柄模式还有这个按钮
  const gamepad = (mode === "pad" || mode === "p5g");
  $("#btnCalib").disabled = !(gamepad && inst.state === "running" && !inst.adopted
                              && inst.hot_capable);
  $("#calibMode").textContent = MODE_LABEL[mode] || mode;
  $("#calibVar").textContent = lVar;
  $("#calibHint").textContent = !gamepad
    ? "hid 的触发是鼠标双侧键长按 5 秒 (没有按钮入口): 游戏内对准有细节的静止背景, 双手离开控制器后按住双侧键不放。"
    : inst.state !== "running" ? "先在【启动】里跑起一个本模式实例, 再从这里触发。"
    : inst.adopted ? "运行中的是认领实例 (不知其读的是哪个 profile), 不盲发热参; 按【启动】接管后即可触发。"
    : !inst.hot_capable ? "运行中的固件是旧版 (无热参数通道): 重编译固件后启动即可用按钮。"
    : "点此按钮, 或在手柄上长按 L3+R3 5 秒。激励期间程序独占右摇杆且人手通道归中 —— 准星会按计划摆动, 那是测量不是失控; 全程几十秒, 结论与逐级读数看日志的 [标定] 行。";

  // 启动命令预览 (懒加载)
  $("#cmdDetails").querySelector("summary").innerHTML =
    `启动命令预览 <span class="hint">(WebUI 将执行的确切命令${inst.profile ? ", 当前 profile: " + esc(inst.profile) : ""})</span>`;
}

async function loadCmdPreview() {
  if (!S.selected) return;
  try {
    const r = await api("/api/instance/cmd?profile=" + encodeURIComponent(S.selected));
    $("#cmdPreview").textContent = r.cmd;
  } catch (e) { $("#cmdPreview").textContent = "(" + e.message + ")"; }
}

/* ================= 日志 ================= */
function lineClass(t) {
  if (/════/.test(t)) return "l-sep";
  if (/✗|❌|失败|错误|ERROR|断开|无法|不存在|未找到|被占/.test(t)) return "l-err";
  if (/⚠/.test(t)) return "l-warn";
  if (/\[热参\]/.test(t)) return "l-hot";
  if (/\[标定\]/.test(t)) return "l-calib";
  if (/✅|已启动/.test(t)) return "l-ok";
  return "";
}
function appendLogs(lines) {
  const box = $("#logBox");
  const fl = ($("#logFilter").value || "").toLowerCase();
  const atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 24;
  for (const [, text] of lines) {
    S.logBuf.push(text);
    if (S.logBuf.length > 1500) S.logBuf.shift();
    if (!fl || text.toLowerCase().includes(fl)) {
      const el = document.createElement("div");
      el.className = lineClass(text);
      el.textContent = text;
      box.appendChild(el);
    }
  }
  while (box.children.length > 1600) box.removeChild(box.firstChild);
  if (S.follow) box.scrollTop = box.scrollHeight;
}
function rerenderLog() {
  const box = $("#logBox");
  const fl = ($("#logFilter").value || "").toLowerCase();
  box.innerHTML = "";
  for (const text of S.logBuf) {
    if (fl && !text.toLowerCase().includes(fl)) continue;
    const el = document.createElement("div");
    el.className = lineClass(text);
    el.textContent = text;
    box.appendChild(el);
  }
  box.scrollTop = box.scrollHeight;
}

/* ================= AI FPS 历史 (会话级: 重新启动即清空) ================= */
function renderFpsPanel() {
  const card = $("#fpsHistCard");
  if (!card || card.classList.contains("hidden")) return;
  const h = S.fpsHist || [];
  const sum = $("#fpsHistSummary"), spark = $("#fpsSpark"), tbl = $("#fpsHistTable");
  $("#fpsHistBody").classList.toggle("hidden", !card.open);
  if (!card.open) return;
  if (!h.length) {
    sum.textContent = "本次运行还没有 [AI FPS] 读数 (固件每 60 秒记一次; 认领实例无日志)。";
    spark.innerHTML = ""; tbl.innerHTML = "";
    return;
  }
  const vals = h.map(x => x[1]);
  const avg = vals.reduce((a, b) => a + b, 0) / vals.length;
  sum.innerHTML = `样本 <b>${h.length}</b> · 最新 <b>${vals[vals.length - 1]}</b> fps · ` +
    `最低 <b>${Math.min(...vals)}</b> · 最高 <b>${Math.max(...vals)}</b> · 均值 <b>${avg.toFixed(1)}</b>`;
  const recent = vals.slice(-120);
  const vmax = Math.max(...recent, 1);
  spark.innerHTML = recent.map(v =>
    `<i style="height:${Math.max(4, Math.round(v / vmax * 100))}%" title="${v} fps"></i>`).join("");
  tbl.innerHTML = `<table class="tbl"><tr><th>时间</th><th>AI FPS</th></tr>` +
    h.slice(-15).reverse().map(([ts, v]) =>
      `<tr><td>${fmtClock(ts)}</td><td>${v}</td></tr>`).join("") + `</table>`;
}

/* ================= 截图文件夹计数 (10s 轮询) ================= */
async function pollCapFolder() {
  const inst = S.state && S.state.instance;
  // 运行中优先看正在写的那份 profile 的输出目录; 其余时候看当前选中的
  const stem = (inst && RUNNING.includes(inst.state) && inst.profile && !inst.adopted)
    ? inst.profile : S.selected;
  if (!stem) { S.capFolder = null; S.capFolderName = ""; S.capDir = ""; return; }
  try {
    const r = await api("/api/capture/count?profile=" + encodeURIComponent(stem));
    S.capFolder = r.total;
    S.capFolderName = r.exists ? r.dir_name : r.dir_name + " (不存在)";
    S.capDir = r.dir;
  } catch (e) { /* profile 切换瞬间可能 404, 下轮再取 */ }
}

/* ================= 渲染: 参数页 ================= */
function paramRow(d) {
  if (d.showIf && !S.params[d.showIf]) return "";
  // 模式相关的行只在当前输出模式下有意义 (hid 用 -D; pad/p5g 用 -P/-T/--pad-dump)
  if (d.showIfMode && !d.showIfMode.includes(S.params.output_mode)) return "";
  const v = S.params[d.key];
  // 运行中的固件不支持热参数 → 🔥 置灰并提示需重编译 (不盲发, 下次启动生效)
  const hotDead = d.hot && S.state.instance.state === "running" &&
                  !S.state.instance.hot_capable;
  const badge = d.hot
    ? `<span class="badge hot${hotDead ? " off" : ""}" title="${hotDead
        ? "运行中的固件是旧版 (无热参数通道, 需重编译): 本项将在下次【启动】生效"
        : "热参数: 保存后即时下发运行中实例"}">🔥 热</span>`
    : `<span class="badge" title="冷参数: 下次【启动】生效">❄ 冷</span>`;
  let ctl = "";
  if (d.type === "num") {
    ctl = `<input type="number" data-pk="${d.key}" value="${esc(v)}" min="${d.min}" max="${d.max}" step="${d.step}">` +
          (d.unit ? `<span class="p-unit">${d.unit}</span>` : "") +
          `<span class="p-unit">范围 ${d.min}–${d.max}</span>`;
  } else if (d.type === "select") {
    ctl = `<select data-pk="${d.key}">` + d.options.map(o =>
      `<option value="${esc(o)}"${String(v) === String(o) ? " selected" : ""}>${esc(d.labels ? d.labels[o] || o : o)}</option>`).join("") + `</select>`;
  } else if (d.type === "text") {
    ctl = `<input type="text" data-pk="${d.key}" value="${esc(v == null ? "" : v)}" spellcheck="false" placeholder="留空 = 任意设备 (取字典序首个)">`;
  } else if (d.type === "bool") {
    ctl = `<label class="switch"><input type="checkbox" data-pk="${d.key}"${v ? " checked" : ""}><span class="tr"></span></label>`;
  } else if (d.type === "engine") {
    const engs = (S.state.scan.engines || []).map(e => e.path);
    ctl = dynSelect(d.key, v, engs.map(p => ({ v: p, t: p })), "先去 convert");
  } else if (d.type === "camera") {
    const cams = (S.state.scan.cameras || []).map(c => ({ v: c.value, t: c.label }));
    ctl = dynSelect(d.key, v, cams, "未发现采集卡");
  } else if (d.type === "dataset") {
    const dirs = (S.state.scan.dataset_dirs || []).map(n => ({ v: "dataset/" + n, t: "dataset/" + n }));
    dirs.unshift({ v: "dataset/" + (S.selected || "game"), t: "dataset/" + (S.selected || "game") + " (建议)" });
    ctl = dynSelect(d.key, v, dirs, "未设置");
  }
  const changed = !sameVal(v, S.savedParams[d.key]);
  return `<div class="prow${changed ? " changed" : ""}" data-row="${d.key}">
    <div class="p-label">${esc(d.label)}
      ${badge}
      <button class="info-btn" data-info="${d.key}" title="说明">i</button></div>
    <div class="p-ctl">${ctl}</div>
    <div class="p-info" data-info-of="${d.key}" style="display:${S.openInfo.has(d.key) ? "block" : "none"}">${esc(d.info)}</div>
  </div>`;
}
function dynSelect(key, val, items, emptyText) {
  const has = items.some(it => String(it.v) === String(val));
  if (val != null && val !== "" && !has)
    items = items.concat([{ v: val, t: val + "（不在发现结果中）" }]);
  if (!items.length) items = [{ v: "", t: emptyText || "（无选项）" }];
  if (val == null || val === "") items = [{ v: "", t: "请选择…" }].concat(items);
  return `<select data-pk="${key}">` + items.map(it =>
    `<option value="${esc(it.v)}"${String(it.v) === String(val) ? " selected" : ""}>${esc(it.t)}</option>`).join("") + `</select>`;
}

function renderParams() {
  const groups = [];
  PARAM_DEFS.forEach(d => {
    let g = groups.find(x => x.name === d.group);
    if (!g) { g = { name: d.group, rows: [] }; groups.push(g); }
    g.rows.push(d);
  });
  // 空组不渲染: 三套输出的槽组各含 showIfMode, 只有当前模式的组有可见行
  $("#paramGroups").innerHTML = groups.map(g => {
    const rows = g.rows.map(paramRow).join("");
    return rows.trim() ? `<div class="pgroup"><h4>${esc(g.name)}</h4>${rows}</div>` : "";
  }).join("");
}

function renderParamsMeta() {
  const inst = S.state.instance;
  const hb = $("#hotBanner");
  if (inst.state === "running") {
    hb.textContent = inst.adopted
      ? "运行中的是认领实例: 不盲发热参数; 按【启动】以当前设置接管后, 热参数才可即时下发"
      : inst.hot_capable
        ? "实例运行中 — 🔥 热参数保存后即时写回脚本并下发 (回执看日志 [热参] 行), ❄ 冷参数下次启动生效"
        : "运行中的固件是旧版 (无热参数通道): 热参数保存后要等下次【启动】才生效 — 重编译固件可启用";
    hb.className = "banner info";
  } else hb.className = "banner info hidden";
  // 行变化态局部刷新 (不动输入焦点): 只更新 changed 类
  $$("#paramGroups .prow").forEach(row => {
    const k = row.dataset.row;
    const d = PARAM_DEFS.find(x => x.key === k);
    if (d) row.classList.toggle("changed", !sameVal(S.params[k], S.savedParams[k]));
  });
}

function markDirty() {
  S.dirty = true;
  $("#btnSave").classList.add("dirty");
  $("#btnSave2").classList.add("dirty");
  $("#dirtyHint").classList.remove("hidden");
  $("#btnRevert").classList.remove("hidden");
  renderParamsMeta();
}

/* ================= 保存 / 启动 / 停止 ================= */
/* 保存的提交体 = 补丁: 只提交与脚本现值不同的键, 且**不含**其它输出槽的槽键 ——
   于是"保存只写当前模式那一槽"成立: 切换模式后原样保存, 另两槽的 10 个 VAR 逐字不动
   (参数页只显示本槽, 另两槽即使被改过也不提交, 切槽时会提示)。*/
function savePatch() {
  const cur = S.params.output_mode || "hid";
  const drop = new Set();
  Object.keys(MODE_SLOT_KEYS).forEach(m => {
    if (m !== cur) MODE_SLOT_KEYS[m].forEach(k => drop.add(k));
  });
  const out = {};
  PARAM_DEFS.forEach(d => {
    if (drop.has(d.key)) return;
    if (!sameVal(S.params[d.key], S.savedParams[d.key])) out[d.key] = S.params[d.key];
  });
  return out;
}

async function saveProfile(silent) {
  const patch = savePatch();
  const changed = Object.keys(patch).length;
  if (!changed) {                       // 无改动不发请求: 不重写脚本 (行不会被规范化改写)
    if (!silent) toast("没有改动需要保存", "");
    S.dirty = false;
    return { ok: true, hot_applied: {} };
  }
  const r = await api("/api/profiles/" + encodeURIComponent(S.selected),
                      { method: "PUT", body: { params: patch } });
  S.savedParams = Object.assign({}, S.params);
  S.dirty = false;
  // 脚本是唯一事实源: 保存 = 服务端已写回脚本; 拉回最新扫描并回读刷新表单 (显示规范值)
  try { S.state.scan = await api("/api/scan", { method: "POST" }); } catch (e) { /* 下次对齐 */ }
  if (!silent) {
    if (r.hot_applied && Object.keys(r.hot_applied).length) {
      toast("已保存并写回脚本; 热参数已下发: " + Object.entries(r.hot_applied).map(([k, v]) => k + "=" + v).join("  ") + " — 生效回执看日志", "ok");
    } else if (r.hot_reason) {
      toast("已保存并写回脚本; " + r.hot_reason, "warn");
    } else {
      toast("已保存并写回脚本 (本槽 " + changed + " 项)", "ok");
    }
  }
  selectProfile(S.selected, { force: true });
  return r;
}

async function startInstance() {
  if (!S.selected) { toast("没有可选的游戏 profile", "err"); return; }
  const inst = S.state.instance;
  const p = curProfile();
  if (RUNNING.includes(inst.state) &&
      !confirm(`将停止当前实例并以「${p ? p.display_name : S.selected}」重新启动 (启动 = 保存 + 清场 + 拉起)。继续?`)) return;
  try {
    if (S.dirty) await saveProfile(true);
    if (window.Notification && Notification.permission === "default") {
      Notification.requestPermission();     // 异常退出桌面通知, 用户手势里申请
    }
    await api("/api/instance/start", { method: "POST", body: { profile: S.selected } });
    toast("启动序列已开始 (jetson_clocks → 鼠标 → aimbot), 看下方步骤与日志", "ok");
    showTab("run");
    pollCapFolder();
  } catch (e) { toast("启动失败: " + e.message, "err"); }
}

async function stopInstance() {
  if (!confirm("停止当前实例?")) return;
  try {
    await api("/api/instance/stop", { method: "POST" });
    toast("已发送停止 (SIGTERM → 超时 SIGKILL)", "ok");
  } catch (e) { toast("停止失败: " + e.message, "err"); }
}

async function triggerCalib() {
  if (!confirm("让运行中的实例跑一轮标定?\n\n期间请双手离开控制器 (手柄模式程序会独占右摇杆), "
               + "游戏内保持有细节的静止背景。准星会按计划摆动几十秒。"))
    return;
  try {
    await api("/api/instance/calib", { method: "POST" });
    toast("标定请求已发出 (热参 padcalib=1) — 生效与结论看日志 [标定] 行", "ok");
    showTab("run");
  } catch (e) { toast("标定触发失败: " + e.message, "err"); }
}

/* ================= 渲染: 模型与运维 ================= */
function renderOps() {
  const scan = S.state.scan;
  // 文件列表只在 scan 变化时重建 (每 tick 重建会浪费 DOM 并打断滚动)
  const sig = JSON.stringify([scan.engines, scan.onnx, S.params && S.params.model]);
  if (S._opsSig !== sig) {
    S._opsSig = sig;
    const engs = scan.engines || [];
    const onnx = scan.onnx || [];
    $("#engineList").innerHTML = engs.length ? engs.map(e =>
      `<div class="frow"><span class="f-name">${esc(e.path)}</span>
        <span class="f-right"><span class="f-meta">${fmtSize(e.size)} · ${fmtDate(e.mtime)}</span>
        ${String(S.params && S.params.model) === e.path ? '<span class="badge okb">当前选中</span>' : ""}
        <button class="btn small" data-use-engine="${esc(e.path)}">选用</button></span></div>`).join("")
      : `<div class="empty">engine/ 下没有模型 — 用右侧 onnx 列表旁的「转换」生成</div>`;
    $("#onnxList").innerHTML = onnx.length ? onnx.map(o =>
      `<div class="frow"><span class="f-name">${esc(o.path)}</span>
        <span class="f-right"><span class="f-meta">${fmtSize(o.size)} · ${fmtDate(o.mtime)}</span>
        ${o.engine ? (o.stale ? '<span class="badge stale">engine 过期</span>' : '<span class="badge okb">已转换</span>') : '<span class="badge">未转换</span>'}</span></div>`).join("")
      : `<div class="empty">onnx/ 下没有模型文件</div>`;
  }

  const running = RUNNING.includes(S.state.instance.state);
  $("#btnCompile").disabled = running;
  $("#btnCompile").title = running ? "实例运行中, 先停止再编译 (二进制被占用)" : "重编 bin/aimbot";

  renderTasks();
  const h = S.state.history || [];
  const hsig = JSON.stringify(h);
  if (S._histSig !== hsig) {
    S._histSig = hsig;
    $("#historyBox").innerHTML = h.length ? `<table class="tbl"><tr><th>结束时间</th><th>profile</th><th>时长</th><th>退出</th></tr>` +
      h.slice().reverse().map(e =>
        `<tr><td>${fmtClock(e.ts)}</td><td>${esc(e.display_name || e.profile || "—")}</td>` +
        `<td>${e.duration_s != null ? fmtDur(e.duration_s) : "—"}</td>` +
        `<td>${e.abnormal ? '<span class="badge stale">异常</span> ' : ""}${esc(e.exit_signal != null ? "信号 " + e.exit_signal : e.exit_code != null ? "rc=" + e.exit_code : "—")}</td></tr>`).join("") +
      `</table>` : `<div class="empty">还没有运行记录</div>`;
  }
}

function renderTasks() {
  const box = $("#taskList");
  const list = S.state.tasks || [];
  const ph = box.querySelector(".empty");
  if (!list.length) {
    if (!ph) box.innerHTML = `<div class="empty">还没有运维任务</div>`;
    return;
  }
  if (ph) ph.remove();
  // 只增不删 DOM; 状态/头部文本每 tick 更新
  list.forEach(t => {
    let el = box.querySelector(`[data-task="${t.id}"]`);
    if (!el) {
      el = document.createElement("div");
      el.className = "task";
      el.dataset.task = t.id;
      el.innerHTML = `<div class="task-hd"><span class="t-kind"></span><span class="t-status"></span>
        <span class="hint t-meta"></span><span class="flex1"></span><span class="hint">点击展开/收起</span></div>
        <pre class="task-log logbox hidden" style="height:200px"></pre>`;
      el.querySelector(".task-hd").addEventListener("click", () => {
        S.taskOpen[t.id] = !S.taskOpen[t.id];
        updateTaskEl(t);
      });
      box.appendChild(el);
      S.taskOpen[t.id] = t.status === "running";
    }
    updateTaskEl(t, el);
  });
}
function updateTaskEl(t, el) {
  el = el || $(`[data-task="${t.id}"]`);
  if (!el) return;
  el.querySelector(".t-kind").textContent = t.kind === "convert" ? "convert 模型转换" : "compile 编译";
  const st = el.querySelector(".t-status");
  st.textContent = { running: "● 运行中", ok: "✓ 成功", failed: "✗ 失败" }[t.status] || t.status;
  st.className = "t-status " + t.status;
  el.querySelector(".t-meta").textContent =
    fmtClock(t.started_at) + (t.ended_at ? " → " + fmtClock(t.ended_at) : "") +
    (t.exit_code != null ? " · 退出码 " + t.exit_code : "");
  const logEl = el.querySelector(".task-log");
  logEl.classList.toggle("hidden", !S.taskOpen[t.id]);
  // 展开时若日志还没加载过 (页面中途进来的历史任务), 从后端补全量
  if (S.taskOpen[t.id] && el.dataset.loaded !== "1") {
    el.dataset.loaded = "1";
    api(`/api/tasks/${t.id}/log?since=0`).then(r => {
      logEl.textContent = r.lines.map(l => l[1]).join("\n");
      el.dataset.domSeq = String(r.seq);
      logEl.scrollTop = logEl.scrollHeight;
    }).catch(() => { });
  }
}
function appendTaskLog(tid, lines) {
  const el = $(`[data-task="${tid}"]`);
  if (!el) return;
  const logEl = el.querySelector(".task-log");
  // 收起时也累积文本 (保持与 seq 一致), 展开时可见; 只有滚动按需
  for (const [, text] of lines) logEl.textContent += (logEl.textContent ? "\n" : "") + text;
  if (!logEl.classList.contains("hidden")) logEl.scrollTop = logEl.scrollHeight;
  el.dataset.domSeq = String(lines[lines.length - 1][0]);
}

/* ================= 渲染: 设置页 ================= */
function renderSettings() {
  const st = S.state, cfg = st.config, scan = st.scan;
  if (document.activeElement !== $("#rootInput")) $("#rootInput").value = cfg.deploy_root;
  $("#bindSel").value = cfg.bind;
  if (document.activeElement !== $("#portInput")) $("#portInput").value = cfg.port;
  if (document.activeElement !== $("#tokenInput")) $("#tokenInput").value = cfg.token;
  $("#pwState").textContent = cfg.has_password
    ? "已设置密码 (其它浏览器忘记密码时, 仍可用上面的 token 登录)"
    : "未设置 — 首次打开页面时会要求凭 token 设置密码";
  $("#svcHint").textContent = (cfg.is_root ? "服务以 root 运行 ✓ " : "服务不是 root — jetson_clocks/USB 初始化/启动会失败, 见 webui/README.md ") +
    "· 热参数端口 127.0.0.1:" + cfg.hot_port + " · 修改监听/端口后需重启服务 (sudo systemctl restart aimbot-webui)";
  const lines = [];
  if (scan.status === "ok") {
    lines.push(`<div class="scan-line">✅ 发现: <b>${scan.profiles.length}</b> 个游戏 profile · <b>${scan.engines.length}</b> 个 engine · <b>${scan.onnx.length}</b> 个 onnx · <b>${scan.cameras.length}</b> 个采集设备 · dataset 子目录 <b>${scan.dataset_dirs.length}</b> 个</div>`);
    lines.push(`<div class="scan-line">bin/aimbot: ${scan.binary.exists ? "存在 (" + fmtSize(scan.binary.size) + ", " + fmtDate(scan.binary.mtime) + ")" : "不存在"} · 热参数支持: ${scan.binary.hot_capable ? "是" : "否 (旧版, 建议重编译)"}</div>`);
    (scan.warnings || []).forEach(w => lines.push(`<div class="scan-warn">⚠ ${esc(w)}</div>`));
  } else {
    lines.push(`<div class="scan-warn">✗ ${esc(scan.error)}</div>`);
  }
  $("#scanSummary").innerHTML = lines.join("");
  $("#rootState").textContent = "根: " + cfg.deploy_root + (scan.status === "ok" ? " ✓" : " ✗");
}

/* ================= WebSocket ================= */
function connectWS() {
  if (S.ws) { try { S.ws.close(); } catch (e) { } }
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws?token=${encodeURIComponent(S.token)}&since=${S.wsSeq}`);
  S.ws = ws;
  ws.onopen = () => { S.wsBackoff = 1000; $("#wsState").textContent = "连接: 实时 ✓"; };
  ws.onmessage = ev => {
    const m = JSON.parse(ev.data);
    if (m.type !== "tick") return;
    if (m.log && m.log.length) { appendLogs(m.log); S.wsSeq = m.log[m.log.length - 1][0]; }
    if (m.fps_new) {
      if (m.fps_new.from === 0) S.fpsHist = m.fps_new.items;   // 重连后整表替换
      else S.fpsHist.push(...m.fps_new.items);
      renderFpsPanel();
    }
    if (m.scan) {
      S.state.scan = m.scan;                 // 服务端 ~5s 重扫: 脚本被 SSH 改动能到达页面
      if (!S.dirty) selectProfile(S.selected, { force: true });   // 表单干净 → 自动跟随脚本现值
      else renderParamsMeta();               // 有未保存改动 → 不覆盖正在编辑的表单
    }
    S.state.instance = m.instance;
    S.state.tasks = m.tasks;
    if (m.telemetry) S.state.telemetry = m.telemetry;
    if (m.task_log) m.task_log.forEach(tl => appendTaskLog(tl.id, tl.lines));
    const prevInst = S._lastInstSig || "";
    const sig = JSON.stringify([m.instance.state, m.instance.profile, m.instance.exit_code, m.instance.abnormal, m.instance.hot_capable]);
    renderTopbar();
    renderRunTab();
    renderParamsMeta();
    renderOps();
    if (sig !== prevInst && prevInst) renderParams();   // 实例状态翻转 (如热参能力) → 刷新 🔥 徽标
    if (sig !== prevInst && m.instance.state === "exited" && m.instance.abnormal && prevInst && document.hidden && Notification && Notification.permission === "granted") {
      new Notification("aimbot 异常退出", { body: (m.instance.display_name || m.instance.profile || "") + " " + exitText(m.instance) });
    }
    S._lastInstSig = sig;
  };
  ws.onclose = () => {
    $("#wsState").textContent = "连接: 断开, 重试中…";
    setTimeout(connectWS, S.wsBackoff);
    S.wsBackoff = Math.min(S.wsBackoff * 2, 10000);
    // 断线期间状态可能变化, 重连前全量对齐一次
    fetchState().catch(() => { });
  };
}

/* ================= Tab ================= */
function showTab(name) {
  $$("#tabs button[data-tab]").forEach(b => b.classList.toggle("on", b.dataset.tab === name));
  ["run", "params", "ops", "settings"].forEach(t =>
    $("#tab-" + t).classList.toggle("hidden", t !== name));
  if (name === "run") loadCmdPreview();
}

/* ================= 复制 profile ================= */
function openCopyModal() {
  const p = curProfile();
  if (!p) return;
  $("#copyFile").value = p.file + "_copy";
  $("#copyErr").classList.add("hidden");
  $("#copyModal").classList.remove("hidden");
  $("#copyFile").focus();
}
async function doCopyProfile() {
  const p = curProfile();
  try {
    const r = await api(`/api/profiles/${encodeURIComponent(p.file)}/copy`, {
      method: "POST",
      body: { file_name: $("#copyFile").value.trim() },
    });
    $("#copyModal").classList.add("hidden");
    toast("已创建副本: " + r.file + ".sh (参数随源)", "ok");
    if (S.dirty && !confirm("当前有未保存改动, 切到新副本将丢弃。继续?")) { fetchState(); return; }
    S.dirty = false;
    S.selected = r.file;
    await fetchState();
    selectProfile(r.file, { force: true });
  } catch (e) {
    $("#copyErr").textContent = e.message;
    $("#copyErr").classList.remove("hidden");
  }
}

/* ================= 事件绑定 ================= */
function bindEvents() {
  $("#tabs").addEventListener("click", e => {
    const b = e.target.closest("button[data-tab]");
    if (!b) return;
    if (S.dirty && b.dataset.tab !== "params" &&
        !confirm("参数有未保存改动 (已保存在表单里不会丢, 但未写入 profile)。仍要离开?")) return;
    showTab(b.dataset.tab);
  });
  $("#themeBtn").addEventListener("click", toggleTheme);
  $("#themeSel").addEventListener("change", e => { S.theme = e.target.value; applyTheme(); });

  $("#profileSel").addEventListener("change", e => selectProfile(e.target.value));
  $("#btnStart").addEventListener("click", startInstance);
  $("#btnStop").addEventListener("click", stopInstance);
  $("#btnCalib").addEventListener("click", triggerCalib);
  $("#btnSave").addEventListener("click", () => saveProfile().catch(e2 => toast("保存失败: " + e2.message, "err")));
  $("#btnSave2").addEventListener("click", () => saveProfile().catch(e2 => toast("保存失败: " + e2.message, "err")));
  $("#btnRevert").addEventListener("click", () => selectProfile(S.selected, { force: true }));
  $("#btnCopyProfile").addEventListener("click", openCopyModal);
  $("#btnCopyCancel").addEventListener("click", () => $("#copyModal").classList.add("hidden"));
  $("#btnCopyOk").addEventListener("click", doCopyProfile);

  // 参数编辑 (事件委托)
  $("#paramGroups").addEventListener("input", e => {
    const k = e.target.dataset.pk;
    if (!k) return;
    const d = PARAM_DEFS.find(x => x.key === k);
    if (!d) return;
    if (d.type === "num") S.params[k] = parseFloat(e.target.value) || 0;
    else if (d.type === "bool") {
      S.params[k] = e.target.checked;
      if (d.captureSwitch) renderParams();          // 开关控制整组显隐
    } else S.params[k] = e.target.value;
    if (k === "output_mode") {                       // 模式决定 -D 与 -P/-T 哪一组可见
      renderParams();
      // 切槽 = 换显示的另一组 5 个值; 保存只提交本槽, 所以另两槽未保存的改动不会被提交
      const other = Object.keys(MODE_SLOT_KEYS).filter(m => m !== S.params.output_mode)
        .filter(m => Array.from(MODE_SLOT_KEYS[m]).some(x => !sameVal(S.params[x], S.savedParams[x])));
      if (other.length) toast("已切到 " + S.params.output_mode + " 槽: 保存只写本槽的 5 个值 — " +
        other.join("/") + " 槽未保存的改动不会被提交", "warn");
    }
    if (d.type !== "bool") markDirtyLite(k);
    markDirty();
  });
  function markDirtyLite(k) {
    const row = $(`[data-row="${k}"]`);
    if (row) row.classList.toggle("changed", !sameVal(S.params[k], S.savedParams[k]));
  }

  // ⓘ 说明
  $("#paramGroups").addEventListener("click", e => {
    const b = e.target.closest("[data-info]");
    if (!b) return;
    const key = b.dataset.info;
    const info = $(`[data-info-of="${key}"]`);
    if (!info) return;
    const open = info.style.display === "none";
    info.style.display = open ? "block" : "none";
    if (open) S.openInfo.add(key); else S.openInfo.delete(key);
  });
  // 选用 engine
  $("#engineList").addEventListener("click", e => {
    const b = e.target.closest("[data-use-engine]");
    if (!b) return;
    S.params.model = b.dataset.useEngine;
    markDirty();
    renderParams();
    toast("已选用 " + b.dataset.useEngine + " — 记得点【保存】(或直接【启动】, 会先保存)", "");
  });

  // 运维按钮
  $("#btnConvert").addEventListener("click", async () => {
    try { await api("/api/tasks", { method: "POST", body: { kind: "convert" } }); toast("convert 任务已开始, 日志见下", "ok"); renderOps(); }
    catch (e) { toast("convert 启动失败: " + e.message, "err"); }
  });
  $("#btnCompile").addEventListener("click", async () => {
    try { await api("/api/tasks", { method: "POST", body: { kind: "compile" } }); toast("compile 任务已开始", "ok"); renderOps(); }
    catch (e) { toast("compile 启动失败: " + e.message, "err"); }
  });

  // 日志工具
  $("#logFilter").addEventListener("input", rerenderLog);
  $("#logFollowBtn").addEventListener("click", () => {
    S.follow = !S.follow;
    $("#logFollowBtn").textContent = "跟随: " + (S.follow ? "开" : "关");
    if (S.follow) { const b = $("#logBox"); b.scrollTop = b.scrollHeight; }
  });
  $("#cmdDetails").addEventListener("toggle", () => { if ($("#cmdDetails").open) loadCmdPreview(); });
  $("#cmdCopy").addEventListener("click", () => {
    navigator.clipboard.writeText($("#cmdPreview").textContent)
      .then(() => toast("已复制启动命令", "ok"))
      .catch(() => toast("复制失败 (浏览器权限)", "err"));
  });

  // 设置
  $("#btnRootSave").addEventListener("click", async () => {
    try {
      await api("/api/config", { method: "POST", body: { deploy_root: $("#rootInput").value.trim() } });
      toast("部署根已保存并重新扫描", "ok");
      await fetchState();
    } catch (e) { toast("保存失败: " + e.message, "err"); }
  });
  $("#btnRescan").addEventListener("click", async () => {
    try { S.state.scan = await api("/api/scan", { method: "POST" }); renderAll(); toast("已重新扫描", "ok"); }
    catch (e) { toast("扫描失败: " + e.message, "err"); }
  });
  $("#btnSvcSave").addEventListener("click", async () => {
    try {
      const r = await api("/api/config", { method: "POST", body: { bind: $("#bindSel").value, port: parseInt($("#portInput").value, 10) } });
      toast(r.needs_restart ? "已保存; 重启服务后生效: sudo systemctl restart aimbot-webui" : "已保存", "warn");
      await fetchState();
    } catch (e) { toast("保存失败: " + e.message, "err"); }
  });
  $("#btnTokenShow").addEventListener("click", () => {
    const i = $("#tokenInput");
    i.type = i.type === "password" ? "text" : "password";
  });
  $("#btnTokenCopy").addEventListener("click", () => {
    navigator.clipboard.writeText($("#tokenInput").value).then(() => toast("已复制 token", "ok"));
  });
  $("#btnTokenReset").addEventListener("click", async () => {
    if (!confirm("重置 token? 当前这台浏览器的连接也会失效 (本页会自动换新 token)。" +
        "其它已打开的页面将要求重新登录 (密码或新 token)。")) return;
    try {
      const r = await api("/api/token/reset", { method: "POST" });
      S.token = r.token;
      localStorage.setItem("wb_token", r.token);
      $("#tokenInput").value = r.token;
      toast("token 已重置 (密码不受影响)", "ok");
      connectWS();
    } catch (e) { toast("重置失败: " + e.message, "err"); }
  });

  // 密码 (登录态内修改; token 持有者也可改, 与重置 token 同级)
  $("#btnPwSave").addEventListener("click", async () => {
    const pw = $("#pwNew").value;
    if (pw.length < 4) { toast("密码至少 4 位", "err"); return; }
    try {
      await api("/api/auth/password", { method: "POST", body: { password: pw } });
      $("#pwNew").value = "";
      S.state.config.has_password = true;
      $("#pwState").textContent = "已设置密码 (其它浏览器忘记密码时, 仍可用上面的 token 登录)";
      toast("密码已更新", "ok");
    } catch (e) { toast("设置失败: " + e.message, "err"); }
  });

  // AI FPS 历史面板
  $("#fpsHistCard").addEventListener("toggle", renderFpsPanel);

  // 参数导出 / 导入 (纯前端; 导入只进表单, 保存后才写回脚本)
  $("#btnExportParams").addEventListener("click", () => {
    const data = JSON.stringify({ params: S.params }, null, 2);
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([data], { type: "application/json" }));
    a.download = (S.selected || "profile") + ".params.json";
    a.click();
    URL.revokeObjectURL(a.href);
  });
  $("#btnImportParams").addEventListener("click", () => $("#importFile").click());
  $("#importFile").addEventListener("change", async e => {
    const f = e.target.files[0];
    e.target.value = "";
    if (!f) return;
    try {
      const doc = JSON.parse(await f.text());
      const src = doc.params || doc;
      const clean = {};
      PARAM_DEFS.forEach(d => { if (d.key in src) clean[d.key] = src[d.key]; });
      S.params = Object.assign(blankParams(), clean);
      renderParams();
      markDirty();
      // 导入文件可能含另两输出槽的键 —— 它们进表单但对当前模式不可见, 保存也不提交
      const cur = S.params.output_mode || "hid";
      let foreign = 0;
      Object.keys(MODE_SLOT_KEYS).forEach(m => {
        if (m === cur) return;
        MODE_SLOT_KEYS[m].forEach(k => { if (!sameVal(clean[k], S.savedParams[k])) foreign++; });
      });
      toast("已导入参数到表单 — 检查后点【保存】写回脚本" +
            (foreign ? " (文件里有 " + foreign + " 项属于另两输出槽: 保存只写当前模式那一槽, 需要时切模式后再导入)" : ""), "ok");
    } catch (err) { toast("导入失败: " + err.message, "err"); }
  });

  // 登录浮层
  $("#btnTokenOk").addEventListener("click", submitAuth);
  $("#btnAuthSwitch").addEventListener("click", () =>
    showAuthModal(Auth.mode === "token" ? "password" : "token"));
  $$("#tokenModal input").forEach(i =>
    i.addEventListener("keydown", e => { if (e.key === "Enter") submitAuth(); }));

  window.addEventListener("beforeunload", e => {
    if (S.dirty) { e.preventDefault(); e.returnValue = ""; }
  });
}

/* ================= 启动 ================= */
async function boot() {
  try {
    await fetchState();
    connectWS();
  } catch (e) {
    if (e.message !== "需要登录") toast("加载失败: " + e.message, "err");
  }
}
// URL 直达: http://<ip>/?token=xxx → 存 localStorage 并清掉地址栏里的 token
const urlTok = new URLSearchParams(location.search).get("token");
if (urlTok) {
  S.token = urlTok;
  localStorage.setItem("wb_token", urlTok);
  history.replaceState(null, "", location.pathname);
}
bindEvents();
applyTheme();
if (S.token) boot(); else initAuth();
setInterval(pollCapFolder, 10000);
