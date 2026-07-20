# 计划书 — 招手召唤自动接近 + 多视角 Demo

> 起草 2026-07-20。**本文是自包含交接文档**：任何 agent 零上下文读完本文即可开工。
> 上一轮见 `计划书-ReID锁定跟随与后续规划.md`（ReID 目标锁定，已完成并部署）。

---

## 执行进度（最新在前）

### 2026-07-20 — 硬件普查完成 + P0 代码完成（**未部署上车**）

- ✅ **P-1 硬件普查**：只读普查两轮，发现车上是四传感器平台。见 §3。
- ✅ **P0.1 `/slate` 时间码页**（`web_ui.py`）：独立 `SLATE_PAGE` r-string 常量 + 新增 `GET /slate`、`GET /now`。
  - 本地实测：对时 offset=1ms/RTT=2ms；色块编码逐格核对与页面显示的 `code` 一致。
  - 踩坑记录：初版 `#frame` 有一圈黑色 padding，解码器按比例内缩剥不干净 → 列投影切出 **18 段而非 16 段** → 解码全灭。改成洋红边框直接贴住灰色色条。
- ✅ **P0.2 demo meta sidecar**（`web_ui.py`）：`_demo_write_meta()` 收尾写 `demo_xxx.meta.json`（起止 ts / fps / 帧数 / 各区 ffmpeg crop 坐标）。单元测试验证三块 crop 正好铺满画布无重叠。顺带修掉 `status()` 里 demo 列表会被同名 `.csv`/`.meta.json` 挤占的问题。
- ✅ **P0.3 `tools/sync_phone.py`**：手机视频 ↔ 车钟对时。
  - 退化鲁棒性实测：缩到 12%、透视 0.20、旋转 ±25°、JPEG q15、偏色过曝全部正确；**零误解码**（失败一律是"拒绝"，丢帧而非污染结果）。
  - 关键修复①：改用 `minAreaRect` 先把色条**摆正**再投影 —— 此前旋转 8° 就解不出来，而手机横屏很难端平。
  - 关键修复②：`CAP_PROP_POS_MSEC` 必须在 `cap.read()` **之后**取，之前取会给每帧少算一帧（30fps 下系统性偏 33ms）。修复后端到端误差 **0.0 ms**。
- ✅ **P0.4 `tools/make_multiview.py`**：一条 ffmpeg `filter_complex + xstack` 出 1920×1080 四宫格，H.264 CRF 23 重编。
  - 合成验证：车 demo 与手机视频各自烧入车钟，成片三个时刻（t=0.5/9.5/19.5s）两侧读数**逐毫秒一致**。
- ✅ 交付自检全绿：`py_compile` ×3、`PAGE` 仍 24410 字符 54 个 id（未碰坏）、引号奇偶、GET 路由无前缀遮蔽、`web_ui` 可导入。
- ✅ `.gitignore` 放行 `/tools` 与 `/计划书-*.md` —— 此前白名单策略把它们（连同上一轮的计划书）全挡在库外，别的 agent clone 下来看不到。`surf_golf/` 保持忽略（含报价金额与业务员手机号）。

- ✅ **P0.5 部署上车**（用户点头后执行）：
  - 备份 `bin/backup_20260720/`（7 个关键文件）；部署前核对车上 `web_ui.py` md5 与仓库 HEAD **完全一致**（无第三方改动）
  - **传输踩坑**：`scp` 走 expect 会挂死（3 分钟超时），`ssh 'cat > dst' < file` 同样挂死，但 `ssh 'bash -s' < script` 一直好用 → **改成把文件 base64 内联进脚本走那条通道**（87KB → 119KB 脚本，一次成功）。两次失败都没写坏车上文件（校验门拦住了 0 字节版本）。**下次传文件直接用 `tools/../push 方式`，别再试 scp**
  - 落地校验：md5 一致 + 车上 `py_compile` 通过 → 原子 `mv` 替换
  - 面板已启动（PID 13063，bms_monitor 仅 1 个无孤儿）；`/` `/slate` `/now` `/status` 全 200
  - **跨机器实测**：Mac 无头 Chrome 渲染车上 `/slate` → 页面报 `offset=143ms RTT=10ms`，独立测得 Mac 钟与车钟实际差 179ms（同量级）→ **对时确实在消这个误差**；解码器解出 `0xC4A` 与页面显示一致

**🔴 唯一待办**：拿真手机拍一次 `/slate` 走完整流程（`sync_phone.py` → `make_multiview.py`），确认端到端 ≤2 帧。需人工操作，可与下次实验合并做。

**下一步**：P1（激光雷达接入 + 局部栅格 + mini-map）。

---

## 0. 给接手 agent 的 30 秒上手

- **这是什么**：一台户外轮式机器人（煜禾森 FR-07 Pro 底盘 + Jetson AGX Xavier），用双目相机做视觉跟随人。本仓库 `~/Desktop/follow_data` 是**唯一真源**，`code/` 与车上 `~/work/AutoApollo/apollo/follow_data/bin/` 保持镜像，Mac 开发 → scp 部署。
- **本轮要做两件事**：① 人招手 → 车自动开过去 → 转跟随（需避障+局部规划）；② Demo 录制加"手机第三人称"视角，四宫格，需时间戳对齐。
- **本轮最大收获**：2026-07-20 只读硬件普查发现**车上远不止相机** —— 16 线激光雷达实装可用、大陆毫米波雷达 CAN 协议已破译、8 个超声波 6 个可用。见 §3。
- **当前进度**：硬件普查完成，代码**一行未动**。下一步 = P0。
- **动手前必读**：§6 环境硬约束、§7 安全规则。**特别注意 `web_ui.py` 的 PAGE 字符串坑（§8）**，上一轮因此整个面板瘫痪过。

---

## 1. 现有系统

### 1.1 数据流（共享文件 IPC，无 ROS）

```
zkhy_grabber (容器 apollo_dev_nvidia, C++, --write-fps 15)
   └─ runtime/grab/{left_latest.ppm, disparity_latest.pgm, depth_latest.pgm(16bit mm),
                    obstacles_latest.json, camera_status.json}
        │
   yolo_follow (容器 follow_yolo2026, TRT6 手搓 YOLO, ~15Hz, --out detections.json)
        └─ runtime/detections.json   (top6 person 候选: bbox/conf/off_x/box_h_norm/dist_m)
             │
   target_selector.py (容器 follow_yolo2026, OSNet x0.25 ONNX + onnxruntime CPU, 20Hz)
        └─ runtime/target.json  (契约: valid/dist_m/lateral_m/off_x/bbox/conf
                                 + locked/lock_conf/track/candidates)
             │
   follow_controller.py (宿主机, python3.6, 40Hz) → car_control.py → CAN can0 @50Hz
        └─ runtime/follow_status.json
             │
   web_ui.py (宿主机, :8080) — 面板 + /stream 视频 + Demo 录制 + 拉起 bms_monitor
```

**关键安全性质**：任一环节挂掉 → 下游文件变旧 → `follow_controller` 的 `TARGET_TIMEOUT=0.6s` 超时 → COAST → SEARCH → 停车。新增模块必须保持这个性质。

### 1.2 关键文件与常量

| 文件 | 要点 |
|---|---|
| `code/control/follow_controller.py` | 40Hz。`DESIRED_MIN/MAX` 保持距离、`near_gate()` 近距防撞门（用人体身高几何给视差距离加上界）、`SEEK/RETURN` 丢失寻回、EMA+slew 平滑、`obstacle_ahead()` 读 `obstacles_latest.json`（**目前障碍只会"停"，不会绕**） |
| `code/control/car_control.py` | `CTRL_CMD_ID=0x98C4D2D0` 扩展帧 50Hz；gear `4=前进 2=倒车 1=驻车`；`MAX_STEER=±25°`；`ABS_MAX_SPEED=2.2 m/s`（硬件顶）；**无使能握手，持续发帧即受控**；默认 dry-run |
| `code/detection/target_selector.py` | OSNet 阈值（07-15 真人标定）：`ACCEPT=0.55 / APP_MIN=0.50 / REACQ=0.60×3帧同一候选`；gallery 滚动 10 条持久化 `lock_gallery.json`；未锁定=透传最大框 |
| `code/orchestration/web_ui.py` | 1443 行。Demo 录制在 `_demo_loop()`/`_demo_compose()`；画布 **1200×900 = 左侧栏 400 + 右上相机 800×450 + 右下视差 800×450**；`DEMO_FPS=10`；**同名 CSV 每视频帧一行且第一列就是墙钟 ts**（多视角对齐的关键既有资产） |
| `code/training/build_dataset.py` | `18C4D2EF` 底盘反馈帧解码（gear/velocity/steering），实时里程推算直接搬这段 |
| `code/orchestration/bms_monitor.py` | 读 candump 解 BMS 的现成范例，新增 CAN 解码模块照抄这个结构 |

---

## 2. 本轮两个需求

### 需求一：招手召唤

人在远处招手 → 车识别到"这是主人在招我" → **自动开过去**（需路径规划 + 避障）→ 到达后转入正常跟随。

拆成三块：**手势识别**、**避障与局部规划**、**HAIL/APPROACH 状态机**。

### 需求二：多视角 Demo

Demo 视频改成四宫格，加入**手机拍的第三人称视角**。需要手机画面与车上数据严格时间戳对齐。

```
┌──────────────┬──────────────┐
│   摄像头      │    视差       │
├──────────────┼──────────────┤
│  手机第三人称  │  俯视 mini-map │
└──────────────┴──────────────┘
```

---

## 3. 硬件底账（2026-07-20 只读普查实测）★

> 此前仓库与文档里**只有相机**的记录。实际是四传感器融合平台。
> 普查脚本见 §附录，可随时重跑。全程只读：未发任何 CAN 帧、未启动任何驱动、未改任何文件。

### 3.1 整车：煜禾森 FR-07 Pro（参数书在 `surf_golf/`）

| 项 | 值 | 用途 |
|---|---|---|
| 长×宽×高 | **1320 × 765 × 1420 mm** | mini-map 车身框、碰撞检查 |
| **轴距 L** | **660 mm** | DWA 自行车模型 |
| 轮距 / 轮径 | 645 mm / 420 mm | |
| **最小转弯半径** | **2 m**（外廓） | DWA 硬约束 |
| 结构 | **前转后驱，阿克曼转向** | 不能原地转，绕障必须提前决策 |
| 质量 / 离地间隙 | 153 kg / 115 mm | |
| 越障 / 跨越 / 爬坡 | 60 mm / 200 mm / 10° | 栅格里多高算障碍 |
| 速度上限 | 8 km/h = **2.22 m/s** | 与代码 `ABS_MAX_SPEED=2.2` 吻合 |

**自洽验算**：`MAX_STEER=25°` → 后轴中心 R = 0.66/tan25° = **1.42 m**，加半车宽 0.38 m + 轮胎 ≈ 外廓 1.9 m ≈ 标称 2 m ✓
→ **DWA 用 1.42 m 做运动学，用车体外廓做碰撞检查，两个数别混。**

出厂标配还有：轮速传感器、前后防撞条、喇叭、转向灯、4G 路由器、USB HUB。
（**有 4G** → 以前规划的 VPS 远程控制不必 frp；**有喇叭** → 手机对时的音频拍板方案有硬件基础。）

### 3.2 镭神 C16 激光雷达 —— 实装、通电、正在实时喷点云 ★

- IP `192.168.1.200`（ARP REACHABLE，MAC `50:3e:7c:20:5f:e5`），eth0 持续 **~840 pkt/s / 1 MB/s**
- **端口是交叉的**（照抄 launch 的 msop/difop 命名会搞反）：
  - 雷达 `:2369` → Xavier `:2368` = **点云 MSOP**，1206 字节，12 个 block 各 100 字节以 `FF EE` 开头
  - 雷达 `:2368` → Xavier `:2369` = **设备信息 DIFOP**，包头 `A5 FF 00 5A`，内含 rpm=600 和自身 IP
- `/apollo/lslidar_c16.launch`：rpm=600 → **10 Hz**，return_mode=1，min_range 0.15 m，max_range 150 m
- 外参（**别人配的，从未验证，用前必须实测标定**）：`lidar_set_z=0.6`（装 0.6 m 高），`qz=1.0/qw=0.0`（绕 Z 轴 180°）
- `DISTANCE_RESOLUTION = 0.01`（米），见容器内 `lslidar_decoder.h:57`
- 垂直角表：**车上没找到 `lslidar_c16_db.yaml`**，需从解码器源码挖，或用 C16 标准 16 角（-15°~+15°，2° 间隔）

**ROS 路线是死路，别再试。** `/apollo/data/log/lslidar_c16.out`（2026-06-25）记录别人跑官方驱动失败：`ros::TimeNotInitializedException` + rosout 进程反复 exit -6。与这台车"C++ roscpp `advertise()` 必崩"的病根同源。

**可行路线已当场跑通**：宿主机 python3.6 直接
```python
s = socket.socket(AF_INET, SOCK_DGRAM); s.bind(("0.0.0.0", 2368))
```
3 秒收到 20 个 1206 字节包，`FF EE` 块头位置 `[0,100,...,1100]` 全对。**不需要 ROS、不需要容器、不需要 roscpp。**

### 3.3 大陆 ARS408-21 毫米波雷达 —— CAN 协议已破译，零成本接入 ★

CAN 上 `61A~61D` 以 ~37Hz 输出。ARS408 目标列表默认 ID 是 `0x60A~0x60D`，**sensor_id 每加 1 全部报文偏移 0x10** → 本车 sensor_id=1。

| CAN ID | 含义 | 已验证 |
|---|---|---|
| `61A` | `Obj_0_Status` | byte0 = 目标数（实测 14~16）；byte1-2 = MeasCounter 单调递增；byte3 高 4 位 = InterfaceVersion=1 |
| `61B` | `Obj_1_General` | byte0 = Obj_ID（`00`→`0B` 轮转，与目标数吻合） |
| `61C` / `61D` | Quality / Extended | 未细解 |
| `211` | `0x201+0x10` = RadarState | |
| `710` | `0x700+0x10` = VersionID | |

**解码验证**（`Obj_1_General` 标准位域：`DistLong=((b1<<5)|(b2>>3))*0.2-500`，`DistLat=(((b2&7)<<8)|b3)*0.2-204.6`）：
```
ObjID 0 → 纵向 3.20 m  横向 0.00 m     ObjID 4 → 9.00 m,  0.20 m
ObjID 1 → 纵向 6.60 m  横向 0.20 m     ObjID 11→ 27.20 m, -0.60 m
```
全部合理。**实施时用官方 dbc 核对 DistLat 位域**（个别远距目标横向解出 -49 m，超 FOV，位域可能差一点）。

价值：250 m / ±60°，直接给**距离 + 横向 + 径向速度 + RCS**，对运动目标敏感、不受雨雾灰尘影响。缺点是分辨率低、静止目标易被滤除。**最佳用途 = 远距目标预警 + 速度估计 + 给雷达/视觉做交叉验证**，不做精细避障。

### 3.4 超声波 8 探头 —— 6/8 确认可用

CAN `301`~`304`，~1.9 Hz，5 字节：`40` 是帧头，后 4 字节是槽位，**每 ID 只用前 2 槽**（4×2 = 8 探头 ✓），`FF` = 无回波。

2026-07-20 真人绕车走动 120 秒实测：

```
301: 槽1  8~90 (46种值)   槽2 13~82 (42种)   ← 响应最活跃, 人退开即回 FF
304: 槽1 39~117(11种)     槽2 38~124(10种)   ← 正常
302: 槽1 28~49 (10种)     槽2 34~44 ( 3种)   ← 弱但确实在动
303: 槽1 恒13             槽2 恒14           ← 正对固定近物被挡死
```

**产品手册 §5.2 的权威规格**（比报价单更准）：工作频率 48 kHz（左右）/ 58 kHz（前后），**探测距离 20 cm ~ 300 cm**，水平 90°±10°，垂直 45°±5°，CAN 接口，12 V。安装角度**尽量朝上**，避免上斜坡时误报。

**单位基本定案 = 2 cm/LSB**：量程上限 300 cm > 单字节 255，所以单位不可能是 cm；2 cm/LSB 时 300 cm = 150，落在字节内 ✓。实测的 `117`/`124` 对应 234/248 cm，量程内合理；`8` 对应 16 cm，略低于 20 cm 下限（近场饱和，正常）。

**仍需在空地确认的两件事**（当前停车位置有静态近距回波干扰）：
1. 单位实测标定（站在量好的距离上核对 2 cm/LSB）
2. **槽位 ↔ 车上探头方位**：需逐个用板子遮挡确认

**建议与 P4 户外实测合并做，不单独跑一趟。** 不做也不阻塞主线。

### 3.5 其他

- **RTK 组合导航**（出厂标配，水平 2cm+1ppm，双天线定向 0.2°，IMU 200Hz）：两个 USB 串口 `/dev/ttyUSB0`(PL2303) / `ttyUSB1`(FTDI) 几乎肯定是它。被动 `cat` 无输出多半是波特率不对。**本轮已决定砍掉，不碰这两个口**（往未知串口写字节有触发设备动作的风险）。
- 已知 CAN：`18C4D2EF`=底盘反馈（gear/velocity/steering，已解码）；`18C4E1EF/E2EF`=BMS（已解码）；`18C4D7EF/D8EF/DAEF/DCEF/DEEF/EAEF`=MCU 广播，内容未知。
- `can1/can2` = Jetson 内置 mttcan，均 DOWN 无接线；另插着 `1d50:606f candleLight USB-CAN` 适配器（别人调试用）。**can0 是唯一在用的总线。**
- **无 `/dev/video*`**（没有额外 USB 摄像头）；`24ae:2015` 是雷柏无线键鼠，无关。
- **车钟已 NTP 同步**（`System clock synchronized: yes`，timesyncd active）→ 手机 slate 对时方案前提成立。
- **无免密 sudo**（tcpdump 用不了）。替代：`/proc/net/dev` 两秒差分看包速率，即可定性判断以太网传感器是否活着。

---

## 4. 已定决策（不要推翻重议）

| 决策 | 结论 | 理由 |
|---|---|---|
| Demo 第 4 格 | **俯视 mini-map** | 与避障功能呼应，最能说明"车在想什么" |
| 手机对时 | **色块 slate 板** | 不依赖手机任何能力，画质用原生相机最好，精度 1 帧 |
| 视频合成位置 | **Mac 侧，不在车上** | 手机视频不在车上；Xavier 用 cv2 3.2 合成是纯亏 |
| RTK | **砍掉** | 两个需求全在车体坐标系内完成，不需要全局定位 |
| 避障主传感器 | **激光雷达** | 360° 消灭"绕行中障碍出摄像头 FOV"的致命缺陷 |
| mini-map 画法 | **cv2 + numpy**，不引 matplotlib | 宿主机只有 cv2 3.2 + numpy 1.13，且禁止 pip 装东西 |
| 手势识别位置 | **并入 `target_selector.py` 同进程** | 帧和候选框它已解好，另开进程读 ppm 是浪费 CPU |
| 局部规划 | **滚动占据栅格 + 自行车模型 DWA** | 尊重最小转弯半径；栅格给绕障"记忆" |
| 里程推算 | **轮速 + 转向角**（`18C4D2EF`），备胎=雷达 scan matching | 只需 3~5 秒尺度一致性，误差厘米级 |

---

## 5. 实施计划

> 每个阶段都留"只看不控"的中间态 —— 这是本项目一贯的安全习惯，别跳过。

### P0 — 多视角 Demo（0.5 天，**不依赖任何硬件结论，建议先做**）

做完之后 P1~P4 每次实验都能录多视角留证据。

**车端改动（`code/orchestration/web_ui.py`）：**

1. 新增 `GET /slate` 全屏时间码页：
   - 中央大号明文 `2026-07-20 14:33:12.480`（人眼可读，自动解码失败时兜底）
   - 下方一排 **12 个黑白色块**编码 `unix_ms & 0xFFF`（4 秒无歧义）+ 两个反相角标块供 Mac 侧自动定位
   - 页面加载时先跟 `/status` 对一次时钟（往返/2 算 offset），**显示的必须是车钟**，不是手机钟 —— 这一步消掉最大误差源
   - 纯 JS `requestAnimationFrame` 绘制，**不引入任何外部库**（车上无外网依赖，且 PAGE 字符串坑见 §8）
2. Demo 收尾写 sidecar `demos/demo_xxx.meta.json`：`{start_ts, end_ts, fps, frames, canvas_layout, csv, git_rev}`
3. `_demo_compose()` 保持 1200×900 不变（车上不合成手机画面）

**Mac 端新增 `tools/`：**

4. `tools/sync_phone.py`：ffmpeg 抽手机视频前 10 秒帧 → 找色块条 → 解码 → 输出 `t_car(手机第0帧)`。失败则弹那一帧让人肉眼读明文手输。
5. `tools/make_multiview.py`：一条 ffmpeg `filter_complex + xstack` 出 1920×1080 四宫格
   - 车 demo 已知布局可直接 crop：相机区 `crop=800:450:400:0`，视差区 `crop=800:450:400:450`，侧栏 `crop=400:900:0:0`
   - 手机流用 `setpts=PTS-STARTPTS+<offset>/TB` 平移对齐
   - **顺带 H.264 CRF 23 重编** —— 现在车上 mp4v 的 demo 单个 837 MB，压完小一个数量级

**拍摄流程**：手机横屏 → 先拍平板/笔记本上的 `/slate` 页 3 秒 → 转身拍车+人。

**验收**：手机画面里人抬手的那一帧，与车 demo 里 `target.json` 出现 wave 的那一帧，误差 ≤ 2 帧。

---

### P1 — 激光雷达接入 + 局部栅格 + mini-map（1.5 天，**只看不控**）

**新增 `code/perception/lslidar_c16_udp.py`（宿主机 python3.6，无第三方依赖）：**

1. 绑 UDP `0.0.0.0:2368` 收 MSOP；解 12 block × 32 通道 → 极坐标 → 车体系点云
   - **C16 的 32 通道打包方式和垂直角表必须按手册/解码器源码核对**（见 §3.2 待办）
   - 距离分辨率 0.01 m
2. 高度过滤（剔地面/剔天空）→ 只留 `0.1 m < 高度 < 1.8 m` 的点
3. 投影成 **robot-centric 占据栅格 8 m × 8 m @ 0.1 m**
4. 里程推算：`candump can0` 解 `18C4D2EF`（照抄 `build_dataset.py` 的 `decode_fdbk`）→ 自行车模型积分 → 栅格随车滚动
5. 输出 `runtime/lidar_grid.json`（或 `.npy`）+ `runtime/freespace.json`

**新增 `code/perception/ars408.py`**：读 candump 解 `61A/61B` → `runtime/radar_objects.json`（零成本，照抄 `bms_monitor.py` 结构）

**`web_ui.py` 新增 mini-map 绘制函数**（面板与 demo 共用同一个）：

960×540 一格，视野 12 m × 6.75 m，车在下方 1/4 处：
- 底：深色 + 极坐标网格（2/4/6/8 m 同心圆弧）
- **激光点云** 灰白 —— **必须 numpy 批量索引赋值** `canvas[ys, xs] = color`，逐点 `cv2.circle` 慢 100 倍
- **占据栅格** numpy → `cv2.resize` + `applyColorMap` 半透明叠加
- **毫米波目标** 黄色方块 + 速度矢量箭头
- **超声波**（P4 标定后）车身四周小扇形，按距离染色
- **锁定的主人** 绿色标记 + ReID 相似度
- **车身** 真实 1.32 × 0.765 m 矩形 + 朝向三角
- **最小转弯半径包络** 左右两条 R=2 m 圆弧
- **DWA 候选轨迹束**（P3 后）灰细线 + 选中那条亮青粗线
- 中文标签沿用 `demo_assets` 预渲染 PNG 机制

**外参标定**：车前方已知位置放物体，看点云里它在哪，校正 launch 里那组未验证的 `z=0.6 / 绕Z轴180°`。

**验收**：面板 mini-map 上，人走动时点云团跟着动；把纸箱放在车前 3 m，栅格上出现对应占据块且位置误差 < 0.2 m。

---

### P2 — 手势识别（1 天，**只出 json 不控车**，可与 P1 并行）

#### 手势词汇表（2026-07-20 用户定，**单手/双手 × 摆动/静止 两两正交**）

正交是刻意的：四个手势在"手数"和"动/静"两个维度上互不重叠，误判空间最小。

| 手势 | 关键点判定 | 含义 | 需 ReID 确认? |
|---|---|---|---|
| **单手挥手** 1.5s | 一侧腕高于肘 + 相对肩横向位移 1~3Hz 过零 ≥3 次 | **召唤 / 认定主人**；暂停状态下也用它恢复 | 冷启动否；已有主人时**是** |
| **单手高举静止** 2s | 一侧腕高于鼻 + 位移方差低于阈值 | **暂停跟随，保留主人锁定**（车停住但记得你是谁） | **是** |
| **双手挥动** 3s | 双腕均高于肘 + 均在摆动 | **解除主人锁定 + 关闭自动跟随**（回到冷启动） | **是** |
| **双手高举静止** 0.8s | 双腕均高于鼻 + 静止 | **急停（锁存）**，面板手动解除 | **否 —— 任何人都能触发** |

**为什么急停必须独立且不要求 ReID**：车 APPROACH 时是朝人开过去的，处于危险中的那个人**未必是主人**（可能是被挡在中间的路人）。安全动作不能设身份门槛。这是"解除主人锁定"替代不了的 —— 后者是正常操作，反而**必须**要 ReID，否则路人挥挥手就能把车从你手里夺走。

**全部手势仅在面板「招手功能」开关 ON 时生效**（默认关）。开关关闭时手势链路照常出 `gesture.json`（供 demo 记录与调试），但控制器一概不消费。

#### 多人同时招手

按**手势置信度**最高者认主（冷启动时没有 gallery，不是 ReID 分）。置信度 = 摆动次数 × 幅度 × 关键点平均 conf 的综合分。

**必须带 margin**：最高分要比次高分高出 `HAIL_MARGIN`（初值 0.15）才认，否则判为"未分出胜负"继续等。没有这条，两个人同时招手会让目标在两人之间来回跳 —— 与 `target_selector.py` 里 gallery 追加用 `GAL_ADD_MARGIN` 防劫持是同一个思路，代码里已有先例可抄。


1. **4090 上导 ONNX**（AutoDL，一次性模型车间，导完关机）：MoveNet SinglePose Lightning 192×192 → tf2onnx → **必须在 ORT 1.10 上验证兼容性**（OSNet 当初被迫用 IR7）。备选 `yolov8n-pose@256`（ultralytics 直接 export，算子更干净）。
2. **`target_selector.py` 集成**：
   - **廉价预筛**：候选框上 1/3 区域帧差能量，1.5 s 环形缓冲，只有过零率落在 1~3 Hz 才进下一步 → MoveNet 平均每秒只跑几次，不挤 OSNet 的 CPU
   - **确认**：在候选 crop 上跑姿态，判定「腕高于肘 + 腕相对肩横向位移 1.5 s 内过零 ≥3 次 + 关键点 conf > 0.3」
   - **双手举 = 急停**：双腕高于鼻持续 0.8 s → **锁存 ESTOP**，无条件生效（不要求 ReID），面板手动解除
   - 输出 `runtime/gesture.json`
3. 面板画骨架可视化，调误触率

**验收**：正常走路/摆臂 5 分钟零误触；主动招手 2 秒内识别率 > 90%。

---

### P3 — DWA 局部规划（1 天，**停车场 dry-run 画轨迹不发帧**）

**新增 `code/perception/local_planner.py`（宿主机）**，消费 P1 的栅格 + `target.json`：

- 在 `(v ∈ [0, v_max], δ ∈ [-25°, +25°])` 网格上采样，**自行车模型**（后轴中心 R = L/tanδ，L=0.66 m）前推 2.5 s
- 代价 = `w1·终点到目标距离 + w2·轨迹最近障碍倒数 + w3·|δ-δ_prev| + w4·速度奖励`
- 输出 `runtime/plan.json {ts, v_ref, steer_ref, blocked, reason, traj[]}`

**`follow_controller.py` 消费**（保持 fail-safe）：
```
plan 新鲜(< 0.5s) 且 面板「避障绕行」开关 ON  → 用 plan 的 steer_ref, 速度取 min(伺服速度, plan 限速)
plan 旧 / 开关 OFF                          → 退回现有纯伺服 + 见障就停（今天的行为）
```

**验收**：停车场摆两个纸箱形成 1.5 m 缝隙，dry-run 下 mini-map 上选中轨迹应从缝隙穿过；缝隙缩到 0.8 m（< 车宽 0.765 m + 余量）时应判定 `blocked`。

---

### P4 — HAIL/APPROACH 状态机 + 实车（1 天，**需用户在场**）

**`follow_controller.py` 新增状态**：`HAIL_WAIT` → `APPROACH` → 到达转 `FOLLOW`

**前提（容易误解，必须写清）**：「自动跟随关闭」**不等于**「控制器没跑」。控制器不 ARM 就根本不发 CAN 帧，招手也驱动不了车；而**让手势去 ARM 车是绝对禁止的** —— ARM 必须是人在面板上按检查单完成的显式动作。真实链路：

```
① 面板: 启动感知 → 启动控制器 → ARM(检查单) → 开「招手功能」开关(默认关)
      ↓
② SEARCH: 已 ARM、能动, 但无跟随目标 → 原地待命不动   ← 这才是「跟随关闭」态
      ↓
③ 有人招手 1.5s → 按 §P2 手势词汇表认主
      ↓
④ APPROACH: 面板可调限速开过去(带避障绕行)
      ↓
⑤ 到达 → 鸣喇叭一声(面板可关, 见下方待解) → 转 FOLLOW + 锁定此人
```

触发条件（**全部满足**）：面板「招手功能」开关 ON（默认关）+ `gesture.json` 新鲜 + 连续 1.5 s + 身份判定：
- **冷启动（gallery 为空）**：置信度最高、且超过次高 `HAIL_MARGIN` 的挥手者 → **注册为主人并立即锁定**
- **已有主人**：只认 ReID 匹配 gallery（cos ≥ `REACQ`）的挥手者，路人招手一概不响应

解除靠「双手挥动 3 s」（需 ReID）或面板「解锁」按钮 → 回到冷启动态。

与 FOLLOW 的差异：
| | 值 |
|---|---|
| 限速 | `HAIL_SPEED` **面板可调**，初值 0.6 m/s（低于 max_speed） |
| 远距限制 | `FAR_LIMIT` 放宽到 15 m（召唤本来就远） |
| 到达判据 | `DESIRED_MIN + 0.3` → 鸣喇叭 → 转 FOLLOW 并锁定该人 |
| 超时 | `APPROACH_TIMEOUT = 30 s` 未到达 → 放弃回 SEARCH |
| 丢目标 | > 2 s 直接放弃（**不走 SEEK 盲走**，速度更高时盲走太危险） |
| 暂停态 | 「单手高举静止 2 s」→ 车停但保留锁定；再「单手挥手」恢复 |

#### 🔶 喇叭：官方控制帧里没有，需先只读逆向

产品手册 §10.2.1 的 `ctrl_cmd 0x18C4D2D0` 报文表**只有 6 个字段**：目标档位、目标车体速度、目标车体转向角、目标车辆制动、Alive Rolling Counter、Check BCC。**没有喇叭、没有灯光。**

（顺带核实：仓库代码里的 `CTRL_CMD_ID = 0x98C4D2D0` 与手册的 `0x18C4D2D0` 差的是 socketcan 扩展帧标志位，`0x98C4D2D0 & 0x1FFFFFFF = 0x18C4D2D0`，实际发出去一致，**不是 bug**。另注：手册表格写周期 10 ms，但同章示例代码 `GetPeriod` 返回 20 ms，我们用的 20 ms 实测可用。）

所以喇叭要么在别的报文里，要么只有硬件/遥控器通路。**绝不能靠猜往 CAN 上发未知帧**（可能触发意外车辆动作）。安全的逆向办法与超声波那次同一个路子：

> **按车上/遥控器的物理喇叭键，同时 `candump` 录制，前后差分看哪个 ID 的哪个字节变了。全程只读，零风险。**

在查清之前，P4 **先只做面板状态提示**，喇叭作为可选项挂在面板开关后面（查到了再接上）。

**顺带做**：超声波空地标定（§3.4 的两件事），半小时。

**实车顺序**：车轮架空 → 停车场 dry-run → 停车场 ARM 低速 → 户外 ARM。全程录多视角 Demo 留证据。

---

### P5 — 收尾（固定惯例，每轮必做，别省）

- `docs_src/ops_doc.html` **附录 D 追加本轮「问题/根因/修复」（最新在前）** + 正文相关节同步
- `docs_src/tech_doc.html` 架构图加雷达/规划层 + 新端点/参数/已知限制
- Chrome headless 重渲两份 PDF 到桌面：
  `chrome --headless --no-pdf-header-footer --print-to-pdf=... file://...`
- 仓库：`code/` 与车上 `bin/` 保持镜像，commit + push（**密码不入库**）
- 更新记忆 `vision-follow-robot-status.md` 与 `car-sensor-inventory.md`

---

## 6. 环境硬约束（违反会出事）

### 连车

- `nvidia@192.168.1.102`（**IP 会变**，先 ping）。Mac 需与车**同一局域网**（192.168.1.x）且 **Clash Verge TUN 关掉**（会偶发劫持局域网 TCP，症状：ping 通但 ssh `No route to host`，重试即可）
- Mac 无 `sshpass`，用 `expect` 喂密码（**密码不写进任何文档/仓库**，走环境变量）
- 投递脚本用 `ssh 'bash -s' < script.sh` —— cmdline 只有 `bash -s`，不会被 `pkill` 误伤
- 远程 `pkill` 含进程名要写 `[w]eb_ui` 防自杀；**`pkill` 与启动命令绝不能放同一条 ssh**

### 车上环境

| | |
|---|---|
| 宿主机 | Jetson AGX Xavier 32GB，Ubuntu 18.04 / L4T R32.3.1（JetPack 4.3）。python3.6 + **cv2 3.2.0** + numpy 1.13 + **python-can 3.3.4**。**无 ROS、无 CUDA toolkit**（onnxruntime-gpu 跑不了，只能 CPU） |
| **禁止** | **别在宿主机 pip 装东西**（会连累 web_ui 的 cv2）。新代码只能用 stdlib + 已装的那几个 |
| 容器 `follow_yolo2026` | py3.6 + onnxruntime 1.10 CPU + numpy + pillow，**无 cv2**。已挂载 `follow_data` |
| 容器 `apollo_dev_nvidia` | Ubuntu 14.04 / py2.7，ROS 版 Apollo 3.0（无 Cyber RT）。**C++ roscpp `advertise()` 必崩** → 一切走文件式 IPC |
| 共享车 | **多人同时在线**，先 `who`。别人会跑 `dev_start.sh` 把 `apollo_dev_nvidia` 删掉重建。别人在用时别启相机驱动、别碰 CAN |
| 磁盘 | 系统盘 28G eMMC（紧），数据放 458G NVMe `/home/nvidia/work`（剩 263G） |
| sudo | **无免密 sudo** |

---

## 7. 安全规则

- **控车默认 dry-run**。`--arm` 才真发帧。
- **ARM 前提**（缺一不可）：车轮架空或场地开阔 → **拔充电枪**（充电互锁）→ 急停释放 → 面板核对 `max_speed`（**≤0.8 起步**，记忆里曾漂移到 1.3/2.0）→ 核对保持距离（户外建议 2~4 m）→ 人在场手持遥控器
- **自动开过去是全项目最危险的功能**（车主动朝人开）。`ESTOP` 锁存优先级最高；`near_gate()` 照常生效；面板常驻急停按钮不动
- 遥控器应一直拿在手上，紧急时立刻切手动模式（厂家 SOP 原话）
- 新增模块必须保持 §1.1 的 fail-safe 性质：**自己挂掉 → 下游停车**，绝不静默降级

---

## 8. 已知遗留 bug（未修，动到相关代码时注意）

1. **`web_ui.py` 的 PAGE 是非 raw `u"""` 字符串** —— JS 字符串里换行必须写 `\\n`，写成单 `\n` 会被 Python 变成真换行 → served JS 字符串未闭合 → SyntaxError → **整个 script 块死 → 所有按钮/tick 全灭**（页面和 /stream 仍显示，极具迷惑性）。上一轮为此瘫痪过一整天。
   - **交付前自检**：`py_compile` + 裸 `\n` 扫描 + 引号奇偶 + id 核对
   - **整页替换用 python splice 脚本，别手拼大 Edit**
2. `yolo_follow.cpp` 的 `const Cand& best = cs[0]` —— persons 非空但候选全被裁剪时对空 vector 取 `[0]` = UB。需加 `if (cs.empty())` 走 `valid:false` 分支 + 容器重编。
3. `web_ui` 每次重启会孤儿化自己带起的 `bms_monitor` → 越积越多。重启面板前先 `pkill -f '[b]ms_monitor'`。
4. `perception_start` 用 `docker exec -d` 起选择器，rc=0 不代表起成功（cd 失败等静默）。靠面板"选择器"灯兜底。
5. 车上 cv2 3.2(aarch64) 的 **`putText` LINE_AA + 粗细 2 会光栅化溢出**，从字形中段画一条贯穿到画布右缘的白线（实测真实值域 26% 触发）。**所有粗细 2 的数值 putText 必须用 `LINE_8`**，细字才可用 AA。

---

## 9. 验收标准（本轮整体）

1. **多视角 Demo**：四宫格成片，手机画面与车上数据误差 ≤ 2 帧；成片体积比车上原始 demo 小一个数量级
2. **mini-map**：面板实时显示激光点云 + 毫米波目标 + 车身 + 转弯包络；纸箱位置误差 < 0.2 m
3. **手势**：正常走路 5 分钟零误触；主动招手 2 秒内识别 > 90%
4. **避障**：1.5 m 缝隙能穿过，0.8 m 缝隙判 blocked 并停
5. **招手召唤**：主人 10 m 外招手 → 车自动接近并停在设定距离 → 转跟随；**路人招手无反应**；全程有障碍时能绕
6. **回滚演练**：`restore_old.sh` 能回到旧栈且 `target.json` 正常

---

## 10. 风险与备胎

| 风险 | 备胎 |
|---|---|
| C16 通道→垂直角映射搞错 → 点云是乱的 | 先用标准 16 角（-15°~+15°/2°）出图，肉眼看墙是不是直的；不对再挖解码器源码 |
| 姿态模型 tf2onnx 转换在 ORT 1.10 上不兼容 | 换 `yolov8n-pose@256`（算子更干净）；再不行退回"帧差周期性"粗判 + 更长确认窗口 |
| Xavier CPU 扛不住 OSNet + MoveNet 同时跑 | 预筛已大幅降频；再不够就 `--top` 降到 3~4，或姿态隔帧跑 |
| 双目/雷达外参不准导致目标与栅格对不上 | P1 就用纸箱标定，别拖到 P3 |
| 阿克曼最小转弯半径导致 DWA 在窄场景无解 | 允许"停下→倒车→重新对准"（已有 `GEAR_REVERSE` 和 RETURN 逻辑可复用） |
| 手机视频 slate 自动解码失败 | 明文大字人肉读一眼手输，5 秒的事 |
| 车 IP 变了 / Clash 劫持 | 先 ping，再看 `ip -br a` |

---

## 附录：可复用的只读诊断脚本

2026-07-20 普查用的三个脚本（在当时会话的 scratchpad，可按本文重建）：

- `hw_survey.sh` — 网口/包速率/CAN ID 普查/USB 串口/Apollo 驱动清单/ROS 话题
- `hw_survey2.sh` — 定向确证：被动收 UDP 2368、读雷达 launch 与历史日志、超声波基线、USB 设备描述符
- `carssh.exp` — expect 包装器，密码走 `CARPW` 环境变量不落盘

**免 root 判断以太网传感器是否活着**（本车无免密 sudo，tcpdump 用不了）：
```bash
awk 'NR>2{gsub(":","",$1); print $1,$2,$3}' /proc/net/dev > /tmp/a; sleep 2
awk 'NR>2{gsub(":","",$1); print $1,$2,$3}' /proc/net/dev > /tmp/b
join /tmp/a /tmp/b | awk '{p=($5-$3)/2; if(p>0.5) printf "%s %.0f pkt/s\n",$1,p}'
```
