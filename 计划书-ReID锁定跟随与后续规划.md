# 特定目标锁定跟随（今天实施）+ 手势 / 语音远程 / 自主路线（规划）

## 执行进度（2026-07-15 傍晚更新）

- ✅ **Step 0** 基线栈验证通过（相机 ~12fps；车在充电，ARM 前拔枪）
- ✅ **Step 1** OSNet ONNX 导出 → `models/osnet_x0_25_msmt17.onnx`（873KB，512维，opset11/**IR7**；导出前剔除 4101 类 `classifier.*`；torch-vs-onnx cos=1.00000）
- ✅ **Step 2** `follow_yolo2026` 容器删掉重建同名 + 挂载 follow_data（用户点头后执行）
- ✅ **Step 3** 代码全部完成并本地验证 15/15（selector/yolo_follow.cpp candidates/web_ui 锁定 UI/启停脚本）
- ✅ **Step 4** 部署完成：车上重编冒烟通过、**真人素材标定阈值**（同人 cos 0.716~0.773 / 异人 0.322~0.435，间隙 +0.281 → REACQ=0.60、APP_MIN=0.50）、Xavier CPU 16.5ms/人；**选择器已在车上跑起来**（修掉冷启动自杀 bug：det 没首帧时曾打自己的钟写 target → 双写自检误杀自己；现在干等首帧 + 自检需先确认感知活过）
- ✅ demo 录像 UI 适配 ReID（主人绿框+像%、候选灰框、锁定状态 chip、CSV 补 locked/track/lock_conf 列）+ 修冻帧假活/收尾谎报两个 bug
- 🔴 **当前卡点（Step 5 前必修）**：面板整页 JS 语法错误 → 浏览器上全部数据"—"、按钮无响应（服务端 /status 正常）。根因已定位：`lockConfirm()` 的 `confirm('...')` 字符串里有**真实换行符**，整个 `<script>` 块解析失败，`tick()` 根本没定义。web_ui.py 一处小修 + 重传重启即好
- ⏳ 下一步：修面板 JS → Step 5 停车场锁定测试 → Step 6 户外 ARM（**先把 max_speed 从 2.0 调回 ≤0.8、保持距离 1~3 调回 2~4**）→ Step 7 收尾（文档/PDF/commit/记忆）

## Context

用户今天要做「识别特定目标的自动跟随」：只跟注册过的主人，路人再近再大也不抢目标，丢失后只重捕获主人。用户点名**直接上神经网络**（参考之前 YOLO 的部署方式）。另外三件事出规划：手势识别（挥手=招车过来、双手举=急停）、手机语音控制（用户定：**先做 web 页，不做小程序**）、自主路线规划能力评估。

**今天已核实的车况**（只读检查完成）：
- 车 `nvidia@192.168.1.102` ping/ssh 均通；车今早刚重启，跟随栈**没在跑**；无其他远程用户；NVMe 剩 264G
- 两容器都在：`apollo_dev_nvidia`（TRT6 编译台）、`follow_yolo2026`（onnxruntime 1.10 + numpy 1.19 + pillow，**但没挂载 follow_data 目录，需重建加挂载**）
- 车上 bin/ 三关键文件与仓库 md5 **完全一致** → 仓库是真源，Mac 开发→scp 部署
- 车外网通（pypi 可达）；宿主机 cv2 3.2/numpy 1.13（**别在宿主机 pip 装东西**，会连累 web_ui 的 cv2）
- 资源：AutoDL 4090 **已验证可连**（`ssh -p 30324 root@connect.westb.seetacloud.com`，密码用户已给；**密码不写进任何文档/仓库**；GPU 在、老会话文件在，非交互 shell 无 `python` → 用 python3/conda 环境）；用户有 VPS（可开随时销毁的 docker）

**为什么不走 TRT6 GPU 跑 ReID**：之前 YOLO 是 tensorrtx 手搓网络绕开 TRT6 老解析器；OSNet 无现成 TRT6 手搓实现（omni-scale 多流+门控，手写量大）。OSNet_x0_25 裁剪图在 Xavier CPU（onnxruntime）预估 20-60ms/crop，对 10Hz 选择器够用 → **CPU ORT 为主，性能不够再二期手搓**。

## 架构（Path 1：插一层选择器，控制器零改动）

```
grabber ──left_latest.ppm/depth_latest.pgm──▶ yolo_follow (改: --out detections.json, 输出 top6 candidates)
                                                    │
                              detections.json ──▶ target_selector.py (新, follow_yolo2026 容器, OSNet ReID)
                                                    │
                                              target.json (契约不变 + locked/lock_conf/track 字段)
                                                    │
                                        follow_controller.py (零改动) ──▶ CAN
```

- 选择器挂了 → target.json 变旧 → 控制器 COAST→SEARCH 安全停（现有机制兜底）
- 未锁定时选择器=透传最大框，行为与今天完全一致
- 丢失后 SEEK/RETURN 照旧；**重捕获只认 gallery 匹配**（修掉"重捕获任意人"的老弱点）

## 今天实施步骤

### Step 0 — 基线起栈（15min）
车上 `bin/start_follow.sh` 起旧栈。验证：面板 :8080 相机 fps>10、target.json 在刷新、无人占相机。

### Step 1 — 4090 导 OSNet ONNX（~40min，可与 Step 3 并行）
- 4090 上：pip 装 boxmot（或 torchreid），取 **osnet_x0_25_msmt17** 权重，导 ONNX **opset 11**、输入固定 `1×3×256×128`，输出 512 维
- 权重下载若卡 Google Drive → boxmot 的 GitHub release 资产兜底
- 4090 上用 onnxruntime CPU 冒烟：同人两 crop 余弦 > 异人余弦
- scp 回 Mac（现有 expect 方式）→ `models/osnet_x0_25.onnx` → `car_scp.sh` 推到车 `follow_data/models/`

### Step 2 — 重建 follow_yolo2026 容器（15min）
- `docker rm -f follow_yolo2026` → 原 recipe 重建 **加挂载** `-v /home/nvidia/work/AutoApollo/apollo/follow_data:/apollo/follow_data`（/work 挂载保留）
- pip 环境恢复（清华源、pip<22、`OPENBLAS_CORETYPE=ARMV8` + `PYTHONIOENCODING=utf-8` 写回 /etc/environment）：onnxruntime==1.10.0 numpy pillow
- 验证：容器内加载 osnet onnx 推随机输入，单 crop 耗时打印（目标 <80ms；超了→输入降 128×64 或隔帧 embed）

### Step 3 — Mac 写代码（2-3h，全部进仓库）
1. **`code/detection/yolo_follow.cpp`**（改动最小化，一次重编）：
   - 加 `--out` 参数（**默认仍 target.json** 保回滚兼容）；新 web_ui 传 `--out detections.json`
   - 输出 `candidates:[{bbox,conf,off_x,box_h_norm,dist_m?},...]` 按面积 top6，每框复用 `sample_depth_mm`
   - 顶层字段保持=最大框（老契约原样）
2. **`code/detection/target_selector.py`**（新，容器内跑，PIL+numpy+ORT）：
   - 循环：detections.json 的 ts 变了才处理 → PIL 读 left_latest.ppm → 裁 candidates crop → OSNet embed → 打分
   - 打分 = `0.6·外观余弦(gallery max) + 0.25·IoU(上帧预测框) + 0.15·深度连续性`；丢失>1s 后仅外观
   - 阈值初值：ACCEPT=0.55；重捕获 REACQ=0.65 且连续 3 帧；gallery 追加条件 cos>0.75 且与次高候选 margin>0.15（防漂移/劫持），滚动 10 条，持久化 `runtime/lock_gallery.json`（选择器重启不丢身份）
   - 锁定命令：面板写 `follow_config.json {"lock":true,"lock_ts":<now>}`，选择器**边沿触发**注册当时最大框；重启后见旧 lock_ts 且无 gallery → 报 `NEED_ENROLL` 不静默注册
   - 写 target.json：契约不变 + `locked/lock_conf/track(TRACKING|REACQ|LOST)/candidates`（不含 embedding）；锁定中无匹配 → `valid:false, n_persons=N, track:LOST`（控制器自然 COAST/SEEK/SEARCH）
   - 未锁定=透传最大框
   - **时序细节（压测结论）**：target.json 的 `ts` **透传 detections 的 ts**（不打自己的钟）——选择器卡死时 ts 停走，控制器 0.6s 超时兜底不受骗；管线新增延迟 ~100-180ms，在超时预算内
   - **近距半身框降权**：bbox 贴画面底边且框很大（near_gate 场景，只见腿）时 OSNet 相似度天然掉 → 权重切到 `w_app 0.2 / w_iou 0.6 / w_depth 0.2`（近距歧义小，连续性主导），防贴近时误报 LOST
   - **重捕获防误锁**：REACQ 的连续 3 帧必须是**同一候选**（帧间 IoU 关联），不是每帧各自过线就行——SEEK 扫过路人时最危险的时序就在这
   - **fail-loud 自检**：detections.json 停更 >2s 而 target.json 还在被别人刷新（=旧二进制在直写 target.json 的双写打架）→ 选择器打日志并退出，绝不静默共写
   - PIL 读 left_latest.ppm 可能撞上半写帧 → try/except 跳帧
3. **`code/orchestration/web_ui.py`**：
   - YOLO_CMD 加 `--out detections.json`；新增 SEL_CMD（docker exec -d follow_yolo2026 …）；perception_start/stop 管 3 个进程 + 面板加选择器运行灯
   - POST `/lock` `/unlock`（合并写 config）；面板「🔒锁定我 / 解锁」按钮 + 锁定状态徽章（lock_conf）
   - 视频叠加层画所有 candidates（灰框）+ 锁定目标（绿框）
4. **`start_follow.sh` / `stop_follow.sh`** 同步管选择器

### Step 4 — 部署 + 重编 + 冒烟（~40min）
- 备份：车上 `bin/backup_20260715/`（沿用 restore_old.sh 链）+ 容器内老二进制 `yolo_follow.bak_20260715`
- **部署顺序有讲究（防双写打架）**：先 scp cpp → 容器重编 → `--once` 冒烟通过 → **再**换 web_ui/selector/脚本并重启栈。禁止"新 web_ui + 旧二进制"组合（旧二进制不认 `--out` 会直写 target.json，和选择器双写；选择器有 fail-loud 自检兜底）
- 冒烟：bus.jpg 当 left_latest.ppm 跑 `--once`，确认 candidates 数组 + 老字段并存；再确认**不带 --out 时行为=旧版**（回滚等价性）
- 回滚组合已验证安全：restore_old.sh（旧 web_ui 不传 --out）+ 新二进制默认写 target.json = 严格旧栈行为

### Step 5 — 停车场 dry-run 锁定测试（~30min）
- 起新栈 → 面板锁定用户 → 双人测试：第二人从中间穿过、走得更近更大 → 面板看绿框不跳、lock_conf 曲线
- 遮挡测试：主人出画 3s 回来，第二人在场 → 只重锁主人
- 看 /tmp/selector 日志：embed 耗时、选择器实际 Hz

### Step 6 — 户外 ARM 实测（~1h，需用户在场）
- 安全检查单：**拔充电枪**（充电互锁）、急停释放、`max_speed` 面板核对（≤0.8 起步；记忆里曾漂移到 1.3）、保持距离核对（用户之前设 1~2m，户外建议 2~4）、场地开阔
- 顺序：单人跟随 sanity → 双人抗干扰 → 丢失-SEEK-重捕获（藏起来再出现）
- 全程录 Demo 视频（现成功能）留证据

### Step 7 — 收尾（固定惯例，每轮必做）
- `docs_src/ops_doc.html` 附录 D 追加本轮「问题/根因/修复」（最新在前）+ §5.4/§6/附录 B 同步；tech_doc 架构图加选择器层；Chrome headless 重渲两 PDF
- 仓库：code/ 与车上 bin/ 保持镜像，commit + push（**密码/IP 不入库**）
- 记忆更新 vision-follow-robot-status.md

## 手势识别规划（用户选定：挥手=招车、双手举=急停）

- 模型：MoveNet singlepose lightning（192×192）ONNX，4090 转好 → 同容器 ORT CPU，**只在锁定目标（或 SEARCH 时最近人）的 crop 上跑** ~5Hz
- 判定 FSM：挥手 = 手腕关键点 1.5s 窗口内相对肘部左右摆动 ≥3 次且腕高于肘；双手举 = 双腕高于鼻持续 0.8s；都要求关键点置信度门槛 + 连续帧确认（防误触）
- 命令通路：`gesture_cmd.json` → follow_controller 消费：
  - **急停 = 锁存 ESTOP**（速度 0 + 状态锁死，面板手动解除才恢复）——纯减风险，无条件生效
  - **招车 = 仅当挥手者 ReID 匹配已注册主人**才把其设为目标并进入跟随（与今天的 ReID 直接联动）
- 里程碑：G1 关键点链路跑通（面板可视化骨架）→ G2 FSM 调误触率 → G3 实车。不需要 GPS/导航能力。

## 语音 + 远程控制规划（web 页，VPS 轮询 GET API，无 frp）

用户定：不用 frp，车**主动外连轮询** VPS——车上不开任何入站端口，VPS 上全是可随时 `docker compose down -v` 销毁的容器。

- **VPS（docker compose 两容器）**：
  - `relay-api`（FastAPI/Flask 小服务）：`POST /api/cmd`（手机下发）、`GET /api/pull?since=`（车长轮询取指令，25s hold）、`POST /api/status`（车心跳+状态）、`GET /api/status`（手机取状态）、`GET /ui`（手机控制页：大按钮 + 按住说话）
  - `caddy`：TLS（有域名→Let's Encrypt 自动；没有→自签+手机信任一次）。**HTTPS 是硬要求**——浏览器麦克风/Web Speech API 只在安全上下文可用
- **车端 `bin/cloud_bridge.py`**（宿主机 stdlib，web_ui 管理启停）：长轮询拉指令 → 白名单转发到本地 web_ui 现有端点；1s 心跳推 status；可选 1fps 快照 jpg
- **语音**：手机端 Web Speech API（zh-CN，本地识别零后端）→ 关键词意图表（停/跟我/近点/远点/快点/慢点/锁定/解锁/回去）→ POST /api/cmd
- **安全规则**：命令白名单；`停车`永远允许；**ARM 默认拒绝**，只有面板开「允许远程」+二次确认才放行；token 双向鉴权 + ts/nonce 防重放；VPS 上不存车的任何凭据
- 延迟预估：长轮询指令到车 0.2~0.5s
- 小程序：缓（用户定）；以后要做时 VPS+TLS 已满足正式版 https 要求
- 里程碑：V1 relay+bridge+按钮页（半天）→ V2 语音意图（半天）→ V3 快照/播报（可选）

## 自主路线规划能力（可以，分四阶；回答用户提问）

结论：**现在这批功能（锁定跟随/招车/急停/语音启停调参）都不需要 GPS/激光雷达级自动驾驶**——全是视觉伺服+双目障碍停车。真"自主规划路线"分阶段：

- **N0 跟随中绕障（本地规划，纯软件）**：现在障碍只会"停"；用 obstacles_latest.json 做转向偏置绕行（简化 VFH：在转向包络内选无障碍方向）。无新硬件，建议下一轮做
- **N1 航迹记录回放「带我回去」**：CAN 已解码 velocity/steering/mileage → 航位推算记 breadcrumb，倒放回走。误差 ~2-5%/程，几十米内可用；和语音"回去"绑定。无新硬件
- **N2 激光雷达 SLAM 导航**：Apollo 配置里有镭神 C16 驱动（launch 文件在；**硬件是否实装下次上车确认**）。坑：车上 roscpp 发布必崩 → 方案 = 旁挂电脑（同 LAN 跑 ROS 节点连车 master，SLAM/规划在旁机，指令走 web_ui HTTP）或容器 py2 rospy 只订阅。Cartographer 建图 + A* 全局 + DWA 局部。工作量最大，室内外通用
- **N3 GNSS 航点导航**：待查车上有无接收机（下次上车看硬件）；没有可加 USB RTK（几百块）走室外航点
- **VLA：用户已定搁置**（车上 JetPack4.3 也装不了：CUDA10 生态太老+共享车禁重刷）。以后若重启，走云端语义层+车端伺服兜底的路线，不动车上系统

**两台远程机器的分工（别混）**：
- **AutoDL 4090**（connect.westb.seetacloud.com:30324）＝一次性模型车间：只用来导 OSNet/MoveNet ONNX，导完即可关机
- **用户 VPS**（另一台，非 4090）＝常驻中转：跑 GET-API relay 的 docker（可随时销毁），以后手机远程控制全走它

## 验收标准（今天）

1. 面板锁定后：第二人从人车之间穿过 / 靠得更近更大，绿框不跳、车不改跟
2. 主人出画 3s 回来（第二人在场）→ 只重锁主人，不锁路人
3. 户外 ARM 连续 10min 跟随含双人干扰，无异常（近距防撞门/SEEK 行为与之前一致）
4. 回滚演练：`restore_old.sh` + 二进制 .bak 能回到旧栈且 target.json 正常

## 风险与备胎

- osnet 权重在 4090 下载卡顿 → boxmot GitHub release 直链兜底
- Xavier CPU embed 超 80ms → 输入降 128×64 / 隔帧 embed（IoU track 补帧）
- TRT6 重编若有意外 → 二进制 .bak 秒回滚；selector 内置 HSV 直方图备胎描述子开关（仅救场用，不作为交付）
- 时间不够 → Step 6 户外压缩为"单人+双人干扰"两项，丢失重捕获挪到明天
