# 视觉跟随机器人 · 核心代码

小车用双目相机检测行人,算出距离/方位,控制底盘 CAN 与人保持 2~4m 跟随。
本目录是**整个跟随功能的核心代码**,按"感知→检测→控制→编排"分层存放。

> ⚠️ `control/` 和 `orchestration/` 里的代码 = **车上当前运行的版本(旧版)**。
> 跟随平滑化的新版 + 一键脚本在 `pending_updates/`(**还没部署到车**),详见文档附录。

---

## 目录结构

```
code/
├── perception/              感知层(容器内, C++)
│   ├── zkhy_grabber.cpp       双目相机 → 左目图 + 真实米深度 + 障碍物, 写文件
│   └── disp_calib.py          视差自标定: 用相机自报障碍物距离反推 focus
├── detection/               检测层(容器内, GPU)
│   ├── yolo_follow.cpp        YOLOv5s 检人 → 挑最大框 → 采深度 → 写 target.json
│   ├── run_yolo_follow.sh     启动脚本(设 LD_LIBRARY_PATH)
│   ├── CMakeLists.txt         编译配置(aarch64/TRT6/sm_72 补丁)
│   └── VENDOR.md              依赖的 tensorrtx 版本说明
├── control/                 控制层(宿主机, Python) —— 车上当前版
│   ├── follow_controller.py   读 target.json → 控制律 → 调 car_control 发 CAN
│   └── car_control.py         ctrl_cmd 编码 + 硬限速 + 死人开关 + dry-run
├── orchestration/           编排/界面(宿主机) —— 车上当前版
│   ├── web_ui.py              :8080 网页面板: 启停感知/跟随 + 实时预览 + 录制
│   └── bms_monitor.py         电池监控(写 runtime/bms.json)
├── alt_target_source/       备选目标源(不走 YOLO, 用相机自带检测)
│   ├── parse_zkhy_obs.py      解析相机 /apollo/zkhy_obs 行人话题
│   └── obstacles_to_target.py 读 obstacles_latest.json → target.json
├── data_collection/         数据采集(Phase2 训练数据)
│   └── cam_collector.py       容器内 py2 早期采集器(当前跟随用 web_ui 录制, 此为参考)
└── pending_updates/         ★ 待部署: 跟随平滑化新版 + 一键脚本(还没上车)
    ├── follow_controller.py   新版: EMA滤波 + 连续斜坡 + slew限幅 + 丢帧宽限
    ├── web_ui.py              新版: 感知提到9fps/15hz + 录 target.csv
    ├── start_follow.sh        一键启动全栈
    └── stop_follow.sh         一键停止全栈
```

---

## 数据流(全部走共享文件, 不走 ROS)

> 这台车 Apollo 的 roscpp `advertise()` 一调就 segfault,所以感知数据不发 ROS 话题,
> 而是 **grabber 写文件 → 各模块读文件**。文件都在 `runtime/`(容器与宿主机同一物理目录)。

```
[双目相机 192.168.1.251]
        │ 私有协议(ZKHY SDK)
        ▼
 zkhy_grabber (容器内)  ──写──▶  runtime/grab/
        │                          ├ left_latest.ppm      左目图
        │                          ├ depth_latest.pgm     16bit 米深度(mm)
        │                          ├ disparity_latest.pgm 视差灰度
        │                          ├ obstacles_latest.json 相机自带障碍物
        │                          └ camera_status.json   fps/标定
        ▼
 yolo_follow (容器内, GPU)  ──读 left+depth, 写──▶  runtime/target.json
        │                          {dist_m, lateral_m, off_x, bbox, conf, ...}
        ▼
 follow_controller (宿主机)  ──读 target.json──▶  控制律  ──▶ car_control ──▶ CAN(can0)
        │                                                        ctrl_cmd_98c4d2d0
        └──写──▶ runtime/follow_status.json
        ▲
 web_ui (宿主机, :8080)  ──读全部状态文件, 显示 + 启停按钮 + 录制──┘
```

## 各层一句话

| 层 | 文件 | 职责 |
|---|---|---|
| 感知 | `zkhy_grabber.cpp` | 唯一接触相机的进程。出左目图 + **真实米深度**(`Z=focus×baseline/视差`) + 障碍物 |
| 感知 | `disp_calib.py` | 自标定:相机自报障碍物距离准 → 反推 focus,**用户不用手量** |
| 检测 | `yolo_follow.cpp` | GPU 跑 YOLOv5s(16ms/62fps)检人,挑最大框,在框中心采深度 → `target.json` |
| 控制 | `follow_controller.py` | 读 target → 算(速度,转向)→ 写 follow_status,默认 dry-run,`--arm` 才真发 |
| 控制 | `car_control.py` | 把(速度,转向)编成 `ctrl_cmd` CAN 帧,50Hz 发,硬限速 0.4m/s + 死人开关 |
| 编排 | `web_ui.py` | 手机/浏览器开 `:8080`,一键启停感知与跟随,看实时视频+YOLO框,录数据 |
| 编排 | `bms_monitor.py` | 解 BMS 帧,显示电量 |

详细的连接、启动、原理见同目录文档 PDF:**《视觉跟随-操作与原理.pdf》**。
