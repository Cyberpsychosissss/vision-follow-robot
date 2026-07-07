# vision-follow-robot

视觉跟随机器人:双目相机检测行人 → 估计**真实米距 / 方位** → CAN 控制底盘,与目标保持 **2~4m** 跟随。
运行平台:Jetson AGX Xavier + Apollo + ZKHY 双目立体相机 + fr 车型底盘(CAN)。

## 仓库结构

| 目录/文件 | 内容 |
|---|---|
| `code/` | **核心代码**,按 感知→检测→控制→编排 分层(详见 [`code/README.md`](code/README.md)) |
| `code/perception/` | 双目采集 + 视差→真实米深度(`Z = focus×baseline/视差`) |
| `code/detection/` | YOLOv5s(TensorRT FP16,~62fps)检人 → 挑目标 → 采深度 |
| `code/control/` | 跟随控制律(EMA 平滑 + 连续斜坡 + slew 限幅)+ CAN 控车(硬限速 / 死人开关 / 默认 dry-run) |
| `code/orchestration/` | 网页面板(启停感知与跟随 / 实时预览 / 录制 / **Demo视频** / 速度距离热调)+ 一键启停/还原脚本 |
| `code/pending_updates/` | (空)写好未上车的新版放这里;平滑化版已于 2026-07-07 部署 |
| `follow_data_collector/` | Phase1 数据采集器(**只监听 CAN、绝不控车**) |
| `experiment_plan.html` / `todo.html` | 实验计划与待办清单 |

## 架构特点

- **全文件式 IPC,不走 ROS**:本车 Apollo 的 roscpp `advertise()` 会段错误,
  故感知数据走「grabber 写文件 → 各模块读文件」,用墙钟时间戳天然对齐。
- **距离来自双目视差**:相机不通过 SDK 吐标定常数,改用其自报障碍物距离**自标定** focus,
  再逐像素 `Z = focus×baseline/视差` 算真实米深度。

## 三阶段路线

1. **Phase 1** — YOLO 检人 + 规则控制律(当前主线)
2. **Phase 2** — 端到端 CNN+GRU(图像 → 转向)
3. **Phase 3** — VLA 语义跟随

## 安全底线

控车默认 **dry-run 不发帧**;默认限速 0.4 m/s,面板可热调(>0.6 需二次确认),绝对天花板 1.5 m/s=固件上限;死人开关;
`--arm` 才真发 CAN(且需车轮架空 / 充电枪拔出 / 急停释放)。
