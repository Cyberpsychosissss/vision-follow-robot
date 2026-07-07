# pending_updates

## 待部署:控制频率 20→40Hz(2026-07-07 写好,车已断连未上车)

本目录当前的 `follow_controller.py` + `car_control.py` 是**控制提频版**,离线验证过、未上车:

1. `LOOP_HZ` 20→**40Hz**(指令延迟减半 ~12ms,斜坡步长更细腻;CAN 发帧仍 50Hz=fr 协议周期,没动)。
2. slew 限幅改**物理单位**定义(`ACCEL_UP=0.6 m/s²`、`ACCEL_DOWN=1.6`、`STEER_RATE=80°/s`),每周期步长运行时按 LOOP_HZ 换算——以后随便改频率,手感不漂。
3. EMA 改成**只对 target.json 新样本滤一次**(旧版每循环重复滤同一帧,手感会随 LOOP_HZ 变);系数 0.40→0.60(≈旧版等效手感)。
4. car_control `SPEED_SLEW` 0.6→**1.2 m/s/s**(旧值把控制器的减速卡死在 0.6,停车偏肉;加速仍由控制器 0.6 m/s² 管)。

部署(下次上车):
```bash
scp code/pending_updates/follow_controller.py code/pending_updates/car_control.py \
    nvidia@<车IP>:/home/nvidia/work/AutoApollo/apollo/follow_data/bin/
# 然后重启控制器(面板点 DRY-RUN 先看决策)即生效
```
部署完把这两个文件挪回 `../control/` 并清掉本目录。

---

以下为历史记录(2026-07-07 上午已部署):

原先放这里的「跟随平滑化新版」4 文件已于 **2026-07-07 部署上车** 并提升为正式版:

- `follow_controller.py` `car_control.py` → [`../control/`](../control/)
- `web_ui.py` `start_follow.sh` `stop_follow.sh` `restore_old.sh` → [`../orchestration/`](../orchestration/)

部署时的增量(相对 6-29 平滑化版):

1. **速度上限面板热调**:car_control `ABS_MAX_SPEED` 0.6→1.5(=固件上限);web 面板「最高速度」输入框 → `follow_config.json` 的 `max_speed` → follow_controller 热读。默认仍 0.4。
2. **Demo 视频录制**:面板「⏺ 录制 Demo」按钮,YOLO 框 + 左上角实时参数 HUD(状态/距离/横向/下发速度转向/限速/置信度/相机fps)直接烧进 mp4,存 `demos/`,面板可下载。
3. **一键还原**:部署前旧版备份在车上 `bin/backup_20260707/`,`./restore_old.sh` 一键回滚并重启旧面板。

以后有「写好但未上车」的新版仍放本目录。
