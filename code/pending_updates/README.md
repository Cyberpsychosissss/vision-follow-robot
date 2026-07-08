# pending_updates(已清空)

「写好但未上车」的新版放本目录;部署后提升进 `../control/` 或 `../orchestration/` 并清空。

## 部署历史

- **2026-07-07 上午**:跟随平滑化版(连续斜坡+EMA+slew)+ 一键脚本 + 速度面板热调 + Demo 录制 → 已部署,车上备份 `bin/backup_20260707/`。
- **2026-07-07 下午**:提频版 → 已部署,车上备份 `bin/backup_20260707_v2/`(部署前的上午版)。
  1. 控制循环 `LOOP_HZ` 20→**40Hz**(实测 ~40Hz;CAN 仍 50Hz=fr 协议周期);
  2. slew 限幅改物理单位(`ACCEL_UP=0.6 m/s²`、`ACCEL_DOWN=1.6`、`STEER_RATE=80°/s`),改频率手感不漂;
  3. EMA 只对 target.json 新样本滤一次,系数 0.4→0.6(等效手感);
  4. car_control `SPEED_SLEW` 0.6→1.2 m/s/s(旧值卡死减速);
  5. grabber `--write-fps` 9→**15**(相机实出 ~12.5fps,旧值白扔近 30% 帧;实测部署后 11.9fps)。

一键还原:车上 `./restore_old.sh bin/backup_20260707_v2`(回到上午版)或不带参数(自动用最新备份)。
