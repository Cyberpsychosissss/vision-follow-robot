#!/usr/bin/env python3
"""渲染 demo 视频侧栏用的中文标签 PNG(透明底, 深色主题浅色字)。车上 cv2.putText 不支持中文, 预渲染位图 blit。"""
import os
from PIL import Image, ImageDraw, ImageFont

FONT = '/System/Library/Fonts/Hiragino Sans GB.ttc'
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'demo_assets')
os.makedirs(OUT, exist_ok=True)

# 深色工程风配色(RGB)
LIGHT = (230, 237, 243)   # 主文字 #e6edf3
SUB   = (139, 152, 165)   # 次文字 #8b98a5
CAPT  = (203, 213, 225)   # 视图标题 #cbd5e1
WHITE = (255, 255, 255)
RED   = (248, 113, 113)   # #f87171
AMBER = (251, 191, 36)    # #fbbf24
BLUE  = (96, 165, 250)    # #60a5fa
GRAYM = (156, 163, 175)   # #9ca3af

ITEMS = [
    ('title',      '视觉跟随机器人', 26, LIGHT, True),
    # 状态 chips(白字, 底色由 cv2 画)
    ('st_follow',    '跟随中',   26, WHITE, True),
    ('st_hold',      '保持距离', 26, WHITE, True),
    ('st_stopnear',  '太近·停止', 26, WHITE, True),
    ('st_search',    '搜索目标', 26, WHITE, True),
    ('st_coast',     '滑行',     26, WHITE, True),
    ('st_steeronly', '仅转向',   26, WHITE, True),
    ('st_off',       '已停止',   26, WHITE, True),
    # 模式(彩色字, 深色底由 cv2 画)
    ('md_armed', '真实控车',        17, RED, True),
    ('md_steer', '仅转向·不前进',   17, AMBER, True),
    ('md_dry',   '模拟运行·不发帧', 17, BLUE, True),
    ('md_off',   '控制器未运行',    17, GRAYM, False),
    # 数据标签(次文字色)
    ('l_dist',  '目标距离 m',    17, SUB, False),
    ('l_lat',   '横向偏移 m',    17, SUB, False),
    ('l_speed', '下发速度 m/s',  17, SUB, False),
    ('l_steer', '下发转向 °',    17, SUB, False),
    ('l_keep',  '保持距离 m',    15, SUB, False),
    ('l_vmax',  '速度上限 m/s',  15, SUB, False),
    ('l_cam',   '相机帧率 fps',  15, SUB, False),
    ('l_conf',  '置信度 %',      15, SUB, False),
    ('l_lux',   '光照 0~255',    15, SUB, False),
    ('l_batt',  '电池电量',      17, SUB, False),
    ('l_volt',  '电压 V',        15, SUB, False),
    ('l_amp',   '电流 A',        15, SUB, False),
    ('l_ah',    '剩余 Ah',       15, SUB, False),
    ('l_left',  '左',            14, SUB, False),
    ('l_right', '右',            14, SUB, False),
    # 小 chips(白字)
    ('chip_chg', '充电中', 15, WHITE, True),
    ('chip_dis', '放电',   15, WHITE, True),
    # 相机画面上的目标标签(白字)
    ('tag_person', '目标', 18, WHITE, True),
    # 右侧视图标题
    ('v_cam',  '相机画面 · YOLO', 15, CAPT, True),
    ('v_disp', '视差图 · 伪彩',   15, CAPT, True),
]

for name, text, size, color, bold in ITEMS:
    font = ImageFont.truetype(FONT, size, index=1 if bold else 0)
    tmp = Image.new('RGBA', (10, 10))
    d = ImageDraw.Draw(tmp)
    box = d.textbbox((0, 0), text, font=font)
    w, h = box[2] - box[0], box[3] - box[1]
    img = Image.new('RGBA', (w + 4, h + 4), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.text((2 - box[0], 2 - box[1]), text, font=font, fill=color + (255,))
    img.save(os.path.join(OUT, name + '.png'))
print('OK %d labels ->' % len(ITEMS), OUT)
