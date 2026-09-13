# -*- coding: utf-8 -*-
# vision_main.py —— 泰山派视觉检测(模块化版)
# 把原来全写在 main 里的代码拆成函数, main() 只做流程编排。
import cv2
import time
import numpy as np
from xbhdcc_tools import detect_cameras, WebStreamer

# ========================= 配置区 =========================
CAM_INDEX          = 9                       # 摄像头索引
CAP_V4L2           = cv2.CAP_V4L2
FRAME_W            = 1280
FRAME_H            = 720
FRAME_FPS          = 30
CROP               = (180, 540, 320, 940)    # (y1, y2, x1, x2) 裁剪
OPENMV_THRESHOLD   = [40, 89, -40, 106, -19, 23]   # (Lmin,Lmax,Amin,Amax,Bmin,Bmax) OpenMV格式
MIN_AREA           = 2000                    # 最小轮廓面积(像素)
APPROX_EPS         = 0.02                    # approxPolyDP 拟合精度系数
STREAM_PORT        = 8081


# ========================= 函数区 =========================
def convert_lab_thresholds(openmv_threshold):
    """OpenMV格式 [Lmin,Lmax,Amin,Amax,Bmin,Bmax] -> OpenCV (lower, upper)
    OpenMV 的 L 是 0~100, 这里乘 2.55 转成 0~255; A/B 是 -128~127, 加 128。"""
    l_min, l_max, a_min, a_max, b_min, b_max = openmv_threshold
    lower_bound = np.array([int(l_min * 2.55), int(a_min + 128), int(b_min + 128)])
    upper_bound = np.array([int(l_max * 2.55), int(a_max + 128), int(b_max + 128)])
    return lower_bound, upper_bound


def open_camera():
    """检测摄像头, 打开指定索引, 设置格式/分辨率/帧率"""
    detect_cameras()
    cap = cv2.VideoCapture(CAM_INDEX, CAP_V4L2)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, FRAME_FPS)
    return cap


def preprocess(frame, lower, upper):
    """BGR -> LAB -> 二值化 -> 形态学去噪 -> Canny 边缘"""
    lab   = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    mask  = cv2.inRange(lab, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask  = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return cv2.Canny(mask, 50, 150)


def extract_quads(edges):
    """从边缘图找所有"面积够大且是四边形"的轮廓, 返回角点列表"""
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    quads = []
    for cnt in contours:
        if cv2.contourArea(cnt) < MIN_AREA:
            continue
        approx = cv2.approxPolyDP(cnt, APPROX_EPS * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            quads.append(approx)
    return quads


def draw_quads(frame, quads, color=(0, 255, 0), thickness=3):
    """在 frame 上画出所有四边形边框(直接改原图, 返回 frame)"""
    for q in quads:
        p0, p1, p2, p3 = q[0][0], q[1][0], q[2][0], q[3][0]
        cv2.line(frame, p0, p1, color, thickness)
        cv2.line(frame, p1, p2, color, thickness)
        cv2.line(frame, p2, p3, color, thickness)
        cv2.line(frame, p3, p0, color, thickness)
    return frame


def quad_center(quad):
    """四边形四个角点的平均 -> 中心点 (x, y)"""
    xs = [p[0][0] for p in quad]
    ys = [p[0][1] for p in quad]
    return (int(sum(xs) / len(xs)), int(sum(ys) / len(ys)))


def draw_fps(frame, last_time, fps):
    """在 frame 上画FPS, 返回 (新last_time, 新fps)"""
    curr = time.time()
    fps = (1.0 / (curr - last_time)) * 0.3 + fps * 0.7
    cv2.putText(frame, "FPS:{:.2f}".format(fps), (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 2)
    return curr, fps


# ========================= 主流程 =========================
def main():
    cap = open_camera()
    lower, upper = convert_lab_thresholds(OPENMV_THRESHOLD)

    streamer = WebStreamer(port=STREAM_PORT)

    last_time = time.time()
    fps = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # 裁剪
        frame = frame[CROP[0]:CROP[1], CROP[2]:CROP[3]]

        # 画FPS
        last_time, fps = draw_fps(frame, last_time, fps)

        # 二值化 + 形态学 + 边缘
        edges = preprocess(frame, lower, upper)

        # 找四边形并画框
        quads = extract_quads(edges)
        frame = draw_quads(frame, quads)

        # 更新网页流: 0=原图(带框), 1=边缘图
        streamer.update_frame(0, frame)
        streamer.update_frame(1, edges)


if __name__ == "__main__":
    main()
