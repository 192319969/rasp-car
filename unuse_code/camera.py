import cv2
import numpy as np

# 初始化摄像头
camera = cv2.VideoCapture(0)

while True:
    # 读取摄像头帧
    ret, frame = camera.read()

    # 将帧转换为HSV色彩空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 定义网球的颜色范围（在HSV色彩空间中）
    lower_color = np.array([27, 104, 25])
    upper_color = np.array([77, 255, 255])

    # 根据颜色范围创建掩膜
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # 对掩膜进行形态学操作（可选，用于去除噪点）
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # 进行霍夫圆检测
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1, minDist=1000, param1=50, param2=15, minRadius=50,maxRadius=150)

    # 绘制检测到的圆
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)


    # 显示结果图像

    res = cv2.bitwise_and(frame,frame,mask=mask)

    cv2.imshow('mask1',mask)

    cv2.imshow('res',res)
    cv2.imshow("Frame", frame)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头资源并关闭窗口
camera.release()
cv2.destroyAllWindows()
