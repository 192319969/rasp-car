import cv2
import numpy as np
import onnxruntime as ort
import car
import pid
import time
import recognition as reco

if __name__ == "__main__":

    # 模型加载
    model_pb_path = "best.onnx"
    so = ort.SessionOptions()
    net = ort.InferenceSession(model_pb_path, so)

    # 标签字典
    dic_labels = {0: 'tennis',
                  1: 'badminton',
                  2: 'pingpong'}

    # 模型参数
    model_h = 320
    model_w = 320
    nl = 3
    na = 3
    stride = [8., 16., 32.]
    anchors = [[10, 13, 16, 30, 33, 23], [30, 61, 62, 45, 59, 119], [116, 90, 156, 198, 373, 326]]
    anchor_grid = np.asarray(anchors, dtype=np.float32).reshape(nl, -1, 2)

    #小车电机初始化
    car.initial()

    video = 0
    cap = cv2.VideoCapture(video)
    flag_det = False
    while True:
        success, img0 = cap.read()

        # 获取图像的中心点坐标
        height, width, _ = img0.shape  # 获取图像的高度和宽度
        xMid = width / 2 * 1.0  # 计算图像中心点的 x 坐标
        yMid = height / 2 * 1.0  # 计算图像中心点的 y 坐标

        # 获取当前帧成功
        if success:

            # 开始识别物体
            if flag_det:
                t1 = time.time()
                det_boxes, scores, ids = reco.infer_img(img0, net, model_h, model_w, nl, na, stride, anchor_grid,
                                                        thred_nms=0.4, thred_cond=0.5)
                t2 = time.time()

                for box, score, id in zip(det_boxes, scores, ids):
                    label = '%s:%.2f' % (dic_labels[id], score)

                    reco.plot_one_box(box.astype(np.int16), img0, color=(255, 0, 0), label=label, line_thickness=None)

                    pid.get_ball_pos(box[0], box[2])

                    # xVariance = (pid.ballX - xMid) / xMid
                    # yVariance = (pid.ballY - yMid) / yMid
                    # pid.x_PID['error'] = xVariance / xMid
                    # pid.y_PID['error'] = yVariance / yMid
                    # pid.x_PID = pid.PID(pid.x_PID)
                    # pid.y_PID = pid.PID(pid.y_PID)
                    #
                    # pid.cal_speed()

                # write(log)
                if pid.is_at_center(xMid):
                    print(time.time(),":ball is at:", pid.ballX,"at the mid")
                    car.move_forward(15)
                else:
                    print(time.time(), ":ball is at:", pid.ballX, "not at the mid")
                    if pid.ballX < xMid:
                        car.turn_left()
                        time.sleep(0.01)
                    else:
                        car.turn_right()
                        time.sleep(0.01)

                    #car.drive_PID(pid.leftSpeed, pid.rightSpeed, pid.driveTime)

                str_FPS = "FPS: %.2f" % (1. / (t2 - t1))

                cv2.putText(img0, str_FPS, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)

            cv2.imshow("video", img0)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):

            break
        elif key & 0xFF == ord('s'):
            flag_det = not flag_det
            print(flag_det)

    cap.release()
    car.exit()