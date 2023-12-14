import cv2
import numpy as np
import onnxruntime as ort
import car1 as car
import pid
import time
import math
import recognition as reco
import jixiebi
from simple_pid import PID
global bilv
def calculate_distance_from_box(box, img_width, img_height):
    box_width = box[2] - box[0]
    box_height = box[3] - box[1]
    box_area = box_width * box_height
    img_area = img_width * img_height
    area_ratio = box_area / img_area
    # 假设距离与面积成反比。这里的scale_factor取决于你的具体场景
    scale_factor = 1.0
    distance = scale_factor / area_ratio
    return distance

def mis_distance(bilv):
    k=80/(100-2.7)
    b=20-2.7*k
    return bilv*k+b
    

if __name__ == "__main__":

    model_pb_path = "best1.onnx"
    so = ort.SessionOptions()
    net = ort.InferenceSession(model_pb_path, so)

    dic_labels = {0: 'tennis',
                  1: 'badminton',
                  2: 'pingpong'}

    model_h = 320
    model_w = 320
    nl = 3
    na = 3
    stride = [8., 16., 32.]
    anchors = [[10, 13, 16, 30, 33, 23], [30, 61, 62, 45, 59, 119], [116, 90, 156, 198, 373, 326]]
    anchor_grid = np.asarray(anchors, dtype=np.float32).reshape(nl, -1, 2)
    car.initial()
    video = 0
    cap = cv2.VideoCapture(video)
    frame_count = 0  # 添加帧计数器
    skip_frames = 8  # 每8帧处理一次
    flag_det = False
    pid1 = PID(0.9, 0.01, 0.9, setpoint=0)

    while True:
        success, img0 = cap.read()
        height, width, _ = img0.shape  
        xMid = width / 2 * 1.0  
        yMid = height / 2 * 1.0
        if success:
            frame_count += 1
            if frame_count % skip_frames == 0: 
                if flag_det:
                    
                    t1 = time.time()
                    det_boxes, scores, ids = reco.infer_img(img0, net, model_h, model_w, nl, na, stride, anchor_grid,
                                                            thred_nms=0.4, thred_cond=0.5)
                      
                    t2 = time.time()
                    if len(det_boxes) == 0:
                        car.spun()
                    else:
                        for box, score, id in zip(det_boxes, scores, ids):
                            
                            label = '%s:%.2f' % (dic_labels[id], score)

                            reco.plot_one_box(box.astype(np.int16), img0, color=(255, 0, 0), label=label, line_thickness=None)

                            pid.get_ball_pos(box[0], box[2])
                            error = xMid - pid.ballX
                            steering_correction = pid1(error)
                            
                            bilv = calculate_distance_from_box(box, width,height)
                            #if abs(steering_correction) < 33:
                                #print(time.time(),":ball is at:", pid.ballX,"at the mid","srceen's mid is ",xMid)
                                #if bilv > 2.7:
                            if pid.ballX <=  xMid + mis_distance(bilv) and pid.ballX >=  xMid - mis_distance(bilv):
                                if bilv >2.7:
                                    print(bilv)
                                    car.move_forward(30)
                                else:
                                    car.stop()
                                    jixiebi.zhuaqu()
                                    
                            else:
                                #print(time.time(), ":ball is at:", pid.ballX, "not at the mid","srceen's mid is ",xMid)
                                if steering_correction < 0:
                                    if(steering_correction < -42):
                                        steering_correction = -42
                                    else:
                                        steering_correction = steering_correction/1.3
                                    car.turn_left(abs(steering_correction))
                                else:
                                
                                    if(steering_correction > 42):
                                        steering_correction = 42
                                    else:
                                        steering_correction = steering_correction/1.3
                                    car.turn_right(abs(steering_correction))

                            #car.drive_PID(pid.leftSpeed, pid.rightSpeed, pid.driveTime)

                    str_FPS = "FPS: %.2f" % (1. / (t2 - t1))

                    cv2.putText(img0, str_FPS, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
                frame_count = 0
            cv2.imshow("video", img0)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            jixiebi.stop_all_servos()
            break
        elif key & 0xFF == ord('s'):
            flag_det = not flag_det
            print(flag_det)
    cap.release()
    car.exit()


    

