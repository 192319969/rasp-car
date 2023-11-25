import cv2
import numpy as np

cap = cv2.VideoCapture(0)
while(1):
 # get a frame
 ret, frame = cap.read()
 # show a frame
 cv2.imshow("capture", frame)

 if cv2.waitKey(1) & 0xFF == ord('q'):
 #退出并拍照
  cv2.imwrite("tennis3.jpg", frame)
  print("take Photo Ok")
  break
cap.release()
cv2.destroyAllWindows()
