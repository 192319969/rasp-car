import cv2
from datetime import datetime

file = open('log.txt', 'a')
while True:
    file.write("now:", datetime.now().strftime("%H:%M:%S.%f")[:-3], "hello, i am here!\n")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

file.close()