import cv2
a=cv2.imread("/home/pi/2020-06-15-162551_1920x1080_scrot.png")
cv2.imshow("test",a)
cv2.waitKey()
cv2.destroyAllWindows()