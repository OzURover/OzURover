import numpy as np
import cv2

cap = cv2.VideoCapture("rtsp://192.168.0.10:554/user=admin_password=tlJwpbo6_channel=1_stream=0.sdp")

while(True):
    # Capture frame-by-frame
    for i in range(4):
        cap.grab()
    ret, frame = cap.read()

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
