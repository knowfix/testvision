import cv2
import time

from contour_detection import get_contour
from get_object_distance import calc_ground_dist, calc_ground_dist_2


VID_INPUT = 'E:\\FW\\vid raspi\\Terbang\\hasil trim.mp4'

DROP_ZONE_REAL_WIDTH = 4.7

# Upper & Lower HSV hasil percobaan
LOWER_H = 5
LOWER_S = 10
LOWER_V = 190

UPPER_H = 65
UPPER_S = 135
UPPER_V = 255

# Jumlah vertices dari contour yg ingin dideteksi
VERTICES = 4

# Luas area (dlm pixel) dari kontur
MIN_AREA = 450
MAX_AREA = 2000

# cap = cv2.VideoCapture(VID_INPUT)
cap = cv2.VideoCapture(0)

CAM_ANGLE = 24

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

fourcc = cv2.VideoWriter_fourcc(*'PIM1')
out = cv2.VideoWriter('E:\hasil1.avi', fourcc, 20, (frame_width, frame_height))


while True:
    _, frame = cap.read()

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    center_of_frame = [frame_width/2, frame_height/2]

    target = get_contour(frame, LOWER_H, UPPER_H, LOWER_S, UPPER_S, LOWER_V, UPPER_V, VERTICES, MIN_AREA, MAX_AREA)

    if target[0] >= 0:
        cx = target[0]
        cy = target[1]

        center_of_target = [cx,cy]
        target_dist = calc_ground_dist(center_of_target, center_of_frame, 75, CAM_ANGLE)

        #target_dist_2 = calc_ground_dist_2(DROP_ZONE_REAL_WIDTH, target[3], 75)

        approx = target[2]
        
        cv2.drawContours(frame, approx, 0, (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 3, (255, 255, 255), -1)
        cv2.putText(frame,
                'Dist : %.2f' %target_dist + ' m',
                (cx+10, cy+5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1)

        '''cv2.putText(frame,
                'Width: %.2f' % target[3],
                (10, 440),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)
        
        cv2.putText(frame,
                'Dist 2: %.2f' % target_dist_2,
                (10, 460),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)'''

    cv2.imshow("Frame", frame)
    cv2.imshow("contour", target[3])

    out.write(frame)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()