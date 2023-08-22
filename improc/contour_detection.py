import cv2
import numpy as np
from math import sqrt

font = cv2.FONT_HERSHEY_COMPLEX

# rumus euclidian distance
def calc_distance(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# fungsi mencari kontur
# input frame, upper dan lower hsv, jumlah vertices dan batas luas area kontur
def get_contour(frame, lower_h, upper_h, lower_s, upper_s, lower_v, upper_v, vertices, min_area, max_area):
    cx = -1
    cy = -1
    approx_arr = []

    #contour_corner = []
    #sides_lenght_arr = []

    #contour_width = -1

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # shift nilai hue untuk agar thresholding bisa menjangkau semua spektrum merah
    h, s, v = cv2.split(hsv)
    shift_h = h
    shift_hsv = cv2.merge([shift_h, s, v])

    # array batas atas dan batas bawah hsv
    lower_hsv = np.array([lower_h, lower_s, lower_v])
    upper_hsv = np.array([upper_h, upper_s, upper_v])

    # threshold dengan range hsv kemudian opening
    mask = cv2.inRange(shift_hsv, lower_hsv, upper_hsv)
    kernel = np.ones((3, 3), np.uint8)
    erosion = cv2.erode(mask, kernel, iterations = 2)
    dilation = cv2.dilate(erosion, kernel, iterations = 3)

    # Contours detection
    contours,_ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours :
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            epsilon = 0.1011*cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # filter kontur yang diinginkan dengan luas area dan jumlah bidang
            if (min_area < area < max_area) :
                if len(approx) == vertices:

                    approx_arr = [approx]

                    # menggunakan fungsi moments untuk mendapat centroid
                    M = cv2.moments(cnt)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    # membuat array 2D untuk sudut2 kontur
                    #for i in range(vertices):
                    #    contour_corner.append([approx.ravel()[2*i], approx.ravel()[2*i+1]])
                    
                    # menghitung panjang setiap sisi
                    #for j in range(vertices):
                    #    sides_lenght_arr.append(calc_distance(contour_corner[j], contour_corner[(j+1)%vertices]))

                    # mencari panjang sisi maksimum sebagai lebar objek
                    #contour_width = max(sides_lenght_arr)

    return (cx, cy, approx_arr,dilation)
    # fungsi get_contour memberi return titik tengah objek, array titik2 kontur, dan lebarnya