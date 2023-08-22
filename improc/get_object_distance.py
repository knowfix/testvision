import cv2
from math import sqrt, tan, atan, degrees, radians, cos

# rasio untuk perbandingan antara image dan aslinya dari percobaan
FOCAL_LENGTH = 613.9154104533

# fungsi kalkulasi jarak ground dari wahana ke drop zone dengan menghitung sudut objek
# karena kamera diberi sudut ke depan, perhitungan jarak X dan Y terpisah
def calc_ground_dist(img_obj_coordinate, center_of_frame, height, cam_angle):
    X_frame_dist = abs(img_obj_coordinate[0] - center_of_frame[0])
    X_angle = atan(X_frame_dist/FOCAL_LENGTH)
    
    X_ground_dist = height*tan(X_angle)

    Y_frame_dist = abs(img_obj_coordinate[1] - center_of_frame[1])
    Y_angle_from_center = atan(Y_frame_dist/FOCAL_LENGTH)
    
    # jika objek di depan titik fokus, angle kamera ditambah angle objek dr fokus
    if img_obj_coordinate[1] < center_of_frame[1]:
        Y_angle_from_cam = radians(cam_angle) + Y_angle_from_center
    
    # jika di belakang, angle kamera dikurang angle objek dari fokus
    else :
        Y_angle_from_cam = radians(cam_angle) - Y_angle_from_center
    
    Y_ground_dist = height*tan(Y_angle_from_cam)

    # jarak ground adalah gabungan jarak X dan Y
    ground_dist = sqrt(X_ground_dist**2+Y_ground_dist**2)

    return ground_dist

# fungsi kalkulasi jarak objek dari kamera dengan perbandingan lebar pixel
def calc_obj_distance_from_cam(real_width, px_width):
    return real_width * FOCAL_LENGTH / px_width

# fungsi kalkulasi jarak ground dengan perbandingan lebar
def calc_ground_dist_2(real_width, px_width, alt):
    distance_from_cam = calc_obj_distance_from_cam(real_width, px_width)
    # use pythagorean theorem to calculate horizontal distance
    if distance_from_cam > abs(alt):
        return sqrt(distance_from_cam**2 - alt**2)
    else:
        return distance_from_cam

# fungsi untuk menghitung range area beerdasarkan ketinggian
# min area dicari dengan jarak maksimal dengan sudut maks 45 derajat
def calc_obj_min_max_area(alt, obj_real_width):
    if alt <= 0:
        alt = 0.1
    min_area = 0.75*(FOCAL_LENGTH*obj_real_width/(alt/cos(radians(60))))**2
    max_area = 1.2*(FOCAL_LENGTH*obj_real_width/alt)**2

    return(min_area, max_area)

# calibration script
if __name__ == '__main__':
    import cv2

    # load the calibration image
    #image_ori = cv2.imread('/Data/coba2/1m 45.jpg')
    image_ori = cv2.imread('E:/FW/fiachra/src/improc/Data/coba2/1,5m 45.jpg')
    image = image_ori.copy()    # use copy so that it can be drawn on

    # define callback for mouse click
    def get_distance(event, x, y, flags, param):
        # grab references to the global variables
        global image
        # if the left mouse button was clicked,
        # record the starting (x, y) coordinates
        # also reset the previously drawn image
        if event == cv2.EVENT_LBUTTONDOWN:
            image = image_ori.copy()
            ref_pts = (x, y)

            dist = calc_ground_dist(ref_pts,[320,240],1.5,45)

            print('Distance =', dist)

            cv2.putText(image,
                        f'predicted distance: {dist} m',
                        (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2)

    # create a window that utilize the callback
    cv2.namedWindow("get_distance_check")
    cv2.setMouseCallback("get_distance_check", get_distance)

    # keep looping until the 'esc' key is pressed
    while True:
        # display the image and wait for a keypress
        cv2.imshow("get_distance_check", image)
        key = cv2.waitKey(1) & 0xFF

        # if the 'esc' key is pressed, break from the loop
        if key == 27:
            break