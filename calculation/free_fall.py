from math import sqrt

G = 9.807     # konstanta G dalam m/s^2

# menghitung waktu gerak jatuh bebas
# return waktu dalam detik
def get_falling_time(height):
    time = sqrt(2*height/G)
    return time

# menghitung jarak horizontal dlm meter
# dengan input kecepatan tertentu pada ketinggian tertentu
def calc_horizontal_travel_dist(velocity, height):
    t = get_falling_time(height)
    horizontal_dist = velocity*t
    return horizontal_dist

if __name__=='__main__':
    time = get_falling_time(70)
    print('Falling time : ', time)

    print('Horizontal dist : ', calc_horizontal_travel_dist(15,70))