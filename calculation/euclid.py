from math import sqrt
# calculate euclidean distance between 2 points
# use pythagoras
# point format: (x, y)

def calc_distance(p1, p2):
  return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)