#!/usr/bin/python3

from Centerpoint import *
random.seed(1)


if __name__ == '__main__':
    n = 100  # total number of points
    point_set = random_point_set(n, lower=-10, upper=10)

    cp = Centerpoint(point_set, plot=True)
    cp.find_L_boundary()
    cp.find_U_boundary()
    cp.find_D_boundary()
    cp.find_R_boundary()
    cp.find_intersections()


