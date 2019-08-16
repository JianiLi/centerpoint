#!/usr/bin/python3

from Centerpoint import *

random.seed(1)

if __name__ == '__main__':
    n = 1000  # total number of points
    plot = False

    point_set = random_point_set(n, lower=-10, upper=10)
    cp = Centerpoint(point_set, plot=plot)
    cp.reduce_then_get_centerpoint()
    # cp.brute_force_centerpoint()
