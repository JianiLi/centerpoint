#!/usr/bin/python3

from Centerpoint import *
import time

random.seed(1)

if __name__ == '__main__':
    n = 100  # total number of points
    plot = False
    start_time = time.time()
    point_set = random_point_set(n, lower=-100, upper=100)
    cp = Centerpoint(point_set, plot=plot)
    cp.reduce_then_get_centerpoint()
    #cp.brute_force_centerpoint()
    print("Total time used for %d points is: %.2f s" %(n, time.time()-start_time))