import math

from shapely.geometry import Point
from shapely.geometry import MultiPoint

import random

import numpy as np
from PlotUtils import *


class Line:
    def __init__(self, m, b):
        self.m = m
        self.b = b


class LineSegment:
    def __init__(self, point1, point2):
        self.p1 = point1
        self.p2 = point2

    def __str__(self):
        return 'LineSegment ({}, {})'.format(self.p1, self.p2)


class Interval:
    def __init__(self, l, r):
        self.l = l
        self.r = r
        assert l <= r

    def __str__(self):
        return 'Interval from {} to {}'.format(self.l, self.r)

    def __len__(self):
        return int(self.r - self.l)


class Intersection:
    def __init__(self, line1, line2):
        if line1.m == line2.m:
            self.x = np.inf
            self.y = np.inf
        else:
            self.x = (line2.b - line1.b) / (line1.m - line2.m)
            self.y = line1.m * self.x + line1.b


def get_intersections(lines, interval):
    intersections = []
    dual_num = len(lines)
    for i in range(dual_num):
        for j in range(i):
            d1 = lines[i]
            d2 = lines[j]
            new_inter = Intersection(d1, d2)
            if new_inter.x == np.inf:
                pass
            elif interval.l < new_inter.x and interval.r > new_inter.x:
                intersections.append(new_inter)
            else:
                pass
    intersections.sort(key=lambda I: I.x)
    return intersections


def line_over_two_points(p1, p2):
    if p1.x == p2.x:
        return Line(np.inf, 0)
    else:
        k = (p2.y - p1.y) / (p2.x - p1.x)
        b = -p1.x * ((p2.y - p1.y) / (p2.x - p1.x)) + p1.y
    return Line(k, b)


def find_x_bounds(point_set):
    min_x = min(point_set, key=lambda P: P.x).x
    max_x = max(point_set, key=lambda P: P.x).x
    return min_x, max_x


def find_y_bounds(point_set):
    min_y = min(point_set, key=lambda P: P.y).y
    max_y = max(point_set, key=lambda P: P.y).y
    return min_y, max_y


def compute_dual_line(P):
    """Compute dual of a point

    Arguments:
        P {shapely.geometry.Point}
    """
    return Line(P.x, -P.y)


def random_point_set(n, lower=-10, upper=10):
    points = []
    assert lower <= upper
    for i in range(n):
        x = random.uniform(lower, upper)
        y = random.uniform(lower, upper)
        points.append(Point(x, y))
    return points


def get_Radon_point(p1, p2, p3, p4):
    convex_hull = MultiPoint([p1,p2,p3,p4]).convex_hull
    line_13 = line_over_two_points(p1, p3)
    line_24 = line_over_two_points(p2, p4)
    Radon_point = Point(Intersection(line_13, line_24).x,Intersection(line_13, line_24).y)
    if Radon_point.within(convex_hull):
        return Radon_point
    else:
        line_12 = line_over_two_points(p1, p2)
        line_34 = line_over_two_points(p3, p4)
        Radon_point = Point(Intersection(line_12, line_34).x, Intersection(line_12, line_34).y)
        if Radon_point.within(convex_hull):
            return Radon_point
        else:
            line_14 = line_over_two_points(p1, p4)
            line_23 = line_over_two_points(p2, p3)
            Radon_point = Point(Intersection(line_14, line_23).x, Intersection(line_14, line_23).y)
            return Radon_point



def point_transfer(p, x0, y0, line):
    x = p.x
    y = p.y
    k = line.m
    b = line.b
    sin_angle = -1 / (math.sqrt(k ** 2 + 1)) if k > 0 else 1 / (math.sqrt(k ** 2 + 1))
    cos_angle = k / (math.sqrt(k ** 2 + 1)) if k > 0 else -k / (math.sqrt(k ** 2 + 1))
    x_prime = (x - x0) * cos_angle + (y - y0) * sin_angle
    y_prime = (y - y0) * cos_angle - (x - x0) * sin_angle
    return Point(x_prime, y_prime)


def point_transfer_back(p_prime, x0, y0, line):
    x_prime = p_prime.x
    y_prime = p_prime.y
    k = line.m
    b = line.b
    sin_angle = -1 / (math.sqrt(k ** 2 + 1)) if k > 0 else 1 / (math.sqrt(k ** 2 + 1))
    cos_angle = k / (math.sqrt(k ** 2 + 1)) if k > 0 else -k / (math.sqrt(k ** 2 + 1))
    x = x_prime * cos_angle - y_prime * sin_angle + x0
    y = y_prime * cos_angle + x_prime * sin_angle + y0
    return Point(x, y)


def test_point_transfer():
    '''
    if point transfer and transfer back functions are correct,
    the black and red points should overlap.
    '''
    k = -2
    b = -2
    l = Line(k, b)
    x0 = 2
    y0 = k * x0 + b
    coord = Point(x0, y0)
    l_pend = Line(-1 / k, 1 / k * coord.x + coord.y)

    sin_angle = -1 / (math.sqrt(k ** 2 + 1)) if k > 0 else 1 / (math.sqrt(k ** 2 + 1))
    cos_angle = k / (math.sqrt(k ** 2 + 1)) if k > 0 else -k / (math.sqrt(k ** 2 + 1))

    plt.ion()
    plt.show()
    prepare_axis()
    plot_line(l)
    plot_line(l_pend)

    point_set = random_point_set(10, lower=-10, upper=10)
    plot_point_set(point_set, color='k')
    P = []
    P_trans = []

    for p in point_set:
        p_trans = point_transfer(p, x0, y0, l)
        P_trans.append(p_trans)
        p_trans_back = point_transfer_back(p_trans, x0, y0, l)
        P.append(p_trans_back)

    # plot_point_set(P_trans, color='b')
    plot_point_set(P, color='r')

    plt.pause(1)
    end = input('Press enter to end the next step')


if __name__ == '__main__':
    test_point_transfer()
