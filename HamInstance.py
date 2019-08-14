from GeoUtils import *
from HamCut import *

class HamInstance:
    def __init__(self, point_set1, point_set2):
        self.red_points = point_set1
        self.blue_points = point_set2
        self.red_points_num = len(point_set1)
        self.blue_points_num = len(point_set2)

        # all points
        self.all_points = self.blue_points + self.red_points

        # duals
        self.red_duals = [compute_dual_line(p) for p in self.red_points]
        self.blue_duals = [compute_dual_line(p) for p in self.blue_points]

