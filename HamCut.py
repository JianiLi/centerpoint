from shapely.geometry import LineString

from GeoUtils import *


class HamCut:
    def __init__(self, min_interval=1):
        self.min_interval = min_interval
        self.ham_instance = None
        self.ham_cuts = None
        self.num_need_red = None
        self.num_need_blue = None
        self.above = True

    def reduce_then_cut(self, ham_instance, num_need_red, num_need_blue, above=True):
        self.above = above
        self.ham_instance = ham_instance
        self.num_need_red = num_need_red
        self.num_need_blue = num_need_blue

        x_min, x_max = find_x_bounds(self.ham_instance.all_points)
        self.interval = Interval(x_min - 30, x_max + 30)

        while len(self.interval) > self.min_interval:
            l_interval, r_interval = self._split_interval()
            # If we have odd intersection in left, then select left half interval. Why not even number? because you cannot tell if it is even or 0.
            if self._odd_intersection(l_interval, above=above):
                self.interval = l_interval
            else:
                self.interval = r_interval
        # When you have a small interval, use a brute-force method to get the median intersection
        self.ham_cuts = self.intersection(above=above)
        return self.ham_cuts

    def _odd_intersection(self, interval, above=True):

        left_red = self._find_part_boundary_level(interval.l, self.ham_instance.red_duals, self.num_need_red,
                                                  above=above)
        left_blue = self._find_part_boundary_level(interval.l, self.ham_instance.blue_duals, self.num_need_blue,
                                                   above=above)

        right_red = self._find_part_boundary_level(interval.r, self.ham_instance.red_duals, self.num_need_red,
                                                   above=above)
        right_blue = self._find_part_boundary_level(interval.r, self.ham_instance.blue_duals, self.num_need_blue,
                                                    above=above)

        # if odd intersections, product should be negative, else positive
        return (left_red - left_blue) * (right_red - right_blue) < 0

    def _find_part_boundary_level(self, x, lines, num, above=True):
        '''
        If not reverse, we find point that has [num] lines below it in the dual plane, and in the primal plane,
        we will have [num] points above above the line (dual of the point).
        If reverse, we will have [num] points below the line in the primal plane.
        '''
        y_vals = [line.b + (x * line.m) for line in lines]
        y_vals.sort(reverse=not above)
        return y_vals[num - 1]

    def intersection(self, above=True):
        red_intersections = self._get_intersections(self.ham_instance.red_duals)
        blue_intersections = self._get_intersections(self.ham_instance.blue_duals)

        red_boundary_linestring = self._get_part_linestring(self.ham_instance.red_duals, red_intersections,
                                                            self.num_need_red, above=above)
        blue_boundary_linestring = self._get_part_linestring(self.ham_instance.blue_duals, blue_intersections,
                                                             self.num_need_blue, above=above)

        ham_points = red_boundary_linestring.intersection(blue_boundary_linestring)
        if isinstance(ham_points, Point):
            ham_points = [ham_points]
        ham_cuts = [compute_dual_line(hp) for hp in ham_points]

        return ham_cuts

    def _get_intersections(self, duals):
        intersections = []
        dual_num = len(duals)
        for i in range(dual_num):
            for j in range(i):
                d1 = duals[i]
                d2 = duals[j]
                new_inter = Intersection(d1, d2)
                if new_inter.x == np.inf:
                    pass
                elif self.interval.l < new_inter.x and self.interval.r > new_inter.x:
                    intersections.append(new_inter)
                else:
                    pass
        intersections.sort(key=lambda I: I.x)
        return intersections

    def _get_part_linestring(self, duals, intersections, part, above=True):
        part_boundary_levels = [
            Point(self.interval.l, self._find_part_boundary_level(self.interval.l, duals, part, above=above))]
        part_boundary_levels.extend(
            [Point(i.x, self._find_part_boundary_level(i.x, duals, part, above=above)) for i in intersections])
        part_boundary_levels.extend(
            [Point(self.interval.r, self._find_part_boundary_level(self.interval.r, duals, part, above=above))])

        return LineString(part_boundary_levels)

    def _split_interval(self):
        mid = float((self.interval.l + self.interval.r) / 2.0)
        return Interval(self.interval.l, mid), Interval(mid, self.interval.r)
