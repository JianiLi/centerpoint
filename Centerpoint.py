import copy

from HamCut.HamCut import *
from utils.GeoUtils import *
from utils.utils import *


class Centerpoint:
    def __init__(self, point_set, plot=False):
        self.point_set = point_set
        self.n = len(point_set)
        self.np = math.ceil(self.n / 3)
        self.mp = math.ceil(self.n / 3) - math.ceil(self.n / 4)

        self.x_min, self.x_max = find_x_bounds(self.point_set)
        self.y_min, self.y_max = find_y_bounds(self.point_set)

        self.points_in_L = None
        self.points_not_in_L = None
        self.points_in_L_boundary = None

        self.points_in_U = None
        self.points_not_in_U = None
        self.points_in_D_boundary = None

        self.points_in_D = None
        self.points_not_in_D = None

        self.L_boundary_line = None
        self.U_boundary_line = None
        self.D_boundary_line = None
        self.R_boundary_line = None

        self.points_LU = None
        self.points_LD = None
        self.points_RU = None
        self.points_RD = None

        self.plot = plot

    def reduce_then_get_centerpoint(self):
        last_point_num = len(self.point_set)
        cur_point_num = 0
        print("point number: %d" % last_point_num)
        while cur_point_num < last_point_num:
            last_point_num = len(self.point_set)
            self.__init__(self.point_set, plot=self.plot)
            try:
                self.find_L_boundary()
                self.find_U_boundary()
                self.find_D_boundary()
                self.find_R_boundary()
                self.find_intersections()
                self.replace_points()
                self.point_set = remove_repeat_points(self.point_set)
                cur_point_num = len(self.point_set)
                print("point number: %d" % cur_point_num)
            except:
                pass
        self.brute_force_centerpoint()

    def brute_force_centerpoint(self):
        centerpoints = []
        remaining_points = copy.deepcopy(self.point_set)

        while len(self.point_set) >= 4:
            p_corner = find_corner_points(self.point_set)[0:4]
            for p in p_corner:
                self.point_set.remove(p)
            if len(p_corner) == 4:
                Radon_point = get_Radon_point(p_corner[0], p_corner[1], p_corner[2], p_corner[3])
                self.point_set.append(Radon_point)

        # finally only three cases arise: 1/2/3 points remaining
        if len(self.point_set) == 1:
            centerpoints.append(self.point_set[0])
        elif len(self.point_set) == 2:
            centerpoints.append(LineString([self.point_set[0], self.point_set[1]]).centroid)
        else:
            centerpoints.append(LineString([self.point_set[0], self.point_set[1], self.point_set[2]]).centroid)

        for cp in centerpoints:
            print("Centerpoints: %.2f, %.2f" % (cp.x, cp.y))
        if self.plot:
            plt.title('Centerpoint: %.2f, %.2f' % (centerpoints[0].x, centerpoints[0].y))
            plot_point_set(remaining_points, color='b')
            plot_point(centerpoints[0], color='r')
            plt.pause(1)
            end = input('Press enter to the next step')
        return centerpoints[0]

    def find_L_boundary(self):
        leftmost_x = min(self.point_set, key=lambda P: P.x).x
        leftmost_y = min(self.point_set, key=lambda P: P.x).y
        leftmost = Point(leftmost_x, leftmost_y)

        slope = [(p.y - leftmost.y) / (p.x - leftmost.x) for p in self.point_set if
                 not p == leftmost and not p.x == leftmost.x]

        equal_x_num = len([p for p in self.point_set if p.x == leftmost.x and p.y > leftmost.y])

        slope_np = findKthLargest(slope, self.np - 1 - equal_x_num)
        slope_np_index = slope.index(slope_np)

        index_before_num = len([p for p in self.point_set if
                                p.x == leftmost.x and p.y >= leftmost.y and self.point_set.index(p) <= slope_np_index])
        L_boundary_point = self.point_set[slope_np_index + index_before_num]

        self.L_boundary_line = Line((L_boundary_point.y - leftmost.y) / (L_boundary_point.x - leftmost.x),
                                    (leftmost.y * L_boundary_point.x - leftmost.x * L_boundary_point.y) / (
                                            L_boundary_point.x - leftmost.x))
        self.points_in_L = [p for p in self.point_set if
                            p.y > p.x * self.L_boundary_line.m + self.L_boundary_line.b + 1e-5]
        self.points_not_in_L = [p for p in self.point_set if
                                p.y < p.x * self.L_boundary_line.m + self.L_boundary_line.b - 1e-5]
        self.points_in_L_boundary = [p for p in self.point_set if
                                     p not in self.points_in_L and p not in self.points_not_in_L]
        if self.plot:
            plt.clf()
            prepare_plot(self.point_set)
            plt.title('Find L boundary')
            plot_point(leftmost, color='r')
            plot_line(self.L_boundary_line, color='g')
            plot_point_set(self.points_in_L, color='r')
            plot_point_set(self.points_not_in_L, color='b')
            plot_point_set(self.points_in_L_boundary, color='g')
            plt.pause(1)
            # end = input('Press enter to the next step')

    def find_U_boundary(self):
        x0, y0 = self.points_in_L_boundary[0].x, self.points_in_L_boundary[0].y
        points_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_in_L]
        points_not_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_not_in_L]
        if not points_in_L_trans == [] and not points_not_in_L_trans == []:
            above = True if self.L_boundary_line.m >= 0 else False
            (red_points, blue_points) = (points_in_L_trans, points_not_in_L_trans) if points_in_L_trans[0].x < points_not_in_L_trans[0].x \
                else (points_not_in_L_trans, points_in_L_trans)
            (red_num_need, blue_num_need) = (self.mp - 1, self.np - self.mp - 1) if red_points == points_in_L_trans else (
                                            self.np - self.mp - 1, self.mp - 1)

            hamcut = HamCut()
            ham_cut_trans = hamcut.reduce_then_cut(red_points, blue_points, red_num_need, blue_num_need, above=above)
            ham_points_trans = [Point(self.x_min - 5, (self.x_min - 5) * ham_cut_trans.m + ham_cut_trans.b),
                                Point(self.x_max + 5, (self.x_max + 5) * ham_cut_trans.m + ham_cut_trans.b)]

            ham_points = [point_transfer_back(p, x0, y0, self.L_boundary_line) for p in ham_points_trans]

            self.U_boundary_line = line_over_two_points(ham_points[0], ham_points[1])

            self.points_in_U = [p for p in self.point_set if
                                p.y > p.x * self.U_boundary_line.m + self.U_boundary_line.b + 1e-5]
            self.points_not_in_U = [p for p in self.point_set if
                                    p.y < p.x * self.U_boundary_line.m + self.U_boundary_line.b - 1e-5]
            if len(self.points_in_U) > len(self.points_not_in_U):
                self.points_in_U, self.points_not_in_U = self.points_not_in_U, self.points_in_U

            self.points_in_U_boundary = [p for p in self.point_set if
                                         p not in self.points_in_U and p not in self.points_not_in_U]
            if self.plot:
                plt.title('Find U boundary')
                plot_line(self.U_boundary_line, color='k')
                plot_point_set(self.points_in_U, color='r')
                plot_point_set(self.points_not_in_U, color='b')
                plot_point_set(self.points_in_U_boundary, color='g')
                plt.pause(1)
                # end = input('Press enter to the next step')

    def find_D_boundary(self):
        x0, y0 = self.points_in_L_boundary[0].x, self.points_in_L_boundary[0].y
        points_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_in_L]
        points_not_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_not_in_L]
        if not points_in_L_trans == [] and not points_not_in_L_trans == []:
            above = False if self.L_boundary_line.m >= 0 else True
            (red_points, blue_points) = (points_in_L_trans, points_not_in_L_trans) if points_in_L_trans[0].x < \
                                                                                      points_not_in_L_trans[0].x \
                else (points_not_in_L_trans, points_in_L_trans)
            (red_num_need, blue_num_need) = (self.mp - 1, self.np - self.mp - 1) if red_points == points_in_L_trans else (
                self.np - self.mp - 1, self.mp - 1)

            hamcut = HamCut()
            ham_cut_trans = hamcut.reduce_then_cut(red_points, blue_points, red_num_need, blue_num_need, above=above)

            ham_points_trans = [Point(self.x_min - 5, (self.x_min - 5) * ham_cut_trans.m + ham_cut_trans.b),
                                Point(self.x_max + 5, (self.x_max + 5) * ham_cut_trans.m + ham_cut_trans.b)]

            ham_points = [point_transfer_back(p, x0, y0, self.L_boundary_line) for p in ham_points_trans]

            self.D_boundary_line = line_over_two_points(ham_points[0], ham_points[1])

            # slope_reverse = 1 if y0 > x0 * self.D_boundary_line.m + self.D_boundary_line.b + 1e-5 else -1
            self.points_in_D = [p for p in self.point_set if
                                p.y > p.x * self.D_boundary_line.m + self.D_boundary_line.b + 1e-5]
            self.points_not_in_D = [p for p in self.point_set if
                                    p.y < p.x * self.D_boundary_line.m + self.D_boundary_line.b - 1e-5]
            if len(self.points_in_D) > len(self.points_not_in_D):
                self.points_in_D, self.points_not_in_D = self.points_not_in_D, self.points_in_D
            points_in_D_boundary = [p for p in self.point_set if
                                    p not in self.points_in_D and p not in self.points_not_in_D]
            if self.plot:
                plt.title('Find D boundary')
                plot_line(self.U_boundary_line, color='g')
                plot_line(self.D_boundary_line, color='k')
                plot_point_set(self.points_in_D, color='r')
                plot_point_set(self.points_not_in_D, color='b')
                plot_point_set(points_in_D_boundary, color='g')
                plt.pause(1)
                # end = input('Press enter to the next step')

    def find_R_boundary(self):
        x0, y0 = self.points_in_U_boundary[0].x, self.points_in_U_boundary[0].y
        points_in_U_trans = [point_transfer(p, x0, y0, self.U_boundary_line) for p in self.points_in_U]
        points_not_in_U_trans = [point_transfer(p, x0, y0, self.U_boundary_line) for p in self.points_not_in_U]

        #above = True if self.U_boundary_line.m < 0 else False
        if not points_in_U_trans==[] and not points_not_in_U_trans==[]:
            (red_points, blue_points) = (points_in_U_trans, points_not_in_U_trans) if points_in_U_trans[0].x < points_not_in_U_trans[0].x \
                else (points_not_in_U_trans, points_in_U_trans)
            (red_num_need, blue_num_need) = (self.mp - 1, self.np - self.mp - 1) if red_points == points_in_U_trans else (
                                            self.np - self.mp - 1, self.mp - 1)

            hamcut = HamCut()
            ham_cut_trans = hamcut.reduce_then_cut(red_points, blue_points, red_num_need, blue_num_need, above=False)

            ham_points_trans = [Point(self.x_min - 5, (self.x_min - 5) * ham_cut_trans.m + ham_cut_trans.b),
                                Point(self.x_max + 5, (self.x_max + 5) * ham_cut_trans.m + ham_cut_trans.b)]

            ham_points = [point_transfer_back(p, x0, y0, self.U_boundary_line) for p in ham_points_trans]

            self.R_boundary_line = line_over_two_points(ham_points[0], ham_points[1])
            self.points_in_R = [p for p in self.point_set if
                                p.y > p.x * self.R_boundary_line.m + self.R_boundary_line.b + 1e-5]
            self.points_not_in_R = [p for p in self.point_set if
                                    p.y < p.x * self.R_boundary_line.m + self.R_boundary_line.b - 1e-5]
            if len(self.points_in_R) > len(self.points_not_in_R):
                self.points_in_R, self.points_not_in_R = self.points_not_in_R, self.points_in_R

            points_in_R_boundary = [p for p in self.point_set if
                                    p not in self.points_in_R and p not in self.points_not_in_R]
            if self.plot:
                plt.title('Find R boundary')
                plot_line(self.U_boundary_line, color='g')
                plot_line(self.D_boundary_line, color='g')
                plot_line(self.R_boundary_line, color='k')
                plot_point_set(self.points_in_R, color='r')
                plot_point_set(self.points_not_in_R, color='b')
                plot_point_set(points_in_R_boundary, color='g')
                plt.pause(1)
                # end = input('Press enter to the next step')

    def find_intersections(self):
        self.points_LU = [p for p in self.points_in_L if p in self.points_in_U]
        self.points_LD = [p for p in self.points_in_L if p in self.points_in_D]
        self.points_RU = [p for p in self.points_in_R if p in self.points_in_U]
        self.points_RD = [p for p in self.points_in_R if p in self.points_in_D]

        if self.plot:
            plt.title('Find P_LU, P_LD, P_RU, P_RD')
            plot_point_set(self.point_set, color='b')
            plot_point_set(self.points_LU, color='r')
            plot_point_set(self.points_LD, color='g')
            plot_point_set(self.points_RU, color='m')
            plot_point_set(self.points_RD, color='k')
            plt.pause(1)
            end = input('Press enter to the next step')

    def replace_points(self):
        Radon_point_set = []
        while self.points_LU != [] and self.points_LD != [] and self.points_RU != [] and self.points_RD != []:
            p_LU, p_LD, p_RU, p_RD = self.points_LU.pop(), self.points_LD.pop(), self.points_RU.pop(), self.points_RD.pop()
            Radon_point = get_Radon_point(p_LU, p_RU, p_RD, p_LD)
            Radon_point_set.append(Radon_point)
            for p in [p_LU, p_LD, p_RU, p_RD]:
                try:
                    self.point_set.remove(p)
                except:
                    pass
            if not math.isnan(Radon_point.x) and not math.isnan(Radon_point.y):
                self.point_set.append(Radon_point)
        if self.plot:
            plt.clf()
            prepare_plot(self.point_set)
            plt.title('replace Q by their Radon point')
            plot_line(self.L_boundary_line, color='g')
            plot_line(self.U_boundary_line, color='g')
            plot_line(self.D_boundary_line, color='g')
            plot_line(self.R_boundary_line, color='g')
            plot_point_set(Radon_point_set, color='r')
            plt.pause(1)
            end = input('Press enter to the next step')
