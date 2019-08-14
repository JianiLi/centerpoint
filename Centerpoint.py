from HamInstance import *
from PlotUtils import *


class Centerpoint:
    def __init__(self, point_set, plot=False):
        self.point_set = point_set
        self.n = len(point_set)
        self.np = math.ceil(self.n / 3)
        self.mp = math.ceil(self.n / 4)

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
        if self.plot:
            self.prepare_plot()

    def prepare_plot(self):
        plt.ion()
        plt.show()
        plt.title('Random points')
        x_min, x_max = find_x_bounds(self.point_set)
        interval = Interval(x_min - 0, x_max + 0)
        y_min, y_max = find_y_bounds(self.point_set)
        prepare_axis(interval.l - 5, interval.r + 5, y_min - 5, y_max + 5)
        plot_point_set(self.point_set)
        plt.pause(1)

    def find_L_boundary(self):
        leftmost_x = min(self.point_set, key=lambda P: P.x).x
        leftmost_y = min(self.point_set, key=lambda P: P.x).y
        leftmost = Point(leftmost_x, leftmost_y)
        slope = [(p.y - leftmost.y) / (p.x - leftmost.x) for p in self.point_set if not p == leftmost]
        # slope_sorted = sorted(slope, reverse=True)
        slope_np = sorted(slope, reverse=True)[self.np - 1 - 1]
        slope_np_index = slope.index(slope_np)
        L_boundary_point = self.point_set[slope_np_index] if self.point_set.index(leftmost) > slope_np_index else \
            self.point_set[
                slope_np_index + 1]
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
            plt.title('Find L boundary')
            plot_point(leftmost, color='r')
            plot_line(self.L_boundary_line, color='g')
            plot_point_set(self.points_in_L, color='r')
            plot_point_set(self.points_in_L, color='r')
            plot_point_set(self.points_not_in_L, color='b')
            plot_point_set(self.points_in_L_boundary, color='g')
            plt.pause(1)
            end = input('Press enter to end the next step')

    def find_U_boundary(self):
        x0, y0 = self.points_in_L_boundary[0].x, self.points_in_L_boundary[0].y
        points_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_in_L]
        points_not_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_not_in_L]

        hamInstance = HamInstance(points_in_L_trans, points_not_in_L_trans)
        # hamInstance = HamInstance(self.points_in_L, self.points_not_in_L)
        above = True if self.L_boundary_line.m >= 0 else False
        ham_cuts_trans = HamCut().reduce_then_cut(hamInstance, self.mp, (self.np - self.mp), above=above)[0]
        ham_points_trans = [Point(self.x_min - 5, (self.x_min - 5) * ham_cuts_trans.m + ham_cuts_trans.b),
                            Point(self.x_max + 5, (self.x_max + 5) * ham_cuts_trans.m + ham_cuts_trans.b)]

        ham_points = [point_transfer_back(p, x0, y0, self.L_boundary_line) for p in ham_points_trans]

        self.U_boundary_line = line_over_two_points(ham_points[0], ham_points[1])
        self.points_in_U = [p for p in self.point_set if
                            p.y > p.x * self.U_boundary_line.m + self.U_boundary_line.b + 1e-5]
        self.points_not_in_U = [p for p in self.point_set if
                                p.y < p.x * self.U_boundary_line.m + self.U_boundary_line.b - 1e-5]
        self.points_in_U_boundary = [p for p in self.point_set if
                                     p not in self.points_in_U and p not in self.points_not_in_U]
        if self.plot:
            plt.title('Find U boundary')
            plot_line(self.U_boundary_line, color='k')
            plot_point_set(self.points_in_U, color='r')
            plot_point_set(self.points_not_in_U, color='b')
            plot_point_set(self.points_in_U_boundary, color='g')
            plt.pause(1)
            end = input('Press enter to end the next step')

    def find_D_boundary(self):
        x0, y0 = self.points_in_L_boundary[0].x, self.points_in_L_boundary[0].y
        points_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_in_L]
        points_not_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_not_in_L]

        hamInstance = HamInstance(points_in_L_trans, points_not_in_L_trans)
        # hamInstance = HamInstance(self.points_in_L, self.points_not_in_L)
        above = False if self.L_boundary_line.m >= 0 else True
        ham_cuts_trans = HamCut().reduce_then_cut(hamInstance, self.mp, (self.np - self.mp), above=above)[0]
        ham_points_trans = [Point(self.x_min - 5, (self.x_min - 5) * ham_cuts_trans.m + ham_cuts_trans.b),
                            Point(self.x_max + 5, (self.x_max + 5) * ham_cuts_trans.m + ham_cuts_trans.b)]

        ham_points = [point_transfer_back(p, x0, y0, self.L_boundary_line) for p in ham_points_trans]

        self.D_boundary_line = line_over_two_points(ham_points[0], ham_points[1])

        slope_reverse = 1 if y0 > x0 * self.D_boundary_line.m + self.D_boundary_line.b + 1e-5 else -1
        self.points_in_D = [p for p in self.point_set if
                            slope_reverse * p.y > slope_reverse * (
                                        p.x * self.D_boundary_line.m + self.D_boundary_line.b + 1e-5)]
        self.points_not_in_D = [p for p in self.point_set if
                                slope_reverse * p.y < slope_reverse * (
                                            p.x * self.D_boundary_line.m + self.D_boundary_line.b - 1e-5)]
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
            end = input('Press enter to end the next step')

    def find_R_boundary(self):
        x0, y0 = self.points_in_U_boundary[0].x, self.points_in_U_boundary[0].y
        points_in_U_trans = [point_transfer(p, x0, y0, self.U_boundary_line) for p in self.points_in_U]
        points_not_in_U_trans = [point_transfer(p, x0, y0, self.U_boundary_line) for p in self.points_not_in_U]

        hamInstance = HamInstance(points_in_U_trans, points_not_in_U_trans)

        above = True if self.U_boundary_line.m >= 0 else False
        ham_cuts_trans = HamCut().reduce_then_cut(hamInstance, self.mp, (self.np - self.mp), above=above)[0]
        ham_points_trans = [Point(self.x_min - 5, (self.x_min - 5) * ham_cuts_trans.m + ham_cuts_trans.b),
                            Point(self.x_max + 5, (self.x_max + 5) * ham_cuts_trans.m + ham_cuts_trans.b)]

        ham_points = [point_transfer_back(p, x0, y0, self.U_boundary_line) for p in ham_points_trans]

        self.R_boundary_line = line_over_two_points(ham_points[0], ham_points[1])
        self.points_in_R = [p for p in self.point_set if
                            p.y > p.x * self.R_boundary_line.m + self.R_boundary_line.b + 1e-5]
        self.points_not_in_R = [p for p in self.point_set if
                                p.y < p.x * self.R_boundary_line.m + self.R_boundary_line.b - 1e-5]
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
            end = input('Press enter to end the next step')

    def find_intersections(self):
        self.points_LU = [p for p in self.points_in_L if p in self.points_in_U]
        self.points_LD = [p for p in self.points_in_L if p in self.points_in_D]
        self.points_RU = [p for p in self.points_in_R if p in self.points_in_U]
        self.points_RD = [p for p in self.points_in_R if p in self.points_in_D]
        self.inter_LU = Intersection(self.L_boundary_line, self.U_boundary_line)

        if self.plot:
            plot_point_set(self.point_set, color='b')
            plot_point_set(self.points_LU, color='r')
            plot_point_set(self.points_LD, color='g')
            plot_point_set(self.points_RU, color='m')
            plot_point_set(self.points_RD, color='k')
            plt.pause(1)
            end = input('Press enter to end the next step')