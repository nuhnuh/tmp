#!/usr/bin/env python
# coding: utf-8

import math
import numpy as np
#import scipy
import scipy.spatial
import scipy.interpolate


# display TODO
#print( """\x1b[1;30;36m
print( """\x1b[5;30;36m
# TODO: nomenclatura: heading angle in ENU
# TODO: error (cte & heading) are wrong
# TODO: make a cache of curvature_origin_at_segment()
# TODO: distance_to_goal after goal should be negative
\x1b[0m""")



################################################################################
# Basic geometry elements


class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        #return 'Point(%.2f, %.2f)' % (self.x, self.y)
        return "Point({0.x:.2f}, {0.y:.2f})".format(self)

    def __str__(self):
        return "({0.x}, {0.y})".format(self)

    def __copy__(self):
        return self.__class__(self.x, self.y)

    def __eq__(self, other):
        #assert isinstance(other, Vector), 'WTF??'
        return self.x == other.x and self.y == other.y

    def __ne__(self, other):
        return not self.__eq__(other)

#    def __sub__(self, other):
#        assert isinstance(other, Point), 'WTF???'
#        return Vector(self.x - other.x, self.y - other.y)

    def distance(self, point):
        """point to point distance"""
        return math.hypot( self.x-point.x, self.y-point.y )



# based in https://github.com/ezag/pyeuclid/blob/master/euclid.py#L304
# see also: https://github.com/jan-mue/geometer/blob/master/geometer/
class Vector:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        #return 'Vector(%.2f, %.2f)' % (self.x, self.y)
        return "Vector({0.x:.2f}, {0.y:.2f})".format(self)

    def __str__(self):
        return "({0.x}, {0.y})".format(self)

    def __copy__(self):
        return self.__class__(self.x, self.y)

    def __eq__(self, other):
        #assert isinstance(other, Vector), 'WTF??'
        return self.x == other.x and self.y == other.y

    def __ne__(self, other):
        return not self.__eq__(other)

    def __add__(self, other):
        x, y = self.x + other.x, self.y + other.y
        return Vector(x,y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __neg__(self):
        return Vector(-self.x, -self.y)

    def __abs__(self):
        #return math.sqrt(self.x ** 2 + self.y ** 2)
        return math.hypot(self.x, self.y)

    norm = __abs__

    def norm_squared(self):
        return self.x ** 2 + self.y ** 2

    def normalize(self):
        d = self.__abs__()
        return Vector(self.x / d, self.y / d)

    def dot(self, other):
        """dot product"""
        return self.x * other.x + self.y * other.y

    def cross(self, other):
        """cross product"""
        return self.x * other.y - self.y * other.x

    def angle(self, other):
        """Return the angle to the vector other"""
        return math.atan2( self.cross(other), self.dot(other) )

    def heading(self):
        return math.atan2( self.y, self.x )


    def rot90(self):
        """Rotate 90 degrees (CCW)"""
        return Vector(-self.y, self.x)

    def rot180(self):
        """Rotate 180 degrees (CCW)"""
        return Vector(-self.x, -self.y)

    def rot270(self):
        """Rotate 270 degrees (CCW)"""
        return Vector(self.y, -self.x)



class Line:

    def __init__(self, point, vector):
        self.point = point
        self.vector = vector

    def __repr__(self):
        return 'Line(point=' + str(self.point) + ', vector=' + str(self.vector) + ')'

    def __copy__(self):
        return self.__class__(self.point, self.vector)


    def intersect(self, line):
        """find intersection point with line"""
        line1 = self
        line2 = line

        x1, y1 = line1.point.x, line1.point.y
        x2, y2 = line1.point.x + line1.vector.x, line1.point.y + line1.vector.y

        x3, y3 = line2.point.x, line2.point.y
        x4, y4 = line2.point.x + line2.vector.x, line2.point.y + line2.vector.y

        denom = (line1.vector.x)*(line2.vector.y) - (line1.vector.y)*(line2.vector.x)
        if denom == 0:
            return Point(np.Infinity, np.Infinity)
        denom = float(denom)

        x = ( (x1*y2-y1*x2)*(-line2.vector.x) - (-line1.vector.x)*(x3*y4-y3*x4) ) / denom
        y = ( (x1*y2-y1*x2)*(-line2.vector.y) - (-line1.vector.y)*(x3*y4-y3*x4) ) / denom
        return Point(x, y)

    def distance(self, point):
        """point to line distance"""
        # create a line 'line90' perpendicular to self that contains point 'point'
        line90 = Line( point, self.vector.rot90() )
        # find intersection
        ipoint = self.intersect(line90)
        #
        return math.hypot( ipoint.x-point.x, ipoint.y-point.y )



################################################################################
# Paths

class PathUtils:

    ###########################################################################
    # PathUtils create path

    @staticmethod
    def create_circular_path(npoints=6, radious=1): # 7 points => 6 sides
        t = np.linspace(0, 2*np.pi, npoints, endpoint=False)
        x = radious * np.cos(t)
        y = radious * np.sin(t)
        path = LinearPWPath(x,y)
        return path

    @staticmethod
    def create_lemniscate_path(npoints=8, scale=1):
        t = np.linspace(0, 2*np.pi, npoints, endpoint=False)
        x = 1.0 * np.sin(1.0*t)
        y = 0.4 * np.sin(2.0*t)
        path = LinearPWPath(scale*x, scale*y)
        return path

    @staticmethod
    def create_quore_path(npoints=7, scale=1):
        t = np.linspace(0, 2*np.pi, npoints, endpoint=False)
        x = 16 * np.sin(t)**3
        y = 13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t)
        path = LinearPWPath(scale*x/16., scale*y/16.)
        return path

    @staticmethod
    def create_spiral_path(npoints=10, nloops=1.0):
        x, y = [], []
        for i in range(npoints):
            t = i * ( nloops * 2 * np.pi ) / npoints
            x_ = (1 + 0.5 * t) * np.cos(t)
            y_ = (1 + 0.5 * t) * np.sin(t)
            x.append(x_)
            y.append(y_)
        x, y = np.array(x)[::-1], np.array(y)[::-1]
        path = LinearPWPath(x,y)
        return path

    @staticmethod
    def create_circular_arc_path(npoints, x0, y0, radious, theta0, theta1):
        import numpy
        t = numpy.linspace(0,1,npoints)
        x = x0 + radious * np.cos(np.deg2rad( theta0 + (theta1-theta0)*t ))
        y = y0 + radious * np.sin(np.deg2rad( theta0 + (theta1-theta0)*t ))
        path = LinearPWPath(x,y)
        return path

    @staticmethod
    def create_1segment_cspline(x0, y0, a0, x1, y1, a1, npoints):
        """
        """

        def rotate(origin, point, angle):
            """
            Rotate a point counterclockwise by a given angle around a given origin.
            The angle should be given in radians.
            """
            ox, oy = origin
            px, py = point

            qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
            qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
            return qx, qy

        d = np.linalg.norm( np.array([x0, y0]) - np.array([x1, y1]) )
        rot = np.arctan2( y1-y0, x1-x0 )
        m0_ = np.tan( a0 - rot )
        m1_ = np.tan( a1 - rot )

        #f = my_interp( [0,abs(d)], [0,0], [m0_, m1_] )
        f = scipy.interpolate.CubicSpline( [0, d], [0, 0], axis=0, bc_type=((1, m0_), (1, m1_)) , extrapolate=True)

        x_smooth = np.linspace(0, d, npoints)
        y_smooth = f(x_smooth)
        for idx in range(len(x_smooth)):
            x_smooth[idx], y_smooth[idx] = rotate( (0,0), (x_smooth[idx], y_smooth[idx]), rot )
            x_smooth[idx], y_smooth[idx] = x_smooth[idx] + x0, y_smooth[idx] + y0

        path = LinearPWPath(x_smooth, y_smooth)
        return path

    ###########################################################################
    # PathUtils transform path

    @staticmethod
    def scale(path, scale_x, scale_y):
        x = scale_x * path.x
        y = scale_y * path.y
        path = LinearPWPath(x,y)
        return path

    @staticmethod
    def move(path, x0, y0):
        x = x0 + path.x
        y = y0 + path.y
        path = LinearPWPath(x,y)
        return path

    @staticmethod
    def rotate(path, ccw_angle):
        x = np.cos(ccw_angle) * path.x - np.sin(ccw_angle) * path.y
        y = np.sin(ccw_angle) * path.x + np.cos(ccw_angle) * path.y
        path = LinearPWPath(x,y)
        return path

    @staticmethod
    def invert_direction(path):
        x = path.x[::-1]
        y = path.y[::-1]
        path = LinearPWPath(x,y)
        return path

    @staticmethod
    def cyclic_workaround(path):
        xi, yi = path.x[0], path.y[0]
        x, y = path.x, path.y
        #
        x1 = xi + 0.01*(x[+1]-xi)
        y1 = yi + 0.01*(y[+1]-yi)
        x0 = x1 + 0.005*(x[-1]-x1)
        y0 = y1 + 0.005*(y[-1]-y1)
        #
        x2n = xi + 0.01*(x[-1]-xi)
        y2n = yi + 0.01*(y[-1]-yi)
        x1n = x2n + 0.005*(x[+1]-x2n)
        y1n = y2n + 0.005*(y[+1]-y2n)
        #
        x = np.hstack(([x0, x1], x[1:], [x2n, x1n]))
        y = np.hstack(([y0, y1], y[1:], [y2n, y1n]))
        path = LinearPWPath( x, y )
        return path


class CircularPath:
    """
    Circular 2D path
    Asumes ENU frame convention (east, north, up). Positive angles are CCW
    """

    def __init__(self, x0, y0, R, ccw=True):
        self.x0 = x0
        self.y0 = y0
        self.R  = R
        self.ccw = ccw

    def __repr__(self):
        return 'CircularPath(x0=' + str(self.x0) + ',y0=' + str(self.y0) + ',R=' + str(self.R) + ',ccw=' + self.ccw + ')'

    def error(self, point):
        """Given a point, returns the cross-track error (cte) and heading"""

        point_vector = Vector(point.x-self.x0, point.y-self.y0)

        heading = point_vector.rot90().heading()

        if self.ccw:
            cte = self.R - point_vector.norm()
        else:
            cte = point_vector.norm() - self.R

        return cte, heading



class LinearPWPath:
    """
    Linear piece-wise 2D path
    Asumes ENU frame convention (east, north, up). Positive angles are CCW
    """

    def __init__(self, x, y):
        """ """

        assert len(x)==len(y)
        self.x = x
        self.y = y

        # create kdtree for fast nearest waypoint search
        X = np.vstack((x,y)).T
        self._kdtree = scipy.spatial.cKDTree(X)

        # precalculate subpaths lenghts
        segment_length = ( np.diff(x)**2 + np.diff(y)**2 )**0.5
        self._cum_length = np.hstack(( [0], segment_length.cumsum() ))

        # precalculate normals at waypoints
        segment_normal_dir = []
        for idx in range(len(x)-1):
            segment_dir = Vector( x[idx+1]-x[idx], y[idx+1]-y[idx] )
            segment_normal_dir_idx = segment_dir.rot90()
            segment_normal_dir.append( segment_normal_dir_idx.normalize() )
        #
        waypoint_normal_dir = [ segment_normal_dir[0] ]
        for idx in range(1,len(x)-1):
            #segment_dir = Vector( x[idx+1]-x[idx], y[idx+1]-y[idx] )
            waypoint_normal_dir_idx = segment_normal_dir[idx-1] + segment_normal_dir[idx]
            waypoint_normal_dir.append( waypoint_normal_dir_idx.normalize() )
        waypoint_normal_dir.append( segment_normal_dir[-1] )
        #
        self._waypoint_normal_dir = waypoint_normal_dir


    def __repr__(self):
        """  """
        #return 'Line(point=' + str(self.point) + ', vector=' + str(self.vector) + ')'
        raise 'TODO'


    def save(self, fn):
        np.savez(file=fn, x=self.x, y=self.y)


    @staticmethod
    def load(fn):
        data = np.load(fn)
        x, y = data['x'], data['y']
        return LinearPWPath(x,y)


    def _subpath_length(self, idx0, idx1):
        """  """
        cum_length = self._cum_length
        subpath_length = cum_length[idx1] - cum_length[idx0]
        return subpath_length


    def nearest_waypoint(self, point): # -> get_nearest_waypoint()
        """find index of nearest point"""
        #distances2 = ( self.x - point.x )**2 + ( self.y - point.y )**2
        #idx = np.argmin( distances2 )
        dist, idx = self._kdtree.query((point.x,point.y), k=1)
        return idx


    def normal_at_waypoint(self, waypoint_idx):
        """ """
        return self._waypoint_normal_dir[waypoint_idx]


    def curvature_origin_at_segment(self, segment_idx): # TODO: curvature_origin??
        """  """

        # just for cleaner code
        x = self.x
        y = self.y

        dir1 = self.normal_at_waypoint( segment_idx+0 )
        dir2 = self.normal_at_waypoint( segment_idx+1 )

        line1 = Line( Point(x[segment_idx+0],y[segment_idx+0]), dir1 )
        line2 = Line( Point(x[segment_idx+1],y[segment_idx+1]), dir2 )

        ipoint = line1.intersect(line2)
        return ipoint


    def nearest_segment(self, point):
        """
        """

        # just for cleaner code
        x = self.x
        y = self.y

        idx = self.nearest_waypoint(point)
    #     print('point', point)
    #     print('nearest_waypoint', idx)
        #
        point_dir = Vector( point.x - x[idx], point.y - y[idx] ) # position in locar coords
        if idx == 0:
            # point is near the first waypoint (path[0]), but it can be ahead or behind
            path_dir = Vector( x[idx+1]-x[idx], y[idx+1]-y[idx] )
            if path_dir.dot( point_dir ) > 0: # point is ahead path[0]
                return 0
            # point is behind path[0]
            return -1
        elif idx == len(x)-1:
            # point is near last waypoint (path[end]), but it can be ahead or behind
            path_dir = Vector( x[idx]-x[idx-1], y[idx]-y[idx-1] )
            if path_dir.dot( point_dir ) < 0: # point behind path[end]
                return len(x)-2
            # point ahead path[end]
            return len(x)-1
        else:
            # point is not near the ends of the path
            ahead_dir  = Vector( x[idx+1]-x[idx], y[idx+1]-y[idx] )
            behind_dir = Vector( x[idx]-x[idx-1], y[idx]-y[idx-1] )
            ahead_normal_dir  = ahead_dir.rot90()
            behind_normal_dir = behind_dir.rot90()
            path_normal_dir = ahead_normal_dir.normalize() + behind_normal_dir.normalize()
            if path_normal_dir.cross( point_dir ) < 0: # point is ahead the path's nearest point
                return idx
            return idx-1

        raise 'WTF??'


    def error(self, point, nearest=None):
        """Given a point, returns the cross-track error (cte) and heading"""

        # just for cleaner code
        x = self.x
        y = self.y

        idx = self.nearest_waypoint(point)
        #
        dir0 = Vector( point.x - x[idx], point.y - y[idx] ) # position in locar coords
        if idx == 0:
            # point is near point path[0]
            # => point to path distance is equal to point to line path[0]->path[1] distance
            #
            # cross track error (cte)
            segment_hdng = Vector( x[idx+1]-x[idx], y[idx+1]-y[idx] )
            segment_line = Line( Point(x[idx],y[idx]), segment_hdng )
            d = segment_line.distance(point)
            cte = math.copysign(d, segment_hdng.cross(dir0)) # segment_hdng.cross(dir0) > 0 => on the left
            # heading
            heading = math.atan2(segment_hdng.y, segment_hdng.x)
            #
            return cte, heading
        elif idx == len(x)-1:
            # point is near point path[end]
            # => point to path distance is equal to point to line path[end-1]->path[end] distance
            #
            # cross track error (cte)
            segment_hdng = Vector( x[idx]-x[idx-1], y[idx]-y[idx-1] )
            segment_line = Line( Point(x[idx],y[idx]), segment_hdng )
            d = segment_line.distance(point)
            cte = math.copysign(d, segment_hdng.cross(dir0)) # segment_hdng.cross(dir0) > 0 => on the left
            # heading
            heading = math.atan2(segment_hdng.y, segment_hdng.x)
            #
            return cte, heading
        else:
            # first of all calculate point to path distance

            # point is not near the ends of the path => 3 possible cases:
            # 1) 'point' to path distance is equal to 'point' to line path[idx-1]->path[idx] distance
            # 2) 'point' to path distance is equal to 'point' to line path[idx]->path[idx+1] distance
            # 3) 'point' to path distance is equal to 'point' to point path[idx] distance
            d1 = np.Inf
            d2 = np.Inf
            d3 = np.Inf
            # case 1
            dir1 = Vector( x[idx-1] - x[idx], y[idx-1] - y[idx] ) # back segment -heading
            if dir1.dot(dir0) > 0: # if dir_diff_smaller_than_90( dir1, dir0 ):
                line = Line( Point( x[idx], y[idx] ),  dir1 )
                d1 = line.distance( point )
            # case 2
            dir2 = Vector( x[idx+1] - x[idx], y[idx+1] - y[idx] ) # front segmet heading
            if dir2.dot(dir0) > 0: # if dir_diff_smaller_than_90( dir2, dir0 ):
                line = Line( Point( x[idx], y[idx] ),  dir2 )
                d2 = line.distance( point )
            # case 3
            d3 = point.distance(Point(x[idx],y[idx]))

            # find which of the 3 cases applies and
            # calculate cross-track error (cte) from point to path distance
            idx2 = np.argmin(np.array([d1,d2,d3]))
            if idx2 == 0:
                # cross track error (cte)
                cte = math.copysign(d1, -dir1.cross(dir0)) # XXXXXXXXXXX
                # heading
                heading = math.atan2(-dir1.y, -dir1.x)
                #
                return cte, heading
            if idx2 == 1:
                # cross track error (cte)
                cte = math.copysign(d2, dir2.cross(dir0)) # XXXXXXXXXXX
                # heading
                heading = math.atan2(dir2.y, dir2.x)
                #
                return cte, heading
            if idx2 == 2:
                # cross track error (cte)
                dir1 = dir1.normalize()
                dir2 = dir2.normalize()
                dir_corner = (-dir1) + dir2
                #dir_corner90 = dir_corner.rot90()
                cte = math.copysign(d3, dir_corner.cross(dir0)) # XXXXXXXXXXX
                # heading
                if dir_corner.cross(dir0) < 0:
                    heading = dir0.rot90().heading()
                else:
                    heading = dir0.rot270().heading()
                #
                return cte, heading

        raise 'WTF??'


    def project_point(self, point):
        """
        Return the projection of the point into the path.
        Also returns the direction of the path at the projected point
        """

        # just for cleaner code
        x = self.x
        y = self.y

        segment_idx = self.nearest_segment(point)
    #     print('segment_idx', segment_idx)
        if segment_idx < 0: # point is behind the first path segment
            segment_dir = Vector(x[1],y[1]) - Vector(x[0],y[0])
            segment = Line( Point(x[0],y[0]), segment_dir )
            #normal_dir = segment_dir.rot90()
            normal_dir = self._waypoint_normal_dir[0]
            normal  = Line( point, normal_dir )
            ppoint = segment.intersect(normal)
            path_dir = segment_dir
            return ppoint, path_dir, Point(np.Infinity, np.Infinity)
        if segment_idx > len(x)-2: # point is ahead the last path segment
            segment_dir = Vector(x[-1],y[-1]) - Vector(x[-2],y[-2])
            segment = Line( Point(x[-1],y[-1]), segment_dir )
            #normal_dir = segment_dir.rot90()
            normal_dir = self._waypoint_normal_dir[-1]
            normal  = Line( point, normal_dir )
            ppoint = segment.intersect(normal)
            path_dir = segment_dir
            return ppoint, path_dir, Point(np.Infinity, np.Infinity)

        icr = self.curvature_origin_at_segment(segment_idx) # instantaneous center of rotation
    #     print('icr', icr)
        if icr.x is np.Infinity:
            segment_dir = Vector(x[segment_idx+1], y[segment_idx+1]) - Vector(x[segment_idx], y[segment_idx])
            segment = Line( Point(x[segment_idx],y[segment_idx]), segment_dir )
            #normal_dir = segment_dir.rot90()
            normal_dir = self._waypoint_normal_dir[segment_idx]
            normal  = Line( point, normal_dir )
            ppoint = segment.intersect(normal)
            path_dir = segment_dir
            return ppoint, path_dir, Point(np.Infinity, np.Infinity)

        segment_dir = Vector(x[segment_idx+1], y[segment_idx+1]) - Vector(x[segment_idx], y[segment_idx])
        segment = Line( Point(x[segment_idx],y[segment_idx]), segment_dir )
        pseudo_normal_dir = Vector( point.x-icr.x, point.y-icr.y )
        pseudo_normal  = Line( icr, pseudo_normal_dir )
        ppoint = segment.intersect(pseudo_normal)
        #
        if np.sign(segment_dir.cross( pseudo_normal_dir )) < 0:
            path_dir = pseudo_normal_dir.rot90()
        else:
            path_dir = pseudo_normal_dir.rot270()
        #
        return ppoint, path_dir, icr


    def subpath_length(self, point1, point2):
        """
        Calculates the distance along the path
        (from the projection of point1 over the path to the projection of point2 over the path)
        """
        #import warnings; import sys; warnings.warn('TODO: lpwp_projection_length() causes discontinuities in the internal region of corners!')

        ppoint1, _, _ = self.project_point(point1)
        ppoint2, _, _ = self.project_point(point2)
    #     ahead_point1_idx, _  = lpwp_get_point_ahead_and_behind_v2( path, point1 )
    #     _, behing_point2_idx = lpwp_get_point_ahead_and_behind_v2( path, point2 );
    #     if ahead_point1_idx is None:
    #         return 0
    #     if behing_point2_idx is None:
    #         return 0
        #
        def ahead_and_behind(segment_idx):
            ahead_idx, behind_idx = segment_idx+1, segment_idx
            if segment_idx < 0:
                ahead_idx, behind_idx = 0, None
            elif segment_idx > len(self.x)-2:
                ahead_idx, behind_idx = None, len(self.x)-1
            return ahead_idx, behind_idx
        ahead_point1_idx, _  = ahead_and_behind( self.nearest_segment(point1) )
        _, behind_point2_idx = ahead_and_behind( self.nearest_segment(point2) )
        if ahead_point1_idx is None:
            return 0
        if behind_point2_idx is None:
            return 0
        #
    #     point1_segment = path.nearest_segment(point1)
    # #     if point1_segment < 0:
    # #         return 0
    #     point2_segment = path.nearest_segment(point2)
    # #     if point2_segment > len(path.x)-2:
    # #         return 0
    #     point2_segment = min( point2_segment, len(path.x)-2 )
    #     point1_segment = min( point1_segment, point2_segment )
    #     ahead_point1_idx  = point1_segment + 1
    #     behind_point2_idx = point2_segment
    # #     if ahead_point1_idx > len(path.x)-1:
    # #         print(point1_segment, point2_segment)
        #
        length_intermediate = self._subpath_length( ahead_point1_idx, behind_point2_idx )
        length_begining = ppoint1.distance(Point( self.x[ahead_point1_idx],  self.y[ahead_point1_idx]  ))
        length_end      = ppoint2.distance(Point( self.x[behind_point2_idx], self.y[behind_point2_idx] ))
        length = length_begining + length_intermediate + length_end
    #     print('length_intermediate:', length_intermediate)
    #     print('length_begining:    ', length_begining)
    #     print('length_end:         ', length_end)
    #     print('projection_length length:', length)
        return length


    def distance_to_goal(self, point):
        """ """
        path_end = Point( self.x[-1], self.y[-1] )
        return self.subpath_length( point, path_end )


    # def pursuit_point(self, point, adist):
    #     # TODO:
    #
    #     #raise 'TODO'
    #
    #     segment_idx = self.nearest_segment(point)
    #     #if segment_idx < 0:
    #     #    segment_idx = -1
    #     #    print('WTF A')
    #     #    return
    #     #if segment_idx > len(self.x) - 2:
    #     #    segment_idx = 0
    #     #    print('WTF B')
    #     #    return
    #     heading_vec = self.normal_at_waypoint( segment_idx+1 ).rot270()
    #     ppoint, _ = self.project_point(point)
    #     pursuit_point = Point( ppoint.x + adist * heading_vec.x, ppoint.y + adist * heading_vec.y )
    #
    #     return pursuit_point


class CubicPWPath:
    """
    Cubic piece-wise 2D path
    Asumes ENU frame convention (east, north, up). Positive angles are CCW
    """

    def __init__(self, x, y, heading):
        assert len(x)==len(y)==len(heading)
        self.x = x
        self.y = y
        self.heading = heading

    def __repr__(self):
        #return 'Line(point=' + str(self.point) + ', vector=' + str(self.vector) + ')'
        raise 'TODO'



################################################################################
# Demos



def _create_path(demo_name):

    if demo_name == 'spiral_path':
        path = PathUtils.create_spiral_path(npoints=14, nloops=1.5)
    elif demo_name == 'circular_path':
        path = PathUtils.create_circular_path(npoints=6)
    elif demo_name == 'quore_path':
        path = PathUtils.create_quore_path(npoints=16)
    elif demo_name == 'lemnicaste_path':
        path = PathUtils.create_lemniscate_path(npoints=16)
        path = LinearPWPath( path.x[1:], path.y[1:] )
    else:
        x = [  1,  2,  2,  1,  1,  2,  1, -1, -2, -1, -1, -2, -2, -1 ]
        y = [ -2, -2, -1, -1,  0,  1,  2,  2,  1,  0, -1, -1, -2, -2 ]
        x, y = np.array(x).astype('float'), np.array(y).astype('float')
        path = LinearPWPath( x, y )
    return path



def _demo_normal_at_path_waypoint():

    path = PathUtils.create_lemniscate_path(npoints=18)
    path = LinearPWPath( path.x[1:], path.y[1:] )
    #
    path = PathUtils.invert_direction( path )
    path = PathUtils.scale( path, 2, 2 )
    path = PathUtils.rotate( path, np.deg2rad(45) )
    path = PathUtils.move( path, 10, 10 )

    import matplotlib.pyplot as plt
    plt.figure()
    plt.title('normals at waypoints')
    plt.plot(path.x,     path.y,     '-k', label='path')
    plt.plot(path.x[-1], path.y[-1], 'ok', label='goal')
    for idx in range(len(path.x)):
        normal = path.normal_at_waypoint( idx )
        normal = Vector( normal.x/5, normal.y/5 )
        plt.plot( [path.x[idx], path.x[idx]+normal.x], [path.y[idx], path.y[idx]+normal.y], '-b' )
    plt.legend()
    plt.axis('equal')
    plt.show()


def _demo_nearest_segment_in_path(demo_name):

    # create the path
    path = _create_path(demo_name)
    x, y = path.x, path.y

    # create a grid including the path and find the nearest segment in the path
    xx, yy = np.meshgrid( np.linspace(x.min()-.5, x.max()+.5, 100), np.linspace(y.min()-.5, y.max()+.5, 100) )
    nearest_segment = np.nan * np.zeros_like(xx)
    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            point = Point(xx[i,j],yy[i,j])
            nearest_segment[i,j] = path.nearest_segment( point )

    # visualize
    import matplotlib.pyplot as plt
    plt.figure()
    plt.title('nearest path segment')
    # nearest segments
    #plt.imshow(nearest_segment, extent=[xx.min(), xx.max(), yy.min(), yy.max()], origin='lower', cmap='RdYlGn')
    plt.imshow(nearest_segment, extent=[xx.min(), xx.max(), yy.min(), yy.max()], origin='lower', cmap='plasma')
    # path
    plt.plot(x, y, '.-k', label='path')
    plt.plot(x[-1],y[-1],'ok',label='goal')
    plt.axis('equal')
    plt.legend()
    #plt.colorbar()
    #
    plt.show()


def _demo_project_point_into_path(demo_name):

    # create the path
    path = _create_path(demo_name)
    x, y = path.x, path.y

    # create random points
    points  = []
    for k in range(500):
        pt_x = np.random.uniform( low=x.min()-.5, high=x.max()+.5 )
        pt_y = np.random.uniform( low=y.min()-.5, high=y.max()+.5 )
        pt = Point( pt_x, pt_y )
        points.append(pt)
    ## replace random points by points arround a circle
    #points  = []
    #for k in range(200):
    #    pt_x = 4 * np.cos( 2*np.pi*k/200 )
    #    pt_y = 4 * np.sin( 2*np.pi*k/200 )
    #    pt = Point( pt_x, pt_y )
    #    points.append(pt)

    # project random points into the path
    ppoints = []
    for pt in points:
        ppt, _, _ = path.project_point( pt )
        ppoints.append(ppt)

    # create a grid including the path and find the nearest segment in the path
    xx, yy = np.meshgrid( np.linspace(x.min()-.5, x.max()+.5, 100), np.linspace(y.min()-.5, y.max()+.5, 100) )
    path_heading = np.nan * np.zeros_like(xx)
    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            point = Point(xx[i,j],yy[i,j])
            path_heading[i,j] = np.rad2deg(path.project_point( point )[1].heading())

    # visualize
    import matplotlib.pyplot as plt
    plt.figure()
    plt.title('project points into the path (bg: path heading)')
    # path heading
    plt.imshow(path_heading, extent=[xx.min(), xx.max(), yy.min(), yy.max()], origin='lower', cmap='hsv')
    # points projections
    for pt, ppt in zip(points, ppoints):
        plt.plot( [pt.x, ppt.x], [pt.y, ppt.y], '-k', alpha=0.1 )
    plt.plot( [pt.x for pt in points],  [pt.y for pt in points], '.k', alpha=0.1, label='random points')
    # path
    plt.plot(x, y, '.-k', label='path')
    plt.plot(x[-1],y[-1],'ok',label='goal')
    plt.axis('equal')
    plt.legend()
    plt.colorbar()
    #
    plt.show()


def _demo_distance_to_goal(demo_name):

    # create the path
    path = _create_path(demo_name)
    x, y = path.x, path.y

    # create a grid including the path and find the nearest segment in the path
    xx, yy = np.meshgrid( np.linspace(x.min()-.5, x.max()+.5, 100), np.linspace(y.min()-.5, y.max()+.5, 100) )
    dist_to_goal = np.nan * np.zeros_like(xx)
    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            point = Point(xx[i,j],yy[i,j])
            dist_to_goal[i,j] = path.distance_to_goal( point )

    # visualize
    import matplotlib.pyplot as plt
    plt.figure()
    plt.title('distance to path end')
    # nearest segments
    #plt.imshow(nearest_segment, extent=[xx.min(), xx.max(), yy.min(), yy.max()], origin='lower', cmap='RdYlGn')
    plt.imshow(dist_to_goal, extent=[xx.min(), xx.max(), yy.min(), yy.max()], origin='lower', cmap='viridis_r')
    # path
    plt.plot(x, y, '.-k', label='path')
    plt.plot(x[-1],y[-1],'ok',label='goal')
    plt.axis('equal')
    plt.legend()
    #plt.colorbar()
    #
    plt.show()


def _demo_cte_CircularPath():
    """  """

    import matplotlib.pyplot as plt

    x0, y0 = 1, 1
    R = 2
    x, y = x0+R*np.cos(2*np.pi*np.linspace(0,1,50)), y0+R*np.sin(2*np.pi*np.linspace(0,1,50))
    path = CircularPath( x0, y0, R, ccw=True )

    xx, yy = np.meshgrid( np.linspace(x0-R-.5, x0+R+.5, 300), np.linspace(y0-R-0.5, y0+R+.5, 300) )
    cte     = np.nan * np.zeros_like(xx)
    heading = np.nan * np.zeros_like(xx)
    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            position = Point( xx[i,j], yy[i,j] )
            cte[i,j], heading[i,j] = path.error(position)


    plt.figure()
    #
    plt.subplot(1,2,1)
    plt.title('cross track error') # cross track error
    plt.plot(x,y,':k',label='reference path')
    plt.imshow(cte
        , extent=[xx.min(), xx.max(), yy.min(), yy.max()], origin='lower', cmap='RdYlGn_r')
    plt.clim(-1,+1)
    plt.colorbar()
    #
    plt.subplot(1,2,2)
    plt.title('heading') # cross track error
    plt.plot(x,y,':k',label='reference path')
    plt.imshow(np.rad2deg(heading) # ENU coord frame convention (east, north, up)
    #plt.imshow(np.rad2deg(np.arctan2(-dir_x,dir_y)) # NED coords frame convention (north, east, down)
        , extent=[xx.min(), xx.max(), yy.min(), yy.max()], origin='lower', cmap='hsv')
    plt.clim(-180,+180)
    plt.colorbar()
    #
    plt.show()


def _demo_cte(demo_name):
    """Cross track error and heading on a path"""

    # create the path
    path = _create_path(demo_name)
    x, y = path.x, path.y

    # create a grid including the path and calculate cte and heading at each point of the path
    xx, yy = np.meshgrid( np.linspace(x.min()-.5, x.max()+.5, 100), np.linspace(y.min()-.5, y.max()+.5, 100) )
    cte     = np.nan * np.zeros_like(xx)
    heading = np.nan * np.zeros_like(xx)
    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            position = Point( xx[i,j], yy[i,j] )
            cte[i,j], heading[i,j] = path.error(position)

    # visualize cte and path
    import matplotlib.pyplot as plt
    plt.figure()
    #
    plt.subplot(1,2,1)
    plt.title('cross track error') # cross track error
    plt.plot(x,y,':k',label='reference path')
    plt.plot(x[-1],y[-1],'ok',label='goal')
    plt.contour(xx, yy, cte, 30, cmap='RdYlGn_r' )
    plt.axis('equal')
    plt.clim(-1,+1)
    #plt.colorbar()
    #
    plt.subplot(1,2,2)
    plt.title('cross track error and heading')
    plt.plot(x,y,':k',label='reference path')
    plt.plot(x[-1],y[-1],'ok',label='goal')
    plt.streamplot( xx, yy, np.cos(heading), np.sin(heading), color=cte, density=1.5, cmap='RdYlGn_r' )
    plt.axis('equal')
    plt.clim(-1,+1)
    #plt.colorbar()
    #
    #plt.show()

    # visualize cte and path (alternative visualization)
    plt.figure()
    #
    plt.subplot(1,2,1)
    plt.title('cross track error') # cross track error
    plt.plot(x,y,':k',label='reference path')
    plt.plot(x[-1],y[-1],'ok',label='goal')
    plt.imshow(cte
        , extent=[xx.min(), xx.max(), yy.min(), yy.max()], origin='lower', cmap='RdYlGn_r')
    plt.clim(-1,+1)
    plt.colorbar()
    #
    plt.subplot(1,2,2)
    plt.title('heading') # cross track error
    plt.plot(x,y,':k',label='reference path')
    plt.plot(x[-1],y[-1],'ok',label='goal')
    plt.imshow(np.rad2deg(heading) # ENU coord frame convention (east, north, up)
    #plt.imshow(np.rad2deg(np.arctan2(-dir_x,dir_y)) # NED coords frame convention (north, east, down)
        , extent=[xx.min(), xx.max(), yy.min(), yy.max()], origin='lower', cmap='hsv')
    plt.clim(-180,+180)
    plt.colorbar()
    #
    plt.show()


if __name__ == '__main__':

    _demo_normal_at_path_waypoint()

    # TODO: origins of curvature

    #_demo_nearest_segment_in_path('circular_path')
    #_demo_nearest_segment_in_path('spiral_path')
    #_demo_nearest_segment_in_path('quore_path')
    _demo_nearest_segment_in_path('lemnicaste_path')

    #_demo_project_point_into_path('circular_path')
    #_demo_project_point_into_path('spiral_path')
    #_demo_project_point_into_path('quore_path')
    _demo_project_point_into_path('lemnicaste_path')

    #_demo_distance_to_goal('circular_path')
    #_demo_distance_to_goal('spiral_path')
    #_demo_distance_to_goal('quore_path')
    _demo_distance_to_goal('lemnicaste_path')

    # cte & heading demos
    _demo_cte_CircularPath()
    #_demo('default')
    _demo_cte('spiral_path')
    #_demo('eliptical_path')
