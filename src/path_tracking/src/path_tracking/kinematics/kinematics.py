#!/usr/bin/env python
# coding: utf-8

import math
import numpy as np


# display TODO
#print( """\x1b[1;30;36m
print( """\x1b[5;30;36m
\x1b[0m""")



################################################################################
# Kinematics


def plan_velocity_change_1dof( a_max, v_0, v_g, debug=False ):

    """
    plans a velocity change (velocity has not discontinuities)
    returns the required info to perform the desired velocity change

    Params:
        kinematic_constraints:
            a_max: maximum acceleration (constraint, absolute value)
            jerk:  jerk (constraint, absolute value)
        boundary conditions:
            v_0: initial velocity
            a_0: initial acceleration
            v_g: final velocity
            a_g: final acceleration

    Returns:
        (dt1, dt2, dt3), jerks, velocity_change_distance
    """

    # a_max and jerk are absolute values
    assert( a_max > 0 )

    # velocity increase and velocity decrease are simetrical
    if v_g <= v_0: # deceleration
        a_max = - a_max

    # time interval
    dt = (v_g-v_0) / a_max


    # position displacement
    dx = v_0 * dt + 1./2 * a_max * dt**2

    # distance traveled during the velocity change
    velocity_change_distance = dx

    if debug:
        print('v_0, v_g = %f, %f' % (v_0, v_g))
        print('dt = %f' % dt)
        print('velocity_change_distance = %f' % velocity_change_distance)

    return dt, velocity_change_distance

def calc_vel_from_d2g_1dof( d2g, a_max ):
    """
    given a brake plan, calculates the velocity of the plan at the distance to goal d2g
    a_max: absolute value of the acceleration in the brake plan (kinematic constraint)
    """

    ## time interval
    #dt = (v_max-0) / a_max

    # find current t form distance to goal (d2g)
    # since d2g = 1./2 * a_max * t**2
    t = (2 * d2g / a_max )**.5

    # calculate the velocity at t
    v = a_max * t

    return v



def plan_velocity_change( a_max, jerk, v_0, v_g, a_0=0, a_g=0, debug=False ):
    """
    continuous-acceleration velocity change (acceleration has not discontinuities)
    returns the required info to perform the desired velocity change

    The plan has three intervals
        dt1: acceleration increases
        dt2: acceleration remains constant
        dt3: acceleration decreases

    Params:
        kinematic_constraints:
            a_max: maximum acceleration (constraint, absolute value)
            jerk:  jerk (constraint, absolute value)
        boundary conditions:
            v_0: initial velocity
            a_0: initial acceleration
            v_g: final velocity
            a_g: final acceleration

    Returns:
        (dt1, dt2, dt3), (dx1, dx2, dx3), (v1, v2), jerks, velocity_change_distance
    """

    # a_max and jerk are absolute values
    assert( a_max > 0 )
    assert( jerk > 0 )

    # velocity increase and velocity decrease are simetrical
    if v_g <= v_0: # deceleration
        jerk  = - jerk
        a_max = - a_max

    # the velocity change plan has three intervals
    #   dt1 acceleration increase interval
    #   dt2 constant acceleration interval (depends on dt1 and dt3)
    #   dt3 acceleration decrease interval
    dt1 = (a_max-a_0) / +jerk # dt1 goes from t0 to t1 (t1 = time when a_max is reached)
    dt3 = (a_g-a_max) / -jerk # dt3 goes from t2 to t3 (period where a goes from a_max to a_g)

    # velocity after dt1 (acceleration increase interval)
    v1 = v_0 + a_0 * dt1 + 1./2 * jerk * dt1**2
    # velocity before dt3 (acceleration decrease interval)
    v2 = v_g - a_g * dt3 - 1./2 * jerk * dt3**2

    # dt2 (constant acceleration interval)
    dt2 = (v2 - v1) / a_max

    if dt2 < 0: # proposed solution is not valid => find a solution not trying to reach a_max
        if debug:
            print('max acceleration will not be applied')
            print('[TMP] dt1, dt2, dt3 = %f, %f, %f' % (dt1, dt2, dt3))
            print('[TMP] v1, v2 = %f, %f' % (v1, v2))

        # recalculate acceleration change intervals so that dt2 = 0 (fastest transition possible)
        # (check the .ipynb to find the derivation of this equations)
        # (a not optimal alternative could be to iteratively reduce a_max until dt2 >= 0)
        dt2 = 0
        if v_g > v_0: # acceleration
            dt1 = -(a_0 - a_g - jerk*(-a_g/jerk + np.sqrt(2)*np.sqrt(a_0**2 + a_g**2 - 2*jerk*v_0 + 2*jerk*v_g)/(2*jerk)))/jerk
            dt3 = -a_g/jerk + np.sqrt(2)*np.sqrt(a_0**2 + a_g**2 - 2*jerk*v_0 + 2*jerk*v_g)/(2*jerk)
        else: # deceleration
            dt1 = -(a_0 - a_g - jerk*(-a_g/jerk - np.sqrt(2)*np.sqrt(a_0**2 + a_g**2 - 2*jerk*v_0 + 2*jerk*v_g)/(2*jerk)))/jerk
            dt3 = -a_g/jerk - np.sqrt(2)*np.sqrt(a_0**2 + a_g**2 - 2*jerk*v_0 + 2*jerk*v_g)/(2*jerk)

        # velocity after dt1 (acceleration increase interval)
        v1 = v_0 + a_0 * dt1 + 1./2 * jerk * dt1**2
        # velocity before dt3 (acceleration decrease interval)
        v2 = v_g - a_g * dt3 - 1./2 * jerk * dt3**2

    # position increase in each interval
    # acceleration increase interval
    dx1 = v_0 * dt1 + 1./2 * a_0 * dt1**2  + 1./6 * jerk * dt1**3
    # acceleration decrease interval
    dx3 = v_g * dt3 - 1./2 * a_g * dt3**2  - 1./6 * jerk * dt3**3
    # constant acceleration interval
    dx2 = v1 * dt2 + 1./2 * a_max * dt2**2

    # distance traveled during the velocity change
    np.testing.assert_equal( np.array([dt1, dt2, dt3]) >= -1e-6, np.ones(3), err_msg='WTF?? time intervals cannot be negative!! dt1,dt2,dt3=%f, %f, %f'%(dt1,dt2,dt3) )
    velocity_change_distance = dx1 + dx2 + dx3

    # jerk at each interval
    jerks = jerk, 0, -jerk

    if debug:
        print('dt1, dt2, dt3 = %f, %f, %f' % (dt1, dt2, dt3))
        print('jerk1, jerk3 = %f, %f' % (jerks[0], jerks[-1]))
        print('v1, v2 = %f, %f' % (v1, v2))
        print('dx1, dx2, dx3 = %f, %f, %f' % (dx1, dx2, dx3))
        print('velocity_change_distance = %f' % velocity_change_distance)

    return (dt1, dt2, dt3), (dx1, dx2, dx3), (v1, v2), jerks, velocity_change_distance



################################################################################
# Demos



def _sim_and_plot_solution( solution, a_max, jerk, v_0, v_g, a_0, a_g ):

    (dt1, dt2, dt3), (dx1, dx2, dx3), (v1, v2), jerks, velocity_change_distance = solution


    # simulation to check estimations
    t1 = dt1
    t2 = dt1 + dt2
    t3 = dt1 + dt2 + dt3
    #
    t = np.linspace(0,2*t3,1000)
    dt = t[1]-t[0]
    j = np.nan * np.zeros_like(t)
    for idx in range(len(t)):
        j[idx] = jerks[0]
        if t[idx] > t1:
            j[idx] = 0
        if t[idx] > t2:
            j[idx] = jerks[-1]
        if t[idx] > t3:
            j[idx] = 0
    a = a_0 + np.cumsum( dt * j ); #a = np.hstack(([0], a[:-1]))
    v = v_0 + np.cumsum( dt * a );    #v = np.hstack(([0], v[:-1]))
    x = np.cumsum( dt * v );          #x = np.hstack(([0], x[:-1]))

    # visualize
    import matplotlib.pyplot as plt
    plt.figure(figsize=(9.5,7))
    #
    ax = plt.subplot(4,1,1)
    plt.plot( t, j, '.-r', label='jerk' )
    plt.axvline(t1, color='k', linestyle=':')
    plt.axvline(t2, color='k', linestyle=':')
    plt.axvline(t3, color='k', linestyle=':')
    plt.legend()
    #plt.tight_layout()
    #
    plt.subplot(4,1,2, sharex=ax)
    plt.plot( t, a, '.-g', label='acceleration' )
    plt.legend()
    #
    plt.subplot(4,1,3, sharex=ax)
    plt.plot( t, v, '.-b', label='velocity' )
    plt.legend()
    #
    plt.subplot(4,1,4, sharex=ax)
    plt.plot( t, x, '.-c', label='distance' )
    plt.legend()
    #
    plt.tight_layout()
    plt.show()


def _demo_velocity_increase():
    # speed up
    print('velocity increase demo')
    a_max, jerk = .5, 1.5
    v_0, v_g = .0, .5
    a_0, a_g = .0, .0
    solution = plan_velocity_change( a_max=a_max, jerk=jerk, v_0=v_0, v_g=v_g, a_0=a_0, a_g=a_g, debug=True)
    _sim_and_plot_solution( solution, a_max=a_max, jerk=jerk, v_0=v_0, v_g=v_g, a_0=a_0, a_g=a_g )


def _demo_velocity_decrease():
    # slow down
    print('velocity decrease demo')
    a_max, jerk = .5, 1.5
    v_0, v_g = .5, .0
    a_0, a_g = .0, .0
    solution = plan_velocity_change( a_max=a_max, jerk=jerk, v_0=v_0, v_g=v_g, a_0=a_0, a_g=a_g, debug=True)
    _sim_and_plot_solution( solution, a_max=a_max, jerk=jerk, v_0=v_0, v_g=v_g, a_0=a_0, a_g=a_g )


def _demo():
    # slow down
    print('stop from t2')
    a_max, jerk = .5, 1.5
    v_0, v_g = 0.083333, .0
    a_0, a_g = -a_max, .0
    solution = plan_velocity_change( a_max=a_max, jerk=jerk, v_0=v_0, v_g=v_g, a_0=a_0, a_g=a_g, debug=True)
    _sim_and_plot_solution( solution, a_max=a_max, jerk=jerk, v_0=v_0, v_g=v_g, a_0=a_0, a_g=a_g )


################################################################################
# Main



def _main():

    _demo_velocity_increase()
    _demo_velocity_decrease()
    _demo()



if __name__ == '__main__':
   _main()
