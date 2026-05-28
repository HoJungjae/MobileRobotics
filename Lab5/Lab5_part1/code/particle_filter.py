import copy
from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np
# ------------------------------------------------------------------------
def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- noisy odometry measurements, a pair of robot pose, i.e. last time
                step pose and current time step pose

        Returns: the list of particle representing belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    alpha1 = 0.001
    alpha2 = 0.001
    alpha3 = 0.005
    alpha4 = 0.005

    # get all values for the calculation
    for particle in particles:
        old_x, old_y, old_theta = particle.xyh
    
        x_bar_prev = odom[0][0]
        y_bar_prev = odom[0][1]
        theta_bar_prev = odom[0][2]

        x_bar_cur = odom[1][0]
        y_bar_cur = odom[1][1]
        theta_bar_cur = odom[1][2]

        # start calculation
        rot1 = math.atan2(y_bar_cur - y_bar_prev, x_bar_cur - x_bar_prev) - theta_bar_prev
        trans = math.sqrt(math.pow(x_bar_prev - x_bar_cur, 2) + math.pow(y_bar_prev - y_bar_cur, 2))
        rot2 = theta_bar_cur - theta_bar_prev - rot1

        rot1_sample = rot1 - np.random.normal(alpha1*rot1 + alpha2*trans)
        trans_sample = trans - np.random.normal(alpha3*trans + alpha4*(rot1 + rot2))
        rot2_sample = rot2 - np.random.normal(alpha1*rot2 + alpha2*trans)

        new_x = old_x + trans_sample*math.cos(old_theta + rot1_sample)
        new_y = old_y + trans_sample*math.sin(old_theta + rot1_sample)
        new_theta = old_theta + rot1_sample + rot2_sample
        
        # update particles with new x,y,theta
        particle.x = new_x
        particle.y = new_y
        particle.h = new_theta

    return particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    measured_particles = []
    
    for particle in particles:
        q = 1
        if len(measured_marker_list) > 0 and len(particle.read_markers(grid)) > 0:
            for markerP in particle.read_markers(grid):
                for markerR in measured_marker_list:
                    x_robot = markerR[0]
                    y_robot = markerR[1]
                    m_jx_particle = markerP[0]
                    m_jy_particle = markerP[1]
                    theta_particle = markerP[2]

                    if grid.is_in(m_jx_particle, m_jy_particle) is True:
                        # calculate r
                        r_particle = math.sqrt(math.pow(m_jx_particle, 2) + math.pow(m_jy_particle, 2))
                        r_robot = math.sqrt(math.pow(x_robot, 2) + math.pow(y_robot, 2))

                        # calculate phi
                        phi_particle = math.atan2(m_jy_particle, m_jx_particle)
                        phi_robot = math.atan2(y_robot, x_robot)

                        # calculate q
                        q *= prob(r_robot - r_particle, math.pow(MARKER_TRANS_SIGMA, 2)) * prob(phi_robot - phi_particle, math.pow(MARKER_ROT_SIGMA, 2))
                        measured_particles.append(q)
                    else:
                        measured_particles.append(0)
        #else:
        #    measured_particles.append(0)
    

    while len(measured_particles) < len(particles):
        measured_particles.append(0)

    # Normalize the probabilities
    sum_q = sum(measured_particles)
    if sum_q == 0 or len(measured_particles) > len(particles):
        return Particle.create_random(PARTICLE_COUNT, grid)
    
    normalized_particles = [q / sum_q for q in measured_particles]

    '''print("sum of prob after normalized", sum(normalized_particles))
    print("particles", len(particles))
    print("measured", len(measured_particles))
    print("normalized", len(normalized_particles))'''

    sampled = np.random.choice(particles, PARTICLE_COUNT, replace=True, p=normalized_particles)
    sampled = [copy.copy(s) for s in sampled]
    return sampled # a list of object particles

def prob(a, b):
    return 1/(np.sqrt(2*math.pi*b))*(math.e**(-1/2*a**2/b))