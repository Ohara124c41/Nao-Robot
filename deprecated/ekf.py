import numpy as np
import scipy.stats
import matplotlib.pyplot as plt

from naoqi import ALProxy

robotIP = "192.168.0.115"




from read_data import read_world, read_sensor_data
from matplotlib.patches import Ellipse

#plot preferences, interactive plotting mode
fig = plt.figure()
plt.axis([-1, 12, 0, 10])
plt.ion()
plt.show()

def DoInitState():
    robotIP ="192.168.0.115"
    nao.InitProxy(nao_ip)
    nao.InitSonar()
    nao.InitLandMark()
    nao.InitTrack()
    nao.InitPose()
    
    return "navigate"

def plot_state(mu, sigma, landmarks, map_limits):
    # Visualizes the state of the kalman filter.
    #
    # Displays the mean and standard deviation of the belief,
    # the state covariance sigma and the position of the 
    # landmarks.

    # landmark positions
    lx=[]
    ly=[]

    for i in range (len(landmarks)):
        lx.append(landmarks[i+1][0])
        ly.append(landmarks[i+1][1])

    # mean of belief as current estimate
    estimated_pose = mu

    #calculate and plot covariance ellipse
    covariance = sigma[0:2,0:2]
    eigenvals, eigenvecs = np.linalg.eig(covariance)

    #get largest eigenvalue and eigenvector
    max_ind = np.argmax(eigenvals)
    max_eigvec = eigenvecs[:,max_ind]
    max_eigval = eigenvals[max_ind]

    #get smallest eigenvalue and eigenvector
    min_ind = 0
    if max_ind == 0:
        min_ind = 1

    min_eigvec = eigenvecs[:,min_ind]
    min_eigval = eigenvals[min_ind]

    #chi-square value for sigma confidence interval
    chisquare_scale = 2.2789  

    #calculate width and height of confidence ellipse
    width = 2 * np.sqrt(chisquare_scale*max_eigval)
    height = 2 * np.sqrt(chisquare_scale*min_eigval)
    angle = np.arctan2(max_eigvec[1],max_eigvec[0])

    #generate covariance ellipse
    ell = Ellipse(xy=[estimated_pose[0],estimated_pose[1]], width=width, height=height, angle=angle/np.pi*180)
    ell.set_alpha(0.25)

    # plot filter state and covariance
    plt.clf()
    plt.gca().add_artist(ell)
    plt.plot(lx, ly, 'bo',markersize=10)
    plt.quiver(estimated_pose[0], estimated_pose[1], np.cos(estimated_pose[2]), np.sin(estimated_pose[2]), angles='xy',scale_units='xy')
    plt.axis(map_limits)
    
    plt.pause(0.01)

def prediction_step(odometry, mu, sigma):
    # Updates the belief, i.e., mu and sigma, according to the motion 
    # model
    # 
    # mu: 3x1 vector representing the mean (x,y,theta) of the 
    #     belief distribution
    # sigma: 3x3 covariance matrix of belief distribution 
    
    x = mu[0]
    y = mu[1]
    theta = mu[2]

    delta_rot1 = odometry['r1']
    delta_trans = odometry['t']
    delta_rot2 = odometry['r2']

    '''your code here'''
    '''***DONE***'''
    #motion noise
    Q = np.array([[0.2, 0.0, 0.0],\
        [0.0, 0.2, 0.0],\
        [0.0, 0.0, 0.02]])
    
    
    #noise free motion
    x_new = x + delta_trans * np.cos(theta + delta_rot1)
    y_new = y + delta_trans * np.sin(theta + delta_rot1)
    theta_new = theta + delta_rot1 + delta_rot2
    
    
    #Jacobian of g with respect to the state
    G = np.array([[1.0, 0.0, -delta_trans * np.sin(theta + delta_rot1)],\
        [0.0, 1.0, delta_trans * np.cos(theta + delta_rot1)],\
        [0.0, 0.0, 1.0]])
    
    
    #new mu and sigma
    mu = [x_new, y_new, theta_new]
    sigma = np.dot(np.dot(G,sigma),np.transpose(G)) + Q
    

    return mu, sigma



def correction_step(sensor_data, mu, sigma, landmarks):
    # updates the belief, i.e., mu and sigma, according to the
    # sensor model
    # 
    # The employed sensor model is range-only
    #
    # mu: 3x1 vector representing the mean (x,y,theta) of the 
    #     belief distribution
    # sigma: 3x3 covariance matrix of belief distribution 

    x = mu[0]
    y = mu[1]
    theta = mu[2]

    #measured landmark ids and ranges
    ids = sensor_data['id']
    ranges = sensor_data['range']

    '''your code here'''
    '''***DONE***'''
# Compute the expected range measurements for each landmark.
    # This corresponds to the function h
    H = []
    Z = []
    expected_ranges = []


    for i in range(len(ids)):
        lm_id = ids[i]
        meas_range = ranges[i]
        lx = landmarks[lm_id][0]
        ly = landmarks[lm_id][1]

        #calculate expected range measurement
        range_exp = np.sqrt( (lx - x)**2 + (ly - y)**2 )

        #compute a row of H for each measurement
        H_i = [(x - lx)/range_exp, (y - ly)/range_exp, 0]
        H.append(H_i)
        Z.append(ranges[i])
        expected_ranges.append(range_exp)

    # noise covariance for the measurements
    R = 0.5 * np.eye(len(ids))

    # Kalman gain
    K_help = np.linalg.inv(np.dot(np.dot(H,sigma),np.transpose(H)) + R)
    K = np.dot(np.dot(sigma,np.transpose(H)),K_help)

    # Kalman correction of mean and covariance
    mu = mu + np.dot(K,(np.array(Z) - np.array(expected_ranges)))
    sigma = np.dot(np.eye(len(sigma)) - np.dot(K,H),sigma)


    return mu, sigma

def main():
    # implementation of an extended Kalman filter for robot pose estimation

    print ("Reading landmark positions")
    landmarks = read_world("../data/world.dat")

    print ("Reading sensor data")
    sensor_readings = read_sensor_data("../data/sensor_data.dat")

    #initialize belief
    mu = [0.0, 0.0, 0.0]
    sigma = np.array([[1.0, 0.0, 0.0],\
                      [0.0, 1.0, 0.0],\
                      [0.0, 0.0, 1.0]])

    map_limits = [-1, 12, -1, 10]

    #run kalman filter
    for timestep in range(len(sensor_readings)/2):

        #plot the current state
        plot_state(mu, sigma, landmarks, map_limits)

        #perform prediction step
        mu, sigma = prediction_step(sensor_readings[timestep,'odometry'], mu, sigma)

        #perform correction step
        mu, sigma = correction_step(sensor_readings[timestep, 'sensor'], mu, sigma, landmarks)

    plt.show('hold')

if __name__ == "__main__":
    main()