import math

'''

fr = repulsive_force as function of time
fa = attractor_force as function of time
w = noise
dt = d_target
do = d_obstacle 

psi = heading_direction

Psi1 = psi_target 
Psi2 = psi_obstacle 
Psi3 = psi_final 

sb = sigma_obstacle = apparent 2r
omega_t = psi_dot_t = first time derivative

'''
pi = math.pi 
exp = math.exp
atan2 = math.atan2
#psi = 
#Psi1 = 
#Psi2 = 
#Psi3 = 

#dt = 
#do = 
#sb = 



b1 = 6
#b1 = 8
b2 = 18
#b2 = 15

##
z = pi/6

sig0 = dist[0]* atan2(30*pi/180)
sig1 = dist[1]* atan2(30*pi/180)

lamb1 = b1*exp(-(dist[0]-r0)/b2)
lamb2 = b1*exp(-(dist[1]-r0)/b2)

fr1 = -(lamb1*z*exp(-(z**2)/(sig0)**2))
fr2 = -(lamb2*z*exp(-(z**2)/(sig0)**2))

fr_final = fr1 + fr2


# Attractor Behaviors

#fr1 = -sin(psi-Psi1)
#fr3 = -exp(-dt)*sin(psi-Psi3)
#frf = fr1+fr3


# Repulsive Behaviors
#fr2 = exp(((-psi-Psi2)^2)/(2(sb^2)))*(psi-Psi2)*exp(-do/b2)