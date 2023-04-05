# CODE REQUISITES:
# 1-At every trial, draw and follow a new circuit. It can have pre-made shapes or random ones.
#   At first I'll go for pre-made, starting from just one iteration to simplify development.
# 2- The car move "horizontally" left to right on the floor, seen 2D on the screen. Friction will be neglected.
# 3- For simplicity, the code will start being very basic. First, forgetting about PID
#    controlling, motion equations will be tested, giving static values for angular velocity.
#    Following that, it will have to follow the tracks by command. Then, PID will be introduced assuming continuous sensor
#    detection, within an interval. Finally, the sensor will be discretized as there will be 8 or 9 IR sensors at the nose.

# TO-DO:
# 1-

# DONE:
# 1-Create coordinates follow up for: Left wheel, right wheel. Center position of the wheels axis.
# 2-Prepare PID parameters.
# 3-Prepare animation and updateplot function
# 4-Create trials loop structure for simulation along different tracks


# ANIMATION
# 1- The car will be drawn from the Center of Wheel Axis or Sensor Center


# CAR (local coordinates)
#             Sensor Center
#                 (0,20)
#            |------|------|
#                   |
#                   |
#                  |||
#                 | | |
#                |  |  |
#               |   |   |
#              |    |    |
#           |       |       |
#        |          |          |
#      |------------|------------|
#   L_wheel   Center of Axis   R_wheel
#   (-10,0)       (0,0)        (10,0)




import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import numpy as np
import random

###############################################################################

# AS A USER, ONLY EDIT THIS SECTION. PID Simulation inputs.

# Initialize input values
trials=1
g=9.8 #[m/s^2] To be used in latter simulations bearing in mind the weight of the car
mass_car=0 #[kg] To be used in latter simulations bearing in mind the weight of the car
motor_nominal_rpm = 0 #[kg] To be used in latter simulations bearing in mind the weight of the car
motor_nominal_torque = 0 #[kg] To be used in latter simulations bearing in mind the weight of the car
y_car = 20 #[cm] Length of the car from center axis to sensor center
x_car = 20 #[cm] Width of the car measured from wheel to wheel

# PID gains tuning
K_p=1
K_d=1
K_i=1
###############################################################################

trials_global=trials # Value for trials stored. trials_global will stay fixed while trials will decrease inside the loop

# Defining time and increments
dt=0.02 # Real period increment given by the frequency detection for the IR sensors will be around 0.001s
t0=0  # [s]
t_end=3 # [s]
t=np.arange(t0,t_end+dt,dt) # [s]

F_g= -mass_car * g #Weight of the car[N]. Not needed at the moment

# Create empty arrays for the car kinematics and error with i times trials and j times the time by the time increments,
# a 2D array or matrix that will have this shape
# VARIABLE | t0 | t1 | t2 | t3 | t4 | ... | tn
# Trial 1  |    |    |    |    |    |
# Trial 2  |    |    |    |    |    |
# Trial 3  |    |    |    |    |    |

displ_sensor_center=np.zeros((trials,len(t))) # Error will be measured from here. Scalar value [cm].
displ_wheel_r=np.zeros((trials,len(t))) # Scalar value [cm].
displ_wheel_l=np.zeros((trials,len(t))) # Scalar value [cm].
displ_center_axis=np.zeros((trials,len(t))) # Scalar value [cm].

v_center_axis=np.zeros((trials, len(t))) # Scalar value [cm/s]
a_center_axis=np.zeros((trials, len(t))) # Scalar value [cm/s^2]

# Decomposition of scalar value displacement [cm]
center_axis_pos_i=np.zeros((trials, len(t))) # x coordinates for the tail of the car
center_axis_pos_j=np.zeros((trials, len(t))) # y coordinates for the tail of the car
sensor_center_pos_i=np.zeros((trials,len(t))) # x coordinates for the nose of the car
sensor_center_pos_j=np.zeros((trials,len(t))) # y coordinates for the nose of the car

# Same variables for local displacement of turns for the car
center_axis_pos_i_local = np.zeros((trials, len(t)))
center_axis_pos_j_local = np.zeros((trials, len(t)))
sensor_center_pos_i_local = np.zeros((trials, len(t)))
sensor_center_pos_j_local = np.zeros((trials, len(t)))

# Positions of the wheels. We'll see if it's necessary
wheel_r_pos_i=np.zeros((trials,len(t)))
wheel_r_pos_j=np.zeros((trials,len(t)))
wheel_l_pos_i=np.zeros((trials,len(t)))
wheel_l_pos_j=np.zeros((trials,len(t)))

phi=np.zeros((trials,len(t))) # Angular position with respect to the LCS or center of each turn
w_LCS=np.zeros((trials, len(t))) # Angular velocity with respect to the LCS or center of each turn
rho=np.zeros((trials,len(t))) # Center radius from LCS to center axis position



# Matrices for error funtion, error derivative and error integral
# Error
e=np.zeros((trials,len(t)))
#Error decomposition
e_i=np.zeros((trials,len(t)))
e_j=np.zeros((trials,len(t)))
# Derivative
e_dot=np.zeros((trials,len(t)))
# Derivative decomposition
e_dot_i=np.zeros((trials,len(t)))
e_dot_j=np.zeros((trials,len(t)))
# Integral
e_int=np.zeros((trials,len(t)))
#Integral decomposition
e_int_i=np.zeros((trials,len(t)))
e_int_j=np.zeros((trials,len(t)))



# Initial position for the car
init_center_axis_pos_i= 30
init_center_axis_pos_j =30
init_sensor_center_pos_i = 50
init_sensor_center_pos_j = 30

init_vel_track=0 # Scalar value of the speed, initially
init_phi = 0 # Initial angle with respect to the LCS.
init_vel_angular = 0
init_rho = 0

animation_window=150 # Used for determining the dimensions of the animation window.

trials_magn=trials # Trials stored once again for loop usage
history=np.ones(trials)


# NOTE: Quick loop solution, try to vectorize


while(trials>0): # Determines how many circuits from the circuit paths array will be run
    times=trials_magn-trials # Row of the kinematics matrices. Starts by 0.

    # Implement PID for the train position
    for i in range(0,len(t)):

        # Insert the initial values into the beginning of the predefined arrays.
        if i==0:
            center_axis_pos_i[times][0]=init_center_axis_pos_i
            center_axis_pos_j[times][0]=init_center_axis_pos_j
            sensor_center_pos_i[times][0]=init_sensor_center_pos_i
            sensor_center_pos_j[times][0]=init_sensor_center_pos_j
            phi[times][0] = init_phi
            v_center_axis[times][0]=init_vel_track
            w_LCS[times][0]=init_vel_angular
            rho[times][0]=init_rho
            gamma = 0



        #AT THE MOMENT TRYING CAR'S CLOCKWISE TURN, TO COMPLETE 180º IN 3 SECONDS
        if 0<i<3000:
            w_LCS[times][i] = np.pi/3 #[rad]
            rho[times][i] = 30 # [cm]


            phi[times][i] = w_LCS[times][i] * dt # Relative step rotated angle with respect to LCS in dt [rad]
            gamma = phi[times][i] + np.arctan(y_car/rho[times][i])


       # W=K_p*e[times][i-1]+K_d*e_dot[times][i-1]+K_i*e_int[times][i-1]


        center_axis_pos_i_local[times][i] = - (rho[times][i] * np.cos(phi[times][i])) #Always x negative for LCS in clockwise turn
        center_axis_pos_j_local[times][i] = rho[times][i]*np.sin(phi[times][i]) # Always x positive for LCS in clockwise turn

         # Angle between sensors center of nose and LCS. No need to store it in array
        hyp = (y_car**2 + rho[times][i]**2)**0.5 # Distance between sensors and center of LCS. No need to store it in array
        sensor_center_pos_i_local[times][i] = - hyp * np.cos(gamma)
        sensor_center_pos_j_local[times][i] = hyp * np.sin(gamma)

        # Obtaining global coordinates by rotation matrix of the LCS
        #   The LCS is always 90º rotated with respect to the car longitudinal axis
        #   reminder: PAY ATTENTION TO ANGLE SIGN, MIGHT BE INVERSED. ANTI-CLOCKWISE IS POSITIVE
        LCS_angle = np.arctan((sensor_center_pos_j[times][i-1]-center_axis_pos_j[times][i-1])/(sensor_center_pos_i[times][i-1]-center_axis_pos_i[times][i-1])) + np.pi/2 # [rad]
        LCS_pos_i = center_axis_pos_i[times][i-1] + np.cos(LCS_angle)*rho[times][i]
        LCS_pos_j = center_axis_pos_j[times][i-1] + np.sin(LCS_angle)*rho[times][i]

        #   Global coordinates
        center_axis_pos_i[times][i] = (center_axis_pos_i_local[times][i]+LCS_pos_i) * np.cos(LCS_angle) - (center_axis_pos_j_local[times][i]+LCS_pos_j)*np.sin(LCS_angle)
        center_axis_pos_j[times][i] = (center_axis_pos_i_local[times][i]+LCS_pos_i) * np.sin(LCS_angle) + (center_axis_pos_j_local[times][i]+LCS_pos_j)*np.cos(LCS_angle)
        sensor_center_pos_i[times][i] = (sensor_center_pos_i_local[times][i]+LCS_pos_i) * np.cos(LCS_angle) - (sensor_center_pos_j_local[times][i]+LCS_pos_j)*np.sin(LCS_angle)
        sensor_center_pos_j[times][i] = (sensor_center_pos_i_local[times][i]+LCS_pos_i) * np.sin(LCS_angle) + (sensor_center_pos_j_local[times][i]+LCS_pos_j)*np.cos(LCS_angle)

        displ_center_axis[times][i] =  (center_axis_pos_i[times][i]**2 + center_axis_pos_j[times][i]**2)**0.5 # Scalar of distance with respect to (0,0) [cm]

        # Tangential velocity

        #v_center_axis_pos_i TO BE CALCULATED AS A VECTOR WITH MODULE W*RHO, DIRECTION CAR'S LONGITUDINAL AXIS (NORMAL TO RHO)
        #v_center_axis_pos_j
        v_center_axis = (v_center_axis_pos_i**2+v_center_axis_pos_j**2)**0.5

    trials=trials-1



############################## ANIMATION #################################
len_t = len(t)
frame_amount = len_t*trials_global
frame_amount = 1
def update_plot(num):
# Num at each iteration will be equal to frame_amount

# Draw the car
#[[posx_left,posx_right],[posy_left,posy_right]]
#se suma y se resta t_i del coche
    pos_i_izq = center_axis_pos_i[int(num / len_t)][num - int(num / len_t) * len_t] - 20
    pos_i_dcha = center_axis_pos_i[int(num / len_t)][num - int(num / len_t) * len_t]
    pos_j_izq = center_axis_pos_j[int(num / len_t)][num - int(num / len_t) * len_t]
    pos_j_dcha = center_axis_pos_j[int(num / len_t)][num - int(num / len_t) * len_t]

    car.set_data([pos_i_izq, pos_i_dcha], [pos_j_izq, pos_j_dcha])

    displ_center_axis_f.set_data(t[0:(num - int(num / len_t) * len_t)],
        displ_center_axis[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    v_center_axis_f.set_data(t[0:(num - int(num / len_t) * len_t)],
        v_center_axis[int(num / len_t)][0:(num - int(num / len_t) * len_t)])


    return displ_center_axis_f, v_center_axis_f

fig=plt.figure(figsize=(16,9),dpi=120,facecolor=(0.8,0.8,0.8))
gs=gridspec.GridSpec(4,3)

# WINDOW
# Create main window
ax_main=fig.add_subplot(gs[0:3,0:2],facecolor=(0.9,0.9,0.9))
plt.xlim(0, animation_window)
plt.ylim(0,120)
plt.xticks(np.arange(0, animation_window + 1, 10)) # intervals for x ticks
plt.yticks(np.arange(0,120+1,10)) # intervals for y ticks
plt.grid(True)

# Title
title=ax_main.text(0, 122, 'Prueba PID Circuito', size=12)

# Race track test on screen
circuit_x = np.array([35, 30, 50, 70, 90, 110, 120, 120, 100, 35])
circuit_y = np.array([30, 30, 50, 50, 80, 90, 80, 40, 30, 30])
circuit_linea=ax_main.plot(circuit_x, circuit_y, 'k', linewidth=6)


car,=ax_main.plot([], [], 'b', linewidth=14)


# PLOT WINDOWS
#upper right window
ax1v=fig.add_subplot(gs[0,2],facecolor=(0.9,0.9,0.9))
displ_center_axis_f,=ax1v.plot([], [], '-b', linewidth=2, label='displacement [cm]') #
# to unpack the tupple resulting from the plot functions
plt.xlim(t0,t_end)
plt.ylim(np.min(displ_center_axis)-abs(np.min(displ_center_axis))*0.1,np.max(displ_center_axis)+abs(np.max(displ_center_axis))*0.1)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

# center right window
ax2v=fig.add_subplot(gs[1,2],facecolor=(0.9,0.9,0.9))
v_center_axis_f,=ax2v.plot([], [], '-b', linewidth=2, label='velocity [cm/s]')
plt.xlim(t0,t_end)
plt.ylim(np.min(v_center_axis) - abs(np.min(v_center_axis)) * 0.1, np.max(v_center_axis) + abs(np.max(v_center_axis)) * 0.1)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

# To complete with the other spaces left of the grid for acceleration or angular velocity, and error functions

# Frame_amount is the length of the animation, defined above the function update_plot
# interval is the period of miliseconds, ideally should be equal to dt.
# blit makes the animation smoother
pid_ani=animation.FuncAnimation(fig,update_plot,
    frames=frame_amount,interval=20,repeat=False,blit=True)
plt.show()

##################################
