
import numpy as np
from scipy.linalg import block_diag
import matplotlib.pyplot as plt


class CameraControlKF:
    def __init__(self, initial_state, process_noise_std, measurement_noise_std): #we can calculate these noises from the simulated measurments we received in .csv
        # Initial state and covariance
        self.state = initial_state
        self.true_state= initial_state
        self.P = np.eye(3)  #covariance for a start
        
        # motion and measurement noise covariance matrices
        self.Q = np.diag(process_noise_std**2)
        self.R = np.diag(measurement_noise_std**2)
        
        
        #Fand H are state and measurement matrices of Kalman
        self.F = np.eye(3) 
        self.H = np.eye(3)
        
        # control gain
        self.Kp = 0.5
    
    def predict(self, control_input):
        
        self.state = self.F @ self.state + control_input
        self.true_state = self.F @ self.true_state + control_input
        self.P = (self.F @self.P@ self.F.T) + self.Q
    
    def correction(self, measurement):
        
        y = measurement - (self.H @ self.state) #innovation
        # Kalman gain
        S = (self.H @self.P @ self.H.T) + self.R
        K = self.P@ self.H.T @ np.linalg.inv(S)
        
        # State update
        
        self.state = self.state + (K@y)
        I=np.eye(self.P.shape[0])
        self.P = I - (K  @ self.H)@ self.P
    
    def control_input(self, target_state):
        
        control = self.Kp * (target_state - self.state)
        return control
    
    def true_control_input(self, target_state):
        
        control = self.Kp * (target_state - self.true_state)
        return control
    
    #def apply_control(self, control):
        # Apply control input to the state (this simulates the movement towards the target)
        
    #    self.state =self.state+ np.round(control,1)

    def movement(self, control):

        if (control[0]>0):
            c=np.round(control[0],1)

            print("Move", c, "units in positive X axis direction (right)")  

        if (control[0] <0):

            c=np.round(control[0],1)

            print("Move", c, "units in negative x axis direction (left)")

        if (control[1]>0):
            c=np.round(control[1],1)

            print("Move", c, "units in positive y axis direction (forward)")  

        if (control[1] <0):

            c=np.round(control[1],1)

            print("Move", c, "units in negative y axis direction (backwards)")

        if (control[2]>0):
            c=np.round(control[2],1)

            print("Move", c, "units in positive z direction (upwards)")  

        if (control[2] <0):

            c=np.round(control[2],1)

            print("Move", c, "units in negative z direction (downwards)")

    def get_measurement(self):
        # Prompt the user for a list of numbers separated by spaces
        
        while True:
            
            user_input = input("Enter Here: ")      

        # Convert the input to a list of integers
        
            try:
                numbers = [float(item) for item in user_input.split()]
                #print("List of numbers entered:", numbers)
                measurement=np.array([numbers[0], numbers[1], numbers[2]])
                break

            except ValueError:
                print("Invalid input! Please enter valid numbers separated by spaces.")
                print("--------------------------------------------------------------")
                


        return measurement   

    
    def get_state(self):
        return self.state
    
    def get_true_state(self):
        return self.true_state








# Initialization
print("***=============================================================================***")
print("***POSE PERFECT: Real time Enhanced Camera Re-Positioning for Desired Pose***")
print("***=============================================================================***")
print("                                                                                   ")
print("***User please follow the instructions below:***")
print("                                                                                   ")
print("STEP 1. Please Enter below the x,y,z coordinates of Target Camera Position you wish to achieve for perfect pose,(in valid float numbers seperated by spaces): ")

print("STEP 2. Now move the camera to any displaced position and enter the camera sensor measurment: x, y and z position coordinates when prompted below (in valid float numbers seperated by spaces)")

print("STEP 3. Adjust the Camera according to instructions and then enter the new current valid camera sensor position measurments x,y z when prompted below until perfect pose position is achieved")
print("                                                                                                   ")

#####
while True:
           
            user_input = input("Enter valid Target Camera position now according to STEP 1: ")
            print("                                                                                   ")    

        # Convert the input to a list of integers
        
            try:
                numbers = [float(item) for item in user_input.split()]
                #print("List of numbers entered:", numbers)
                tmeasurement=np.array([numbers[0], numbers[1], numbers[2]])
                break

            except ValueError:
                print("Invalid input! Please enter valid numbers separated by spaces.")
                print("--------------------------------------------------------------")

target_state = tmeasurement
initial_state = np.array([0, 0, 0])  # New Displaced Position

process_noise_std = np.array([1.7, 0.1, 1.7])  # In form of std.devs
measurement_noise_std = np.array([1.3, 1.8, 0.7])  # In form of Std. devs

camkf = CameraControlKF(initial_state, process_noise_std, measurement_noise_std)



states = []
measurements = []
controls=[]
true_states = []
true_controls=[]




        

i=0
measurement= np.array([0, 0, 0])
control_signal=0
# Control loop
while not np.allclose(camkf.get_state(), target_state, atol=2.0): #keep comparing the 2 matrices if within tolerance
    # Acquire measurment here: Simulation or real data 
    
    # Simulate measurement (replace with actual measurement data)
    #measurement = camkf.get_state() + np.random.normal(0, measurement_noise_std)
    if (i==0):
        print("Please Now Enter the Displaced Camera position measurement x,y,z separated by spaces according to STEP 2") 
        measurement=camkf.get_measurement()
        measurements.append(measurement)
        
        camkf.state=measurement
        camkf.true_state=measurement
        
        states.append(camkf.get_state())
        true_states.append(camkf.get_true_state())
        
        control_signal=camkf.control_input(target_state)
        controls.append(np.linalg.norm(control_signal))

        true_control_signal=camkf.true_control_input(target_state)
        true_controls.append(np.linalg.norm(true_control_signal))


        i=i+1
        #control_signal=(target_state-measurement)*camkf.Kp
    else:
        control_signal=camkf.control_input(target_state)
        controls.append(np.linalg.norm(control_signal))

        true_control_signal=camkf.true_control_input(target_state)
        true_controls.append(np.linalg.norm(true_control_signal))

        camkf.predict(control_signal)
        camkf.true_state=camkf.true_state+true_control_signal

        camkf.movement(control_signal)
        
        print("                                                                                                                 ")
        print("Please Now Enter the Current Displaced Camera position measurement x,y,z separated by spaces according to STEP 3") 
        
        measurement=camkf.get_measurement()
        measurements.append(measurement)
        camkf.correction(measurement)
            # Current state stimation/calculation with control
        print("                                                                                                                 ")
        print("Estimated current Camera state (x,y,z) from Kalman Filter:", camkf.get_state())
        print("                                                                                                                 ")

        states.append(camkf.get_state())
        true_states.append(camkf.get_true_state())

        i=i+1

        print("==================================================================================================================")



print("Target Camera Position Achieved--Program Exiting")


##############################################
####


# Convert lists to numpy arrays for plotting
states = np.array(states)
true_states = np.array(true_states)
measurements = np.array(measurements)

controls = np.array(controls)
true_controls = np.array(true_controls)
num_steps=i
steps = np.arange(num_steps)

################Testing/Printing arrays used iin plotting for verification

#print("controls shape", controls.shape)
#print("states shape", states.shape)
#print("true states shape", true_states.shape)
#print("measurments shape", measurements.shape)
#print("true controls shape", true_controls.shape)

#print("controls", controls)
#print("true controls", true_controls)
#print("measurements", measurements)
#print("true states", true_states)
#print("states", states)
#print("num_steps", num_steps)
################################################################

# Plotting true states vs estimated states
fig1, axs = plt.subplots(3, 1, figsize=(10, 15))  # 3 rows for x, y, z components

# X Component Plot
axs[0].plot(steps, states[:, 0], 'rx', label='Kalman Filter Estimated X')
axs[0].plot(steps, true_states[:, 0], 'ro', label='Ground Truth X')
axs[0].set_title('X Component-Ground Truth vs Estimated Kalman state - Experiment Trial ')
axs[0].set_xlabel('No of Steps')
axs[0].set_ylabel('X Value')
axs[0].legend()
axs[0].grid(True)

# Y Component Plot
axs[1].plot(steps, states[:, 1], 'gx', label='Kalman Filter Estimated Y')
axs[1].plot(steps, true_states[:, 1], 'go', label='Ground Truth Y')
axs[1].set_title('Y Component-Ground Truth vs Estimated Kalman state - Experiment Trial')
axs[1].set_xlabel('No of Steps')
axs[1].set_ylabel('Y Value')
axs[1].legend()
axs[1].grid(True)

# Z Component Plot
axs[2].plot(steps, states[:, 2], 'bx', label='Kalman Filter Estimated Z')
axs[2].plot(steps, true_states[:, 2], 'bo', label='Ground Truth Z')
axs[2].set_title('Z Component-Ground Truth vs Estimated Kalman state - Experiment Trial')
axs[2].set_xlabel('No of Steps')
axs[2].set_ylabel('Z Value')
axs[2].legend()
axs[2].grid(True)

# Display the plots
plt.subplots_adjust(left=0.1, right=0.9, top=0.8, bottom=0.2, hspace=0.6, wspace=0.6)
#plt.tight_layout()
plt.show()

#########################Plotting true state vs Estimated combined
plt.figure(figsize=(10, 6))

# Plotting the X component of states and measurements
plt.plot(steps, states[:, 0], 'rx', label='Estimated State X', alpha=0.9)
plt.plot(steps, true_states[:, 0], 'ro', label='Ground Truth X', alpha=0.9)

# Plotting the Y component of states and measurements
plt.plot(steps, states[:, 1], 'gx', label='Estimated State Y', alpha=0.9)  # Green dashed line for Y states
plt.plot(steps, true_states[:, 1], 'go', label='Ground Truth Y', alpha=0.9)


# Plotting the Z component of states and measurements
plt.plot(steps, states[:, 2], 'bx', label='Estimated State Z', alpha=0.9)  # Blue dashed line for Z states
plt.plot(steps, true_states[:, 2], 'bo', label='Ground Truth Z', alpha=0.9)


plt.title('Ground Truth vs Estimated Kalman state - Experiment Trial')
plt.xlabel('Steps')
plt.ylabel('State Value')
#plt.legend()  # Display a legend to identify each line
plt.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
plt.grid(True)
plt.tight_layout() 
plt.show()


####################
##Bar Graph True state control input vs Estimated control input


# Set the width for the bars
bar_width = 0.35

# Creating a new figure for the bar graph
plt.figure(figsize=(10, 6))

# Creating a bar graph for controls
# Shift each bar to the left by half the width of the bar
plt.bar(steps[1:] - bar_width/2, controls[1:], width=bar_width, color='blue', alpha=0.7, label='Control')

# Creating a bar graph for true_controls
# Shift each bar to the right by half the width of the bar
plt.bar(steps[1:] + bar_width/2, true_controls[1:], width=bar_width, color='green', alpha=0.7, label='Ground Truth Control')

# Adding titles and labels
plt.title('Control Input vs Ground Truth Control Input at Each Step')
plt.xlabel('Steps')
plt.ylabel('Control Magnitude')

# Adding a legend to distinguish between the datasets
plt.legend()
plt.show()
################

