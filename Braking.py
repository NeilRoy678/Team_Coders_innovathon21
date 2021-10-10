"""my_controller controller."""


from controller import Robot, DistanceSensor, Motor,Brake
from vehicle import Driver
import math
# create the Robot instance.
if __name__ == "__main__":
    driver = Driver()
    driver.setCruisingSpeed(20)

    ps = []
       # get the time step of the current world.
    timestep = 64
    psNames = [
                'distance sensor front', 'distance sensor front right 0', 'distance sensor front right 1', 'distance sensor front right 2',
                'distance sensor front left 0', 'distance sensor front left 1', 'distance sensor front left 2'
              ]

    for i in range(7):
        ps.append(driver.getDevice(psNames[i]))
        ps[i].enable(timestep)
        #print(ps[i])
    max_speed= 60
    
    while driver.step() != -1:
        psValues = []
        for i in range(7):
            psValues.append(ps[i].getValue())

    # detect obstacles
        right_obstacle = psValues[0] <17.0 or psValues[1] > 80.0 or psValues[2] > 80.0
        left_obstacle = psValues[4] > 80.0 or psValues[5] > 80.0 or psValues[6] > 80.0
        #Performs Braking When obstacle is detected in front 
        if psValues[0]<18:
            driver.setBrakeIntensity(1)

            driver.setCruisingSpeed(0)
            
