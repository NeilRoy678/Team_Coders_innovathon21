

from vehicle import Driver
from controller import Camera 
driver = Driver()
sensorsNames = ["front","front right 0","front right 1","front right 2","front left 0","front left 1","front left 2","rear",
    "rear left","rear right","right","left"]
sensors = {}


lanePositions = [-10.6, -6.875, -3.2]
currentLane = 1
overtaking = None
maxSpeed = 35
safeOvertake = False
gps = driver.getDevice("gps")
gps.enable(10)

camera = driver.getDevice("camera")
camera.enable(10)
#camera.recognitionEnable(50)

#Regulastes speed
def speed_(speed):
    speed_.previousSpeeds.append(speed)
    if len(speed_.previousSpeeds) > 100:  
        speed_.previousSpeeds.pop(0)
    return sum(speed_.previousSpeeds) / float(len(speed_.previousSpeeds))
#Checks if vehicle is on the side takes (Right or left as parameters which indicates which sensors should it go data for)

def vehicle_side(side):
    for i in range(3):
        name = "front " + side + " " + str(i)
        if sensors[name].getValue() > 0.8 * sensors[name].getMaxValue():
            return True
    return False

#Reduces speed if it detects obstacles 
def reduce_speed(speed, side):
    minRatio = 1
    for i in range(3):
        name = "front " + overtaking + " " + str(i)
        ratio = sensors[name].getValue() / sensors[name].getMaxValue()
        if ratio < minRatio:
            minRatio = ratio
    return minRatio * speed
    
#Enables all the distance sensors  presents in the car
for name in sensorsNames:
    sensors[name] = driver.getDevice("distance sensor " + name)
    sensors[name].enable(10)


#Calculates the optimal angle based of speed (Used github repos for reference )
def apply_PID(position, targetPosition):
    P = 0.05
    I = 0.000015
    D = 25
    diff = position - targetPosition
    if apply_PID.previousDiff is None:
        apply_PID.previousDiff = diff
    if diff > 0 and apply_PID.previousDiff < 0:
        apply_PID.integral = 0
    if diff < 0 and apply_PID.previousDiff > 0:
        apply_PID.integral = 0
    apply_PID.integral += diff
    angle = P * diff + I * apply_PID.integral + D * (diff - apply_PID.previousDiff)
    apply_PID.previousDiff = diff
    return angle
    
apply_PID.integral = 0
apply_PID.previousDiff = None
speed_.previousSpeeds = []

#Main simulation happens here 
#Uses various sensors and and checks for cars in the sorrunding
#checks whether its safe for steering and sets the steering angle calulated by the PID functions 
while driver.step() != -1:
    frontDistance = sensors["front"].getValue()
    frontRange = sensors["front"].getMaxValue()
    speed = maxSpeed * frontDistance / frontRange
    if sensors["front right 0"].getValue() < 8.0 or sensors["front left 0"].getValue() < 8.0:
        speed = min(0.5 * maxSpeed, speed)
    
    if overtaking is not None:
        if overtaking == 'right' and sensors["left"].getValue() < 0.8 * sensors["left"].getMaxValue():
            overtaking = None
            currentLane -= 1
        elif overtaking == 'left' and sensors["right"].getValue() < 0.8 * sensors["right"].getMaxValue():
            overtaking = None
            currentLane += 1
        else:  
            speed2 = reduce_speed(speed, overtaking)
            if speed2 < speed:
                speed = speed2
                
    speed = speed_(speed)
    print("Speed : "+str(speed))
    driver.setCruisingSpeed(speed)
    speedDiff = driver.getCurrentSpeed() - speed
    #adjusts the braking intensity 
    if speedDiff > 0:
        driver.setBrakeIntensity(min(speedDiff / speed, 1))
    else:
        driver.setBrakeIntensity(0)
    if frontDistance < 0.8 * frontRange and overtaking is None:
        if (vehicle_side("left") and
                (not safeOvertake or sensors["rear left"].getValue() > 0.8 * sensors["rear left"].getMaxValue()) and
                sensors["left"].getValue() > 0.8 * sensors["left"].getMaxValue() and
                currentLane < 2):
            currentLane += 1
            overtaking = 'right'
            
        elif (vehicle_side("right") and
                (not safeOvertake or sensors["rear right"].getValue() > 0.8 * sensors["rear right"].getMaxValue()) and
                sensors["right"].getValue() > 0.8 * sensors["right"].getMaxValue() and
                currentLane > 0):
            currentLane -= 1
            overtaking = 'left'
  
          
    position = gps.getValues()[0]
    print("Lane Position" + str(position))
    print("Front distance "+ str(frontDistance))
    angle = max(min(apply_PID(position, lanePositions[currentLane]), 0.5), -0.5)
    driver.setSteeringAngle(angle - 0.09)
    
    if abs(position - lanePositions[currentLane]) < 1.5:  
        overtaking = None
    