"""
Simulation of SpotMicroAI and it's Kinematics 

"""
import time
import simulation.spotmicroai as spotmicro
import os

urdfPath = os.path.dirname(__file__) + "/robot_model/spot.xml"
print(urdfPath)
robot = spotmicro.Robot(urdf=urdfPath)

while True:
    robot.step()
    time.sleep(1.0 / 240.0)

p.disconnect()
