import pygame
import sys
import time
import math
import argparse
import numpy as np
from RobotLib.FrontEnd import *
from RobotLib.IO import *
from Robot import Robot
from OccupancyMap import OccupancyMap
from RobotLib.Math import *

class MyFrontEnd(FrontEnd):
    def __init__(self,sparki,omap_path):
        self.omap = OccupancyMap(omap_path)
        FrontEnd.__init__(self,self.omap.width,self.omap.height)
        self.sparki = sparki
        self.robot = Robot()
        self.distance = 0
        self.cycle = 0
        self.wait_time = 0

        # center robot
        self.robot.x = self.omap.width*0.5
        self.robot.y = self.omap.height*0.5
        
        # zero out robot velocities
        self.robot.lin_vel = 0
        self.robot.ang_vel = 0
        
    def mouseup(self,x,y,button):
        pass

    def draw(self,surface):
        # draw occupancy map
        self.omap.draw(surface)

        # draw robot
        self.robot.draw(surface)
    
    def update(self,time_delta):
        # get sonar distance
        self.robot.sonar_distance = self.sparki.dist
    
        #spin cycle, rotate ccw by 1 degree
        if self.cycle == 0:
            if self.robot.sonar_distance < 50: #distance in cm?
               
                self.robot.lin_vel = 0
                self.robot.ang_vel = math.pi/90
                
                #rotate a little more due to robot sonar seeing object before it is infront
                left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()
                self.sparki.send_command(left_speed, left_dir, right_speed, right_dir, 0, 1)
                print("Sonar Distance: ", self.robot.sonar_distance)

                #how to calculate this??
                print("wait 17")
                time.sleep(17)
                self.sparki.send_command(0,0,0,0,0,0)
                
                #move to next cycle
                self.cycle = 1
            else:
                #spin
                print("Sonar Distance: ", self.robot.sonar_distance)
                self.robot.lin_vel = 0
                self.robot.ang_vel = math.pi/90 #2 degree per sec?

                #find speed to send
                left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()
                self.sparki.send_command(left_speed, left_dir, right_speed, right_dir)
                
                #wait .5sec
                time.sleep(.5)

                #stop
                self.sparki.send_command(0,0,0,0,0,0)

        #move cycle, find object, remeber object distance, move to it, grip it, 
        elif self.cycle == 1:
            #save distance, set velocity and find wait time
            self.distance = self.robot.sonar_distance
            self.robot.lin_vel = 3.06 #80% power, 90% is 3.44
            self.robot.ang_vel = 0
            self.wait_time = (self.distance / self.robot.lin_vel) + 2 #wait time cm / (cm/s) + delay

            #send command to sparki
            left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()
            self.sparki.send_command(left_speed, left_dir, right_speed, right_dir)
            
            #wait for sparki to get there
            print("Wait Time:  ", self.wait_time)
            time.sleep(self.wait_time) #in secs

            #zero speed
            self.robot.lin_vel = 0
            left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()
            self.sparki.send_command(left_speed, left_dir, right_speed, right_dir)
            
            #close grippers, wait 4 secs stop
            self.sparki.send_command(0,0,0,0,0,2)
            print("Wait Time:  4")
            time.sleep(4)
            self.sparki.send_command(0,0,0,0,0,0)
          
            #move to next cycle
            self.cycle = 2


        #return cycle, reverse motors, check line sensors?, drop object, Wait 10 S, spin 
        #??added wait slow robot so reverse calcs off??
        elif self.cycle == 2:
            #reverse speed that we used to get there
            self.robot.lin_vel = -3.06
            self.robot.ang_vel = 0
            left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()

            #send command and wait the same amount of time
            self.sparki.send_command(left_speed, left_dir, right_speed, right_dir)
            print("Wait Time:  ", self.wait_time)
            time.sleep(self.wait_time)

            #stop
            self.sparki.send_command(0,0,0,0,0,0)
            
            #open gripper
            self.sparki.send_command(0,0,0,0,0,1)
            time.sleep(4)

            #stop gripper
            self.sparki.send_command(0,0,0,0,0,0)

            self.cycle = 0
            #reset
            print("Remove Object, Waiting 10 secs")
            time.sleep(10)
        
        else: #shouldn't get here, how did you do it
            print("Bad Programmer")

        self.robot.update(time_delta)        
        

def main():
    # parse arguments
    parser = argparse.ArgumentParser(description='template')
    parser.add_argument('--omap', type=str, default='map.png', help='path to occupancy map image file')
    parser.add_argument('--port', type=str, default='', help='port for serial communication')
    args = parser.parse_args()

    with SparkiSerial(port=args.port) as sparki:
        # make frontend
        frontend = MyFrontEnd(sparki,args.omap)
    
        # run frontend
        frontend.run()

if __name__ == '__main__':
    main()
