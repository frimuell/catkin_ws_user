#
# PID Controller
#
import rospy
import sys

import time
import numpy as np
from std_msgs.msg import Int16, UInt8, Float32
import signal
import matplotlib.pyplot as plt

class pid_vel:
    
    def __init__(self, meters_per_second):
        self.aimed_velocity = meters_per_second # input value 
        self_aimed_tps = meters_per_second / ((5/9)/100) # 5/9 = 1 tick = 5/9cm
        
        self.current_velocity = 0 # in Int16 values for topic /manual_control/speed Int16
        self.current_tps = 0
        
        self.ticks = 0 # 1m = ca 180 ticks 
        

        self.time_sum = 0 # runtime in seconds
    
        self.ticks_sub = rospy.Subscriber("/ticks", UInt8, self.callback_ticks, queue_size=20)
        self.speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=20)
        
        # for plotting
        self.time_vals = [0]
        self.velo_vals = [0]
        
    # calculate current speed at each tick 
    def callback_ticks(self, data):
        self.ticks += int(data.data)

    # use the counted ticks of the last second to compute velocity and execute PID control
    def pid_controller(self):
        print("test")
                
        K_P = 10 # to be adjusted by trial and error
        K_I = 0.001 # to be adjusted by trial and error
        K_D = 2.5 # to be adjusted by trial and error
        
        time_sum = 0 # runtime in seconds        
        error_sum = 0
        
        error_prev = self.aimed_tps - self.current_tps
        begin_ticks = self.ticks
        ticks_prev = beging_tricks
        
        while True:
            ticks = begin_tricks
            self.current_tps  = ticks - ticks_prev
            
            error = self.aimed_tps - self.current_tps
            error_sum += error * time_sum
            deriv = error - error_prev # discrete derivative (delta t = 1 second)
            
            speed_increase = K_P * error  + K_I * error_sum + K_D * deriv # P + I + D
            self.current_velocity += speed_increase
            
            self.speed_pub.publish(np.int16(self.current_velocity))
            print(self.current_velocity, speed_increase)
            
            
            self.time_vals.append(time_sum)
            
            error_prev = error
            ticks_prev = ticks
            time_sum += 1
            
            # for plotting
            self.time_vals.append(time_sum)
            self.velo_vals.append(self.current_velocity)
            plt.plot(time_vals,velo_vals)
            plt.title("Velocity over Time, using PID-Control")
            plt.xlabel("time")
            plt.ylabel("velocity")
            time.sleep(1 - (time.time() % 1)) 
        
def main(args):
    rospy.init_node('pid_velocity', anonymous=True)
    pidvel = pid_vel(0.3)
    pidvel.pid_controller()

    def rescue(*_):
        print(pid_vel.output)
        rospy.signal_shutdown("")

    signal.signal(signal.SIGINT, rescue)

    rospy.spin()
    

if __name__ == '__main__':
    main(sys.argv)






