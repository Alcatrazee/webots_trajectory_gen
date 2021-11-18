from controller import Robot,Supervisor,Display
import threading
import time
import numpy as np
from minmun_snap import trajectory_generation
from FK_solver import FK_solver
import math

class kr6_900(Robot):
    def init(self):

        self.TIME_STEP= int(Supervisor.getBasicTimeStep(self))
        initial_position = [-106.930000,-125.420000,-88.800000,87.410000,-69.910000,114.640000]
        for i in range(len(initial_position)):
            initial_position[i] = math.radians(initial_position[i])
        self.angle_sensor = []
        self.motors = []
        name_sensor = []
        name_motor = []
        self.joint_angle = [0,0,0,0,0,0]
        self.des_joint_angle = [0,-1.57,1.57,0,1.57,0]
        self.threadLock = threading.Lock()



        for i in range(6):
            name_sensor.append('joint_a'+str(i+1)+'_sensor')
            name_motor.append('joint_a'+str(i+1))

        for i in range(6):
            self.angle_sensor.append(self.getDevice(name_sensor[i]))
            self.angle_sensor[i].enable(self.TIME_STEP)
            self.motors.append(self.getDevice(name_motor[i]))
        
        for i in range(6):
            self.motors[i].setPosition(initial_position[i])
        w_mat,q_mat,gst_0 = self.get_param()
        self.fk = FK_solver(w_mat,q_mat,gst_0)

        self.run()

    def get_param(self):
        w_mat = np.array([[0,0,-1],
                        [0,1,0],
                        [0,1,0],
                        [-1,0,0],
                        [0,1,0],
                        [-1,0,0]]).T
        q_mat = np.array([[0,0,400],
                        [25,0,400],
                        [480,0,400],
                        [900,0,435],
                        [900,0 ,435],
                        [900, 0, 435]]).T
        gst_0 = np.array([[0,0,1,80+420+480],
                        [0,1,0,0],
                        [-1,0,0,435],
                        [0,0,0,1]])
        return w_mat,q_mat,gst_0
        

    def run(self):
        self.thread_creation()
        while self.step(self.TIME_STEP)!=-1:

            time.sleep(0.001)

    def get_joint_angle(self):
        for i in range(6):
            self.joint_angle[i] = self.angle_sensor[i].getValue()
        

        

    def thread_creation(self):
        thread_sensor = threading.Thread(name='sensor',target=self.read_joint_angle)
        # thread_control = threading.Thread(name='control',target=self.control_joint_angle)

        thread_sensor.start()
        # thread_control.start()

    def read_joint_angle(self):
        print('sensor threading in')
        timer = 0
        while True: 
            self.threadLock.acquire()
            self.get_joint_angle()
            if timer==100:
                print(self.fk.compute_fk(np.array(self.joint_angle))[0:3,3])
                timer = 0
            self.threadLock.release()
            timer+=1
            time.sleep(0.001)
        print('sensor program exited.')

    def control_joint_angle(self):
        steps_per_sec = 1000
        print('control threading in')
        switch = False
        initial_state = np.array([-1.57,0,0,0])
        end_state = np.array([1.57,0,0,0])
        T = 2
        steps = 2*steps_per_sec
        t,positive_path = trajectory_generation(initial_state,end_state,T,steps)
        t,negative_path = trajectory_generation(end_state,initial_state,T,steps)
        k = 0
        while True:
            self.threadLock.acquire()
            if k==steps:
                k = 0
                switch = ~switch
            if switch:
                self.des_joint_angle[0] = positive_path[k]
                self.des_joint_angle[2] = positive_path[k]
                self.des_joint_angle[4] = positive_path[k]
            else:
                self.des_joint_angle[0] = negative_path[k]
                self.des_joint_angle[2] = negative_path[k]
                self.des_joint_angle[4] = negative_path[k]
            k+=1

            self.set_joint_angle(self.des_joint_angle)
            # print('controller.')
            self.threadLock.release()
            # while True:
                
            time.sleep(0.001)
        print('control program exited.')

    def set_joint_angle(self,des_angle):
        for i in range(6):
                self.motors[i].setPosition(des_angle[i])




def main():
    robot = kr6_900()
    robot.init()
    
if __name__ == "__main__":

    main()