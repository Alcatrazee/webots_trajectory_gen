from controller import Robot,Supervisor,Display
import threading
import time
import numpy as np
from minmun_snap import trajectory_generation
from FK_solver import FK_solver
import math
from IK import IK_solver
import operator

class kr6_900(Robot):

    def init(self):
        self.states_dict = {'idle':1,'jogging':2,'motion':3,'pause':4}
        max_min_angles_deg = np.array([[-170,170],[-190,45],[-120,156],[-185,185],[-120,120],[-350,350]])
        self.max_min_angles = np.deg2rad(max_min_angles_deg)
        self.steps_per_sec = 100
        
        self.valid_pos = False

        self.state = self.states_dict['idle']
        self.TIME_STEP= int(Supervisor.getBasicTimeStep(self))
        home_position = [0,-90,90,0,90,0]

        # motion control variables
        self.gst0 = [ [   -0.000000,    -0.000000,     1.000000,   525.000000 ],
                    [ -0.000000,     1.000000,     0.000000,     0.000000 ],
                    [-1.000000,    -0.000000,    -0.000000,   890.000000 ],
                    [ 0.000000,     0.000000,     0.000000,     1.000000] ]

        self.gst1 = [[     0.044882,    -0.002677,     0.998989,   334.390020 ],
                    [0.829955,     0.556681,    -0.035796,   501.340369 ],
                    [-0.556022,     0.830722,     0.027206,   489.513829] ,
                    [0.000000,     0.000000,     0.000000,     1.000000 ]]

        self.gst2 = [    [-0.407247,     0.223049,    -0.885663,   -63.973226] ,
                    [-0.901412,    -0.254226,     0.350464,   -49.049778 ],
                    [-0.146988,     0.941073,     0.304592,  1038.441005], 
                    [0.000000,     0.000000,     0.000000,     1.000000] ]

        self.gst_list = [self.gst2,self.gst1,self.gst0]

        self.gst = []

        self.planned = False

        for i in range(len(home_position)):
            home_position[i] = math.radians(home_position[i])

        self.angle_sensor = []
        self.motors = []
        name_sensor = []
        name_motor = []
        self.joint_angle = [0,0,0,0,0,0]
        self.des_joint_angle = [0,-1.57,1.57,0,1.57,0]
        self.threadLock = threading.Lock()

        # sensors naming
        for i in range(6):
            name_sensor.append('joint_a'+str(i+1)+'_sensor')
            name_motor.append('joint_a'+str(i+1))

        # activate sensors
        for i in range(6):
            self.angle_sensor.append(self.getDevice(name_sensor[i]))
            self.angle_sensor[i].enable(self.TIME_STEP)
            self.motors.append(self.getDevice(name_motor[i]))
        
        # for i in range(6):
        #     self.motors[i].setPosition(home_position[i])
        w_mat,q_mat,self.gst_0,self.gst_0_0 = self.get_param()
        self.fk = FK_solver(w_mat,q_mat,self.gst_0)

        self.state = self.states_dict['idle']
        
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
        gst_0 = np.array([[0,0,1,980],
                            [0,1,0,0],
                            [-1,0,0,435],
                            [0,0,0,1]])

        gst_0_0 = np.array([[0,0,1,900],
                        [0,1,0,0],
                        [-1,0,0,435],
                        [0,0,0,1]])
        return w_mat,q_mat,gst_0,gst_0_0
        
    def run(self):
        self.thread_creation()
        while self.step(self.TIME_STEP)!=-1:
            if self.state == self.states_dict['idle']:
                if self.gst_list:
                    self.gst = self.gst_list.pop()
                    self.state = self.states_dict['motion']
                
            time.sleep(0.001)

    def get_joint_angle(self):
        for i in range(6):
            self.joint_angle[i] = self.angle_sensor[i].getValue()
        
    def thread_creation(self):
        thread_sensor = threading.Thread(name='sensor',target=self.read_joint_angle)
        thread_control = threading.Thread(name='control',target=self.control_joint_angle)

        thread_sensor.start()
        thread_control.start()

    def read_joint_angle(self):
        print('sensor threading in')
        timer = 0
        while True: 
            self.get_joint_angle()
            if not math.isnan(self.joint_angle[0]):
                self.valid_pos = True
                # print(self.joint_angle)
            if self.valid_pos == True:
                self.posture = self.fk.compute_fk(np.array(self.joint_angle))
            if timer==100:
                # print(self.posture)
                timer = 0
            timer+=1
            time.sleep(1/self.steps_per_sec)
        print('sensor program exited.')

    def control_joint_angle(self):
        self.ik_solver = IK_solver()
        print('control threading in')
        
        while(1):
            self.fsm()
            
            time.sleep(1/self.steps_per_sec)
        print('control program exited.')

    def fsm(self):
        if self.state == self.states_dict['idle']:
            pass
        elif self.state == self.states_dict['jogging'] and self.valid_pos == True:
            pass
        elif self.state == self.states_dict['motion'] and self.valid_pos == True:
            if self.planned == False:
                self.T = 4
                time_start = time.time()
                # step 1. ik
                solutions = self.ik_solver.compute_IK(gst=self.gst,gst_0=self.gst_0,gst_0_0=self.gst_0_0)
                # step 2. find optimal solution
                # print(solutions)
                initial_state = self.joint_angle

                self.des_joint_angle = self.get_optimal_solution(solutions,initial_state)
                
                # step 3. trajectory planning
                end_state = self.des_joint_angle
                self.sum_steps = self.T*self.steps_per_sec
                self.path = np.zeros([self.sum_steps,6])
                for i in range(6):
                    t,self.path[:,i] = trajectory_generation(np.array([initial_state[i],0,0,0]),np.array([end_state[i],0,0,0]),self.T,self.sum_steps)
                # step 4. set planned as true
                print('time elapsed for ik:',time.time() - time_start)
                self.planned = True
                self.current_step_index = 0
            
            # step 1. set joint angle
            self.set_joint_angle(self.path[self.current_step_index,:])
            self.current_step_index += 1
            # step 2. check if it's the final step,if yes, set state
            if self.current_step_index == self.sum_steps:
                self.state = self.states_dict['idle']
                self.planned = False
        elif self.state == self.states_dict['pause'] and self.valid_pos == True:
            pass

    def set_joint_angle(self,des_angle):
        for i in range(6):
                self.motors[i].setPosition(des_angle[i])

    def get_optimal_solution(self,solutions,start_pos):
        # TODO: find optimal solution, and validate solutions
        cost_cof = [1,0.9,0.8,0.7,0.6,0.5]
        cost = []
        # remove the unreachable solutions
        index_to_remove = []
        for i in range(solutions.shape[0]):
            for j in range(6):
                if solutions[i,j]>self.max_min_angles[j,1] or solutions[i,j]<self.max_min_angles[j,0]:
                    index_to_remove.append(j)
                    break
        index_to_remove = np.array(index_to_remove)
        index_to_remove = np.flipud(index_to_remove)
        for index in index_to_remove:
            np.delete(solutions,index)

        for i in range(solutions.shape[0]):
            delta = np.abs(solutions[i,:] - start_pos)
            print(delta)
            for j in range(6):
                delta[j] = cost_cof[j]*delta[j]
            cost.append([np.sum(delta),i])
        # cost = sorted(cost,key=operator.itemgetter(0))
        print(cost)

        return solutions[0,:]  
        # return solutions[cost[0][1],:]



def main():
    robot = kr6_900()
    robot.init()
    
if __name__ == "__main__":
    main()