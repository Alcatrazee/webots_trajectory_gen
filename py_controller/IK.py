from FK_solver import FK_solver,R_matrix
import numpy as np
import math,time

class IK_solver():
    def init():
        pass

    def compute_IK(self,gst,gst_0,gst_0_0):
        # prepare output
        theta_vector = np.zeros([8,6])
        # compute theta1 two solutions
        wrist = np.dot(np.dot(gst,np.linalg.inv(gst_0)),gst_0_0)
        T_1_6 = np.dot(gst,np.linalg.inv(gst_0))
        R_1_6 = T_1_6[0:3,0:3]
        wrist_pos = wrist[0:4,3]

        theta1_0 = -math.atan2(wrist_pos[1],wrist_pos[0])
        if theta1_0>=0:
            theta1_1 = theta1_0 - math.pi
        elif theta1_0<0:
            theta1_1 = theta1_0 + math.pi

        # output of theta1 two results
        theta_vector[0:4,0] = theta1_0
        theta_vector[4:8,0] = theta1_1

        # compute theta 2 and 3
        # rotate joint 1 to obtain a relationship of joint 2 and 3 and so the links
        T1_0 = np.eye(4)
        T1_1 = np.eye(4)

        T1_0[0:3,0:3] = R_matrix(np.array([0,0,-1]),theta1_0)
        T1_1[0:3,0:3] = R_matrix(np.array([0,0,-1]),theta1_1)
        
        axis_456_in_t0 = np.array(np.dot(np.linalg.inv(T1_0),np.matrixlib.mat(wrist_pos).T))
        axis_456_in_t1 = np.array(np.dot(np.linalg.inv(T1_1),np.matrixlib.mat(wrist_pos).T))
        # to formalize a shape to solve with 2d geometry knowledge
        axis_456_in_t0[2] = axis_456_in_t0[2] - 400
        axis_456_in_t0[0] = axis_456_in_t0[0] - 25
        axis_456_in_t1[2] = axis_456_in_t1[2] - 400
        axis_456_in_t1[0] = axis_456_in_t1[0] - 25

        theta_23_t0 = self.compute_theta23(axis_456_in_t0,455,math.sqrt(35**2+420**2),math.atan(35/420))
        theta_23_t1 = self.compute_theta23(axis_456_in_t1,455,math.sqrt(35**2+420**2),math.atan(35/420))

        theta_vector[0,1:3] = theta_23_t0[0,:]
        theta_vector[1,1:3] = theta_23_t0[0,:]
        theta_vector[2,1:3] = theta_23_t0[1,:]
        theta_vector[3,1:3] = theta_23_t0[1,:]
        theta_vector[4,1:3] = theta_23_t1[0,:]
        theta_vector[5,1:3] = theta_23_t1[0,:]
        theta_vector[6,1:3] = theta_23_t1[1,:]
        theta_vector[7,1:3] = theta_23_t1[1,:]

        # theta 4 5 6 computation
        eps = 2.2204e-16
        pi = np.pi
        for i in range(0,7,2):
            # get R4*R5*R6
            R1 = np.matrixlib.mat(R_matrix(np.array([0,0,-1]),theta_vector[i,0]))
            R2 = np.matrixlib.mat(R_matrix(np.array([0,1,0]),theta_vector[i,1]))
            R3 = np.matrixlib.mat(R_matrix(np.array([0,1,0]),theta_vector[i,2]))
            R_1_3 = R1*R2*R3
            R_4_6 = np.dot(np.linalg.inv(R_1_3),R_1_6)
            
            # decouple R4*R5*R6, 2 solutions
            # step 1. compute theta5,2 solutions
            theta5_1 = math.acos(R_4_6[0,0])
            if(abs(math.cos(theta5_1)-R_4_6[0,0])>eps):
                theta5_1 = -theta5_1
            
            # step 2. compute theta4, 2 solutions, but it needs to valify the
            # result rotation matrix is in the correct sign amount the origin
            # matrix
            theta4_1 = math.atan2(-R_4_6[1,0],-R_4_6[2,0])
            while(abs(-math.sin(theta5_1)*math.sin(theta4_1)-R_4_6[1,0])>10*eps or np.abs(-math.sin(theta5_1)*math.cos(theta4_1)-R_4_6[2,0])>10*eps):
                if theta4_1>=0:
                    theta4_1 = theta4_1-pi
                else:
                    theta4_1 = pi+theta4_1
                
            
            # step 3. compute theta6, 2 solutions, just like theta4, solution needs
            # to validate(compare with origin matrix)
            theta6_1 = math.atan2(-R_4_6[0,1],R_4_6[0,2])
            while(abs(math.sin(theta5_1)*math.cos(theta6_1)-R_4_6[0,2])>10*eps or abs(-math.sin(theta5_1)*math.sin(theta6_1)-R_4_6[0,1])>10*eps):
                if theta6_1>=0:
                    theta6_1 = theta6_1-pi
                else:
                    theta6_1 = pi+theta6_1
                
            # step 4. compute solution 2 of each joint
            theta5_2 = -theta5_1
            theta4_2 = math.atan2(-R_4_6[1,0],-R_4_6[2,0])

            while(abs(-math.sin(theta5_2)*math.sin(theta4_2)-R_4_6[1,0])>10*eps or abs(-math.sin(theta5_2)*math.cos(theta4_2)-R_4_6[2,0])>10*eps):
                if theta4_2>=0:
                    theta4_2 = theta4_2-pi
                else:
                    theta4_2 = pi+theta4_2
                

            theta6_2 = math.atan2(-R_4_6[0,1],R_4_6[0,2])
            while(abs(math.sin(theta5_2)*math.cos(theta6_2)-R_4_6[0,2])>10*eps or abs(-math.sin(theta5_2)*math.sin(theta6_2)-R_4_6[0,1])>10*eps):
                if theta6_2>=0:
                    theta6_2 = theta6_2-pi
                else:
                    theta6_2 = pi+theta6_2
                
            theta_vector[i:i+2,3:6] = np.array([[theta4_1,theta5_1,theta6_1],[theta4_2,theta5_2,theta6_2]])
            #     # this code is to validate the result and origin matrix
            #     for j = 1:2
            #         R_result = R_matrix([-1,0,0],theta_vector(i,4))*R_matrix([0,1,0],theta_vector(i,5))*R_matrix([-1,0,0],theta_vector(i,6))
            #         diff = R_result/R_4_6
            #     
        return theta_vector
        


    def compute_theta23(self,wrist_pos,link1,link2,offset_angle):
        # this function is to compute angle 2 and 3
        # parameters: axis456(in):position of wrist
        # parameters: l1(in): length of link 1
        # parameters: l2(in): length of link 2
        # parameters: offset(in): angle offset

        # get position of target position
        x = wrist_pos[0]
        y = wrist_pos[2]
        lxy = math.sqrt(x**2+y**2)

        # compute theta3
        alpha = (math.acos(( link1**2 + link2**2 - (x**2 + y**2) )/(2*link1*link2)))
        theta3_1 = math.pi - alpha + offset_angle
        theta3_2 = alpha - math.pi + offset_angle

        # compute theta2
        beta = math.acos((link1**2 + lxy**2 - link2**2)/(2*link1*lxy))
        theta2_2 = math.atan2(y,x) - beta
        theta2_1 = 2*beta + theta2_2
        # assign result
        theta_23 = np.array([[-theta2_1,theta3_1],[-theta2_2,theta3_2]])
        return theta_23

def main():
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

    fk = FK_solver(w_mat=w_mat,q_mat=q_mat,gst_0=gst_0)
    with open('angles.xml','r') as file:
        vec_array = np.deg2rad(list(map(float,file.readline().split(','))))   
    # vec_array = np.deg2rad([-31.040000, -38.880000, -88.800000, 71.310000, -81.040000, 187.680000])
    gst = fk.compute_fk(theta_vec=vec_array)
    print(gst)
    ik = IK_solver()
    time_start = time.time()
    theta_vector = ik.compute_IK(gst=gst,gst_0=gst_0,gst_0_0=gst_0_0)
    print('time elapsed for ik:',time.time() - time_start)

    print(theta_vector)
    with open('output_py.txt','wb') as file:
        np.savetxt(file, theta_vector, delimiter=' ', newline='\n', header='', footer='', comments='# ')
        #  np.savetxt(file, theta_vector, delimiter=' ', newline='\n', header='', footer='', comments='# ',fmt='%.8f')
    print('write file done.')



if __name__ == "__main__":
    main()