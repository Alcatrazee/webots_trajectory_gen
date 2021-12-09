from numba.core.decorators import jit
from FK_solver import FK_solver,hat
import numpy as np
import math,time
from scipy import linalg 
import numba


class IK_solver():
    def init(self,gst_0,w_vec,q_vec,angle_limits):
        self.gst_0 = gst_0
        self.w_vec = w_vec
        self.q_vec = q_vec
        self.angle_limits = angle_limits
        self.v = np.zeros([3,6])
        for x in range(6):
            self.v[:,x] = np.cross(-self.w_vec[:,x],self.q_vec[:,x])
        # twist coordinate
        self.epsi_vector = self.make_epsilo()
        self.fk_solver = FK_solver(w_mat=self.w_vec,q_mat=self.q_vec,gst_0=self.gst_0)

    def compute_IK(self,gst,initial_guess):
        # prepare output
        theta_vector = np.zeros([1,6])
        theta_vector,succ = self.inverse_k(gst,initial_guess)
        return theta_vector,succ

    #function: inverse_kinematic
    #input: a homogenous represantation posture of a robot end_effector
    #output: the angle vector needed to reach the posture 
    def inverse_k(self,g_st_theta,initial_guess):
        # error tolarance
        E = 10e-7
        # maximun times of iterate                        
        limit_of_iterate = 100
        # initial state vector(just 6 angles)
        theta_vector = initial_guess
        succeeded = 0
        # apply Newton's method
        for i in range(limit_of_iterate):
            # time_start = time.time()

            # forward kinematic
            # fk_time_start = time.time()
            gst_current,T_mat = self.fk_solver.compute_fk(theta_vector.tolist())
            # time_elapse_fk = time.time() - fk_time_start
            # get epsilon_theta(k)
            # epsi_th_k = self.vee(linalg.logm(np.dot(gst_current,linalg.inv(g_st_theta))))
            # logT_time_start = time.time()
            epsi_th_k = self.log_T(np.dot(gst_current,linalg.inv(g_st_theta)))
            # time_elapse_logT = time.time() - logT_time_start

            # get Jacobian matrix
            # J_time_start = time.time()
            Jacobian = self.get_Jacobian(self.epsi_vector,T_mat)
            # time_elapse_J = time.time() - J_time_start

            # get persuado Jacobian matrix
            # pinv_time_start = time.time()
            ps_Jacobian =  linalg.pinv(Jacobian)                              # take a lot of time to compute
            # time_elapse_J = time.time() - pinv_time_start

            # iterate theta vector of vector(k+1)
            # norm_time_dot = time.time()
            theta_vector = theta_vector - np.dot(ps_Jacobian,epsi_th_k).T[0]
            # time_elapse_dot = time.time() - norm_time_dot

            # calculate the norm of epsilo_theta(k)
            # norm_time_start = time.time()
            Norm_of_phi = linalg.norm(epsi_th_k)
            # time_elapse_norm = time.time() - norm_time_start

            # print('time elapsed for one cycle: fk logT J dot norm',time.time() - time_start,time_elapse_fk,time_elapse_logT,time_elapse_J,time_elapse_dot,time_elapse_norm) 
            # check if we have reached the posture
            if Norm_of_phi<E:
                succeeded=1
                break
        
            
        # print('norm of phi:'+str(Norm_of_phi))
        if succeeded==1:
            # print('inverse result:'+str(theta_vector))
            theta_vector = self.limit_angles(theta_vector,self.angle_limits)
            return theta_vector,succeeded
        else:
            print('cannot solve the equation,maybe the posture is out of range.')
            return np.matrix([[0,0,0,0,0,0]]),succeeded

    def limit_angles(self,theta_vec,angle_limits):
        for i in range(theta_vec.size):
            if theta_vec[i]<angle_limits[i,0]:
                theta_vec[i] = theta_vec[i] + np.round(np.abs(theta_vec[i])/(2*np.pi))*(2*np.pi)
                if theta_vec[i]<angle_limits[i,0]:
                    theta_vec[i] = theta_vec[i] + np.pi
            elif theta_vec[i]>angle_limits[i,1]:
                theta_vec[i] = theta_vec[i] - np.round(np.abs(theta_vec[i])/(2*np.pi))*(2*np.pi)
                if theta_vec[i]>angle_limits[i,1]:
                    theta_vec[i] = theta_vec[i] - np.pi
        return theta_vec


    # epsilo coordinate
    def make_epsilo(self):
        epsi = np.vstack((self.v,self.w_vec))
        return epsi

    # get adjoint transformation matrix
    def Get_Adg(self,exp):
        left_up = exp[0:3,0:3]
        right_up = np.dot(hat(exp[0:3,3]),exp[0:3,0:3])
        left_bottom = np.zeros((3,3))
        right_bottom = exp[0:3,0:3]
        out = np.vstack((np.hstack((left_up,right_up)),np.hstack((left_bottom,right_bottom))))
        return out
    # operation vee,or remove hat
    def vee(self,se3):
        omega = np.matrix([[se3[2,1]],[se3[0,2]],[se3[1,0]]])
        v = np.matrix([[se3[0,3]],[se3[1,3]],[se3[2,3]]])
        out = np.vstack((v,omega))
        return out

    # get Jacobian matrix
    
    def get_Jacobian(self,epsi_all,T_mat):
        epsi_all = np.matrix(epsi_all,dtype=np.float64)
        out = epsi_all[:,0]
        out = np.hstack((out,np.matrix(self.Get_Adg(T_mat[0,:,:]))*epsi_all[:,1]))
        out = np.hstack((out,np.matrix(self.Get_Adg(np.dot(T_mat[0,:,:],T_mat[1,:,:])))*epsi_all[:,2]))
        out = np.hstack((out,np.matrix(self.Get_Adg(np.dot(np.dot(T_mat[0,:,:],T_mat[1,:,:]),T_mat[2,:,:])))*epsi_all[:,3]))
        out = np.hstack((out,np.matrix(self.Get_Adg(np.dot(np.dot(np.dot(T_mat[0,:,:],T_mat[1,:,:]),T_mat[2,:,:]),T_mat[3,:,:])))*epsi_all[:,4])) 
        out = np.hstack((out,np.matrix(self.Get_Adg(np.dot(np.dot(np.dot(np.dot(T_mat[0,:,:],T_mat[1,:,:]),T_mat[2,:,:]),T_mat[3,:,:]),T_mat[4,:,:])))*epsi_all[:,5]))
        return out

    def log_T(self,T):
        eps = 10e-7
        R = T[0:3,0:3]
        p = T[0:3,3]
        if np.abs(np.trace(R)-3) < eps:
            twist = np.vstack((p.reshape(3,1),np.zeros([3,1])))
        else:
            omega = np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]]).T
            theta = math.atan2(np.linalg.norm(omega),np.trace(R)-1)
            omega = omega/np.linalg.norm(omega)
            pitch = np.dot(omega.T,p)/theta
            if np.linalg.norm(np.cross(omega,p)) < eps:
                v = np.dot(pitch,omega)
            else:
                p = (p - np.dot(np.dot(omega.T,p),omega))/(2*np.sin(theta/2))
                v = p*np.cos(theta/2) - np.cross(omega,p) * np.sin(theta/2) + pitch * omega
            twist = np.vstack((v.reshape(3,1),omega.reshape(3,1)))*theta
        return twist

def main():
    # w_mat = np.array([[0,0,-1],
    #                     [0,1,0],
    #                     [0,1,0],
    #                     [-1,0,0],
    #                     [0,1,0],
    #                     [-1,0,0]]).T
    # q_mat = np.array([[0,0,400],
    #                     [25,0,400],
    #                     [480,0,400],
    #                     [900,0,435],
    #                     [900,0 ,435],
    #                     [900, 0, 435]]).T
    # gst_0 = np.array([[0,0,1,980],
    #                     [0,1,0,0],
    #                     [-1,0,0,435],
    #                     [0,0,0,1]])
    w_mat = np.array([[0,0,1],
                        [0,1,0],
                        [0,1,0],
                        [0,1,0],
                        [0,0,1],
                        [1,0,0]]).T
    q_mat = np.array([[0,0,0],
                        [0,0,53],
                        [0,0,173],
                        [0,0,293],
                        [0,51.5 ,330],
                        [52.2, 51.5, 359.7]]).T
    gst_0 = np.array([[0,0,1,52.2],
                        [1,0,0,51.5],
                        [0,1,0,359.7],
                        [0,0,0,1]])

    # gst_0_0 = np.array([[0,0,1,900],
    #                     [0,1,0,0],
    #                     [-1,0,0,435],
    #                     [0,0,0,1]])

    fk = FK_solver(w_mat=w_mat,q_mat=q_mat,gst_0=gst_0)
    with open('angles.xml','r') as file:
        vec_array = np.deg2rad(list(map(float,file.readline().split(','))))   
    vec_array = np.deg2rad([50.000000, -10.000000, 20.000000, 30.000000, 40.000000, 50.000000])
    gst,T = fk.compute_fk(theta_vec=vec_array)

    # gst = [    [-0.407247,     0.223049,    -0.885663,   -63.973226 ],
    #  [-0.901412,    -0.254226,     0.350464,   -49.049778 ],
    # [ -0.146988,     0.941073,     0.304592,  1038.441005 ],
    #  [ 0.000000,     0.000000,     0.000000,     1.000000 ]]

    angle_limits = np.matrix([[-math.pi,math.pi],[-math.pi,math.pi],[-math.pi,math.pi],[-math.pi,math.pi],[-math.pi,math.pi],[-math.pi,math.pi]])
    print(gst)
    ik = IK_solver()
    ik.init(gst_0,w_mat,q_mat,angle_limits)
    time_start = time.time()
    theta_vector = ik.compute_IK(gst,np.array([0,0,0,0,0,0]))
    print('time elapsed for numerical ik:',time.time() - time_start) 

    print(theta_vector)
    with open('output_py.txt','wb') as file:
        np.savetxt(file, theta_vector, delimiter=' ', newline='\n', header='', footer='', comments='# ')
        #  np.savetxt(file, theta_vector, delimiter=' ', newline='\n', header='', footer='', comments='# ',fmt='%.8f')
    print('write file done.')



if __name__ == "__main__":
    main()