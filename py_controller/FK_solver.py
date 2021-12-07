import numpy as np
import math
from scipy.linalg import expm
import time,numba

class FK_solver:
    def __init__(self,w_mat,q_mat,gst_0):
        self.w = w_mat
        self.q = q_mat
        self.gst_0 = gst_0
        self.eps_vec = np.zeros((6,6))
        self.eps_vec[3:][:] = self.w
        
        self.compute_v()
    
    
    def compute_fk(self,theta_vec):
        T = np.zeros([6,4,4])
        T_all = np.eye(4)
        for i in range(6):
            T[i,:,:] = T_matrix(self.eps_vec[:,i],theta_vec[i])
            T_all = np.dot(T_all,T[i,:,:])
        gst = np.dot(T_all,self.gst_0)

        return gst,T

    def compute_v(self):
        for i in range(6):
            self.eps_vec[0:3,i] = np.cross(-self.eps_vec[3:,i],self.q[:,i].T)

def R_matrix(w,theta):
    return np.eye(3) + hat(w)*math.sin(theta) + np.dot(hat(w),hat(w))*(1-math.cos(theta))

def T_matrix(eps,theta):
    result = np.eye(4)
    w = eps[3:]
    v = eps[0:3]
    R_part = R_matrix(w,theta)
    mat_w_T = np.matrix(w).T
    mat_w = np.matrix(w)
    P_part = np.dot(np.dot((np.eye(3) - R_part),hat(w)),v)+np.dot(mat_w_T*mat_w,v)*theta
    result[0:3,0:3] = R_part
    result[0:3,3] = P_part
    return result 

def hat(vector):
    if vector.size == 3:
        result = np.zeros((3,3))
        result[0,1] = -vector[2]
        result[0,2] = vector[1]
        result[1,0] = vector[2]
        result[1,2] = -vector[0]
        result[2,0] = -vector[1]
        result[2,1] = vector[0]
    elif vector.size ==6 :
        result = np.zeros((4,4))
        v = vector[0:3]
        w = vector[3:]
        w_hat = hat(w)
        result[0:3,0:3] = w_hat
        result[0:3,3] = v
    else:
        print('error while using hat().')
    return result

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

    fk = FK_solver(w_mat=w_mat,q_mat=q_mat,gst_0=gst_0)
    vec_array = np.deg2rad([-31.040000, -38.880000, -88.800000, 71.310000, -81.040000, 187.680000])
    print(fk.compute_fk(theta_vec=vec_array))


if __name__ == "__main__":
    main()