
import numpy as np

def limit_angles(theta_vec,angle_limits):
        for i in range(theta_vec.size):
            if theta_vec[i]<angle_limits[i,0]:
                theta_vec[i] = theta_vec[i] + np.round(np.abs(theta_vec[i])/np.pi)*np.pi
                if theta_vec[i]<angle_limits[i,0]:
                    theta_vec[i] = theta_vec[i] + np.pi
            elif theta_vec[i]>angle_limits[i,1]:
                theta_vec[i] = theta_vec[i] - np.round(np.abs(theta_vec[i])/np.pi)*np.pi
                if theta_vec[i]>angle_limits[i,1]:
                    theta_vec[i] = theta_vec[i] - np.pi
        return theta_vec

max_min_angles_deg = np.array([[-170,170],[-190,45],[-120,156],[-185,185],[-120,120],[-350,350]])
max_min_angles_deg = np.deg2rad(max_min_angles_deg)    
theta_vec = np.array([-9.34567146e-04 ,1.57032056e+00, -1.57058953e+00,  1.49822225e+00 ,-2.08990705e-04, -1.49811803e+00])

print(limit_angles(theta_vec,max_min_angles_deg))