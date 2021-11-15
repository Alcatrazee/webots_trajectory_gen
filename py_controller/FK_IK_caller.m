clear all 
theta_vec = [pi/2 -pi/2 pi/2 pi/3 pi/2 0];

gst_0 = [0,0,1,81.4+418.6+490
    0,1,0,0
    -1,0,0,400
    0,0,0,1];
g_st = FK(theta_vec,gst_0);

theta_vector_IK = IK(g_st)