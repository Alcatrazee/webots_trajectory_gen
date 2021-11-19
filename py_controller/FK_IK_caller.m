%% prepare the env.
clear all 
close all
%% parameters setting.
theta_vec = deg2rad([-31.040000, -38.880000, -88.800000, 71.310000, -81.040000, 187.680000]);

gst_0 = [0,0,1,980
    0,1,0,0
    -1,0,0,435
    0,0,0,1];

gst_0_0 = [0,0,1,900
    0,1,0,0
    -1,0,0,435
    0,0,0,1];

w_vec = [0 0 -1;0 1 0;0 1 0;-1 0 0;0 1 0;-1 0 0]';
q_vec = [0 0 400;25 0 400;480 0 400;900 0 435;900 0 435;900 0 435]';
%% FK and display
figure(2)
subplot(3,3,1)
[g_st,T_mat] = FK(theta_vec,w_vec,q_vec,gst_0);
display_skeleton(q_vec,T_mat,gst_0,g_st,'b')
%% IK
tic
theta_vector_IK = IK(g_st,gst_0,gst_0_0);
time_ik = toc
%% visuallize IK result
error_sum = zeros(1,8);
error_mat = zeros(4,4,8);
T_FK = zeros(4,4,6,8);
for i = 1:size(theta_vector_IK,1)
    subplot(3,3,i+1)
    [gst_ik,T_FK(:,:,:,i)] = FK(theta_vector_IK(i,:),w_vec,q_vec,gst_0);
    display_skeleton(q_vec,T_FK(:,:,:,i),gst_0,gst_ik,'b')
    error_sum(i) = sum(sum(gst_ik - g_st));
    error_mat(:,:,i) = gst_ik - g_st;
end
error_sum
figure()
display_skeleton(q_vec,T_mat,gst_0,g_st,'b')
display_skeleton(q_vec,T_FK(:,:,:,5),gst_0,g_st,'r')

