%% prepare the env.
clear all 
close all
%% parameters setting.
theta_vec = deg2rad([50.000000, -10.000000, 20.000000, 30.000000, 40.000000, 50.000000]);
% theta_vec = deg2rad(load('angles.xml'));
initial_guess = [0,0,0,0,0,0]';
angle_limits = deg2rad([-180,180;-180,180;-180,180;-180,180;-180,180;-180,180]);

gst_0 = [0,0,1,52.2
    1,0,0,51.5
    0,1,0,359.7
    0,0,0,1];

w_vec = [0 0 1;0 1 0;0 1 0;0 1 0;0 0 1;1 0 0]';
q_vec = [0 0 0;0 0 53;0 0 173;0 0 293;0 51.5 330;52.2 51.5 359.7]';
%% FK and display
figure(2)
% subplot(3,3,1)
[g_st,T_mat] = FK(theta_vec,w_vec,q_vec,gst_0);
display_skeleton_ur_like(q_vec,T_mat,gst_0,g_st,'b')
%% IK numerical
figure
% tic
[theta_vec_ik,succ] = IK_numerical(g_st,gst_0,w_vec,q_vec,angle_limits,initial_guess)
% toc
theta_vec_ik
theta_vec
figure(3)
[g_st_ik,T_mat_ik] = FK(theta_vec_ik,w_vec,q_vec,gst_0);
display_skeleton_ur_like(q_vec,T_mat_ik,gst_0,g_st_ik,'b')
g_st_ik/g_st
% succ

