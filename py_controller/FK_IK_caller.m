clear all 
close all

theta_vec = deg2rad([-31.040000, -38.880000, -88.800000, 71.310000, -81.040000, 187.680000]);

gst_0 = [0,0,1,980
    0,1,0,0
    -1,0,0,435
    0,0,0,1];
figure(2)
subplot(3,3,1)
w_vec = [0 0 -1;0 1 0;0 1 0;-1 0 0;0 1 0;-1 0 0]';
q_vec = [0 0 400;25 0 400;480 0 400;900 0 435;900 0 435;900 0 435]';

g_st = FK(theta_vec,w_vec,q_vec,gst_0,'b');

tic
theta_vector_IK = IK(g_st);
time_ik = toc

error_sum = zeros(1,8);
error_mat = zeros(4,4,8);
for i = 1:size(theta_vector_IK,1)
    subplot(3,3,i+1)
    fk_solution = FK(theta_vector_IK(i,:),w_vec,q_vec,gst_0,'b');
    error_sum(i) = sum(sum(fk_solution - g_st));
    error_mat(:,:,i) = fk_solution - g_st;
end
% error_sum
figure()
FK(theta_vec,w_vec,q_vec,gst_0,'b');
FK(theta_vector_IK(5,:),w_vec,q_vec,gst_0,'r');






