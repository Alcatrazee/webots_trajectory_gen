clear all 
close all
% theta_vec = [pi/2 deg2rad(0) -pi/2 0 pi/2 0];
% theta_vec = -pi + (pi+pi).*rand([1 6]);
theta_vec = deg2rad([-6,-50,-12,82,45,36]);

gst_0 = [0,0,1,81.4+418.6+490
    0,1,0,0
    -1,0,0,400
    0,0,0,1];
figure(2)
subplot(3,3,1)
g_st = FK(theta_vec,gst_0);

theta_vector_IK = IK(g_st);
error_sum = zeros(1,8);
error_mat = zeros(4,4,8);
for i = 1:size(theta_vector_IK,1)
    subplot(3,3,i+1)
    fk_solution = FK(theta_vector_IK(i,:),gst_0);
    error_sum(i) = sum(sum(fk_solution - g_st));
    error_mat(:,:,i) = fk_solution - g_st;
end
error_sum
% [          cos(t5),                            -sin(t5)*sin(t6),                           cos(t6)*sin(t5)]
% [ -sin(t4)*sin(t5),   cos(t4)*cos(t6) - cos(t5)*sin(t4)*sin(t6), cos(t4)*sin(t6) + cos(t5)*cos(t6)*sin(t4)]
% [ -cos(t4)*sin(t5), - cos(t6)*sin(t4) - cos(t4)*cos(t5)*sin(t6), cos(t4)*cos(t5)*cos(t6) - sin(t4)*sin(t6)]


