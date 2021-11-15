function g_st = FK(theta_vec,g_st0)
%input theta_vec

w_vec = [0 0 -1;0 1 0;0 1 0;-1 0 0;0 1 0;-1 0 0]';
q_vec = [0 0 400;0 0 400;490 0 400;908.6 0 400;908.6 0 400;908.6 0 400]';

eps_vec = [zeros(3,6);w_vec];
for i = 1:6
    eps_vec(1:3,i) =  cross(-w_vec(:,i),q_vec(:,i));
end

% tic
% T_mat = zeros(4,4,6);
% T_all = eye(4);
% for i = 1:6
%     T_mat(:,:,i) = expm(hat(eps_vec(:,i))*theta_vec(i));
%     T_all = T_all*T_mat(:,:,i);
% end
% time_expm = toc

tic
T_mat = zeros(4,4,6);
T_all = eye(4);
for i = 1:6
   T_mat(:,:,i) = T_matrix(eps_vec(:,i),theta_vec(i));
   T_all = T_all*T_mat(:,:,i);
end
time_analitic = toc

g_st = T_all*g_st0;


display_skeleton(q_vec,T_mat,g_st)
