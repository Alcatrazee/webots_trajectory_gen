function result = T_matrix(eps,theta)
% this function is to map se(3) to SE(3)

result = eye(4);
w = eps(4:6);
v = eps(1:3);
R_part = R_matrix(eps(4:6),theta);
P_part = (eye(3) - R_part)*hat(w)*v+w*w'*v*theta;

result(1:3,1:3) = R_part;
result(1:3,4) = P_part;

end