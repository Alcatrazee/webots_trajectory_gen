function result = R_matrix(w,theta)
% this function is to map so(3) to SO(3)

result = eye(3,3) + hat(w)*sin(theta) + hat(w)*hat(w)*(1-cos(theta));
