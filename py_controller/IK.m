function theta_vector = IK(gst)

theta_vector = zeros(8,6)

gst_0 = [0,0,1,81.4+418.6+490
    0,1,0,0
    -1,0,0,400
    0,0,0,1];

gst_0_0 = [0,0,1,418.6+490
    0,1,0,0
    -1,0,0,400
    0,0,0,1];

axis_456 = gst/gst_0*gst_0_0;

axis_456_pos = axis_456(1:4,4);

theta1_0 = -atan2(axis_456_pos(2),axis_456_pos(1));
if theta1_0>0
   theta1_1 = theta1_0 - pi;
elseif theta1_0<0
   theta1_1 = theta1_0 + pi;
end

theta_vector(1:4,1) = theta1_0;
theta_vector(5:8,1) = theta1_1;

T1_0 = eye(4);
T1_1 = eye(4);

T1_0(1:3,1:3) = R_matrix([0,0,-1],theta1_0);
T1_1(1:3,1:3) = R_matrix([0,0,-1],theta1_1);

axis_456_in_t0 = T1_0\axis_456_pos;
axis_456_in_t1 = T1_1\axis_456_pos;

axis_456_in_t0(3) = axis_456_in_t0(3) - 400;
axis_456_in_t1(3) = axis_456_in_t1(3) - 400;

theta_23_t0 = compute_theta23(axis_456_in_t0,490,418.6);
theta_23_t1 = theta_23_t0;


theta_23_t1(1,1) = theta_23_t0(1,1)-pi/2;
theta_23_t1(2,1) = theta_23_t0(2,1)-pi/2;

theta_vector(1,2:3) = theta_23_t0(1,:);
theta_vector(2,2:3) = theta_23_t0(1,:);
theta_vector(3,2:3) = theta_23_t0(2,:);
theta_vector(4,2:3) = theta_23_t0(2,:);
theta_vector(5,2:3) = theta_23_t1(1,:);
theta_vector(6,2:3) = theta_23_t1(1,:);
theta_vector(7,2:3) = theta_23_t1(2,:);
theta_vector(8,2:3) = theta_23_t1(2,:);
theta_vector = axis_456
end

function theta_23 = compute_theta23(axis456_pos,l1,l2)
    x = axis456_pos(1);
    y = axis456_pos(3);
    lxy = sqrt(x^2+y^2);
    alpha = acos(1/2*(x^2 + y^2 - l1^2 - l2^2));
    theta3_1 = alpha - pi;
    theta3_2 = pi - alpha;
    
    beta = acos(lxy/2/l1);
    theta2_2 = atan2(y,x) - beta;
    theta2_1 = 2*beta + theta2_2;
    
    theta_23 = -[theta2_1,theta3_1;theta2_2,theta3_2];  
end

% Rst_0 = gst_0(1:3,1:3);
% 
% Rst = gst(1:3,1:3)/(Rst_0);

% theta5 = -acos(Rst(3,3));
% if theta5 > 0
%    theta5_2 = theta5 - pi;
% elseif theta5 < 0
%    theta5_2 = theta5 + pi;     
% end
% theta6 = acos(Rst(3,1)/(-sin(theta5)));
% theta4 = asin(Rst(2,3)/(-sin(theta5)));
% 
% theta6_2 = acos(Rst(3,1)/(-sin(theta5_2)));
% theta4_2 = asin(Rst(2,3)/(-sin(theta5_2)));
% 
% theta_vector = [theta4,theta5,theta6;theta4_2,theta5_2,theta6_2];

