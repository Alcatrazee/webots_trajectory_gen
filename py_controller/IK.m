function theta_vector = IK(gst,gst_0,gst_0_0)
%% prepare output
theta_vector = zeros(8,6);
%% compute theta1 two solutions
wrist = gst/gst_0*gst_0_0;
T_1_6 = gst/gst_0;
R_1_6 = T_1_6(1:3,1:3);
wrist_pos = wrist(1:4,4);

theta1_0 = -atan2(wrist_pos(2),wrist_pos(1));
if theta1_0>=0
    theta1_1 = theta1_0 - pi;
elseif theta1_0<0
    theta1_1 = theta1_0 + pi;
end
% output of theta1 two results
theta_vector(1:4,1) = theta1_0;
theta_vector(5:8,1) = theta1_1;
%% compute theta 2 and 3
% rotate joint 1 to obtain a relationship of joint 2 and 3 and so the links
T1_0 = eye(4);
T1_1 = eye(4);

T1_0(1:3,1:3) = R_matrix([0,0,-1],theta1_0);
T1_1(1:3,1:3) = R_matrix([0,0,-1],theta1_1);

axis_456_in_t0 = T1_0\wrist_pos;
axis_456_in_t1 = T1_1\wrist_pos;
% to formalize a shape to solve with 2d geometry knowledge
axis_456_in_t0(3) = axis_456_in_t0(3) - 400;
axis_456_in_t0(1) = axis_456_in_t0(1) - 25;
axis_456_in_t1(3) = axis_456_in_t1(3) - 400;
axis_456_in_t1(1) = axis_456_in_t1(1) - 25;

theta_23_t0 = compute_theta23(axis_456_in_t0,455,sqrt(35^2+420^2),atan(35/420));
theta_23_t1 = compute_theta23(axis_456_in_t1,455,sqrt(35^2+420^2),atan(35/420));

theta_vector(1,2:3) = theta_23_t0(1,:);
theta_vector(2,2:3) = theta_23_t0(1,:);
theta_vector(3,2:3) = theta_23_t0(2,:);
theta_vector(4,2:3) = theta_23_t0(2,:);
theta_vector(5,2:3) = theta_23_t1(1,:);
theta_vector(6,2:3) = theta_23_t1(1,:);
theta_vector(7,2:3) = theta_23_t1(2,:);
theta_vector(8,2:3) = theta_23_t1(2,:);
%% theta 4 5 6 computation
% algorithm: R_st = R0*R1*R2*R3*R4*R5*R6*Rst0, to get R4~R6, we first get
% rid of R1~R3 and Rst0 with inverse multiplication,R4~R6 multiplication
% result is as follows:
% [          cos(t5),                            -sin(t5)*sin(t6),                           cos(t6)*sin(t5)]
% [ -sin(t4)*sin(t5),   cos(t4)*cos(t6) - cos(t5)*sin(t4)*sin(t6), cos(t4)*sin(t6) + cos(t5)*cos(t6)*sin(t4)]
% [ -cos(t4)*sin(t5), - cos(t6)*sin(t4) - cos(t4)*cos(t5)*sin(t6), cos(t4)*cos(t5)*cos(t6) - sin(t4)*sin(t6)]
% due to kr6 900 rotation axis is not actually z axis, it's -y axis instead
for i=1:2:8
    % get R4*R5*R6
    R1 = R_matrix([0,0,-1],theta_vector(i,1));
    R2 = R_matrix([0,1,0],theta_vector(i,2));
    R3 = R_matrix([0,1,0],theta_vector(i,3));
    R_1_3 = R1*R2*R3;
    R_4_6 = R_1_3\R_1_6;
    
    % decouple R4*R5*R6, 2 solutions
    % step 1. compute theta5,2 solutions
    theta5_1 = acos(R_4_6(1,1));
    if(abs(cos(theta5_1)-R_4_6(1,1))>eps)
        theta5_1 = -theta5_1;
    end
    % step 2. compute theta4, 2 solutions, but it needs to valify the
    % result rotation matrix is in the correct sign amount the origin
    % matrix
    theta4_1 = atan2(-R_4_6(2,1),-R_4_6(3,1));
    while(abs(-sin(theta5_1)*sin(theta4_1)-R_4_6(2,1))>10*eps||abs(-sin(theta5_1)*cos(theta4_1)-R_4_6(3,1))>10*eps)
        if theta4_1>=0
            theta4_1 = theta4_1-pi;
        else
            theta4_1 = pi+theta4_1;
        end
    end
    % step 3. compute theta6, 2 solutions, just like theta4, solution needs
    % to validate(compare with origin matrix)
    theta6_1 = atan2(-R_4_6(1,2),R_4_6(1,3));
    while(abs(sin(theta5_1)*cos(theta6_1)-R_4_6(1,3))>10*eps||abs(-sin(theta5_1)*sin(theta6_1)-R_4_6(1,2))>10*eps)
        if theta6_1>=0
            theta6_1 = theta6_1-pi;
        else
            theta6_1 = pi+theta6_1;
        end
    end
    
    % step 4. compute solution 2 of each joint
    theta5_2 = -theta5_1;
    theta4_2 = atan2(-R_4_6(2,1),-R_4_6(3,1));
    while(abs(-sin(theta5_2)*sin(theta4_2)-R_4_6(2,1))>10*eps||abs(-sin(theta5_2)*cos(theta4_2)-R_4_6(3,1))>10*eps)
        if theta4_2>=0
            theta4_2 = theta4_2-pi;
        else
            theta4_2 = pi+theta4_2;
        end
    end
    
    theta6_2 = atan2(-R_4_6(1,2),R_4_6(1,3));
    while(abs(sin(theta5_2)*cos(theta6_2)-R_4_6(1,3))>10*eps||abs(-sin(theta5_2)*sin(theta6_2)-R_4_6(1,2))>10*eps)
        if theta6_2>=0
            theta6_2 = theta6_2-pi;
        else
            theta6_2 = pi+theta6_2;
        end
    end
    
    theta_vector(i:i+1,4:6) = [theta4_1 theta5_1 theta6_1
        theta4_2 theta5_2 theta6_2];
    %     % this code is to validate the result and origin matrix
    %     for j = 1:2
    %         R_result = R_matrix([-1,0,0],theta_vector(i,4))*R_matrix([0,1,0],theta_vector(i,5))*R_matrix([-1,0,0],theta_vector(i,6));
    %         diff = R_result/R_4_6
    %     end
end

end

%% function definition
function theta_23 = compute_theta23(axis456_pos,l1,l2,offset)
% parameters: axis456(in):position of wrist
% parameters: l1(in): length of link 1
% parameters: l2(in): length of link 2
% parameters: offset(in): angle offset

% get position of target position
x = axis456_pos(1);
y = axis456_pos(3);
lxy = sqrt(x^2+y^2);

% compute theta3
alpha = real(acos(( l1^2 + l2^2 - (x^2 + y^2) )/(2*l1*l2)));
theta3_1 = pi - alpha + offset;
theta3_2 = alpha - pi + offset;

% compute theta2
beta = acos((l1^2 + lxy^2 - l2^2)/(2*l1*lxy));
theta2_2 = atan2(y,x) - beta;
theta2_1 = 2*beta + theta2_2;
% assign result
theta_23 = [-theta2_1,theta3_1;-theta2_2,theta3_2];
end