function theta_vector = IK(gst)

theta_vector = zeros(8,6);

gst_0 = [0,0,1,81.4+418.6+490
    0,1,0,0
    -1,0,0,400
    0,0,0,1];

gst_0_0 = [0,0,1,418.6+490
    0,1,0,0
    -1,0,0,400
    0,0,0,1];

wrist = gst/gst_0*gst_0_0;

T_1_6 = gst/gst_0;

R_1_6 = T_1_6(1:3,1:3);
axis_456_pos = wrist(1:4,4);


theta1_0 = -atan2(axis_456_pos(2),axis_456_pos(1));
if theta1_0>=0
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
% axis_456_in_t1(3) = axis_456_in_t1(3) - 400;

theta_23_t0 = compute_theta23(axis_456_in_t0,490,418.6);
theta_23_t1 = theta_23_t0;


theta_23_t1(1,1) = pi - theta_23_t0(1,1);
theta_23_t1(2,1) = pi - theta_23_t0(2,1);
theta_23_t1(1,2) = - theta_23_t0(1,2);
theta_23_t1(2,2) = - theta_23_t0(2,2);

theta_vector(1,2:3) = theta_23_t0(1,:);
theta_vector(2,2:3) = theta_23_t0(1,:);
theta_vector(3,2:3) = theta_23_t0(2,:);
theta_vector(4,2:3) = theta_23_t0(2,:);
theta_vector(5,2:3) = theta_23_t1(1,:);
theta_vector(6,2:3) = theta_23_t1(1,:);
theta_vector(7,2:3) = theta_23_t1(2,:);
theta_vector(8,2:3) = theta_23_t1(2,:);

for i=1:2:3
    R1 = R_matrix([0,0,-1],theta_vector(i,1));
    R2 = R_matrix([0,1,0],theta_vector(i,2));
    R3 = R_matrix([0,1,0],theta_vector(i,3));
    R_1_3 = R1*R2*R3;
    R_4_6 = R_1_3\R_1_6;
   
    theta5_1 = acos(R_4_6(1,1));
    theta5_2 = -theta5_1;
    theta6_1 = acos(R_4_6(1,3)/(sin(theta5_1)));
    theta4_1 = acos(R_4_6(3,1)/(-sin(theta5_1)));
%     theta6_1 = atan2(-R_4_6(1,2),R_4_6(1,3));
%     theta4_1 = atan2(-R_4_6(2,1),R_4_6(3,1));
    
    if theta6_1>=0
        theta6_2 = theta6_1-pi;
    else
        theta6_2 = pi-theta6_1;
    end
    
    if theta4_1>=0
        theta4_2 = theta4_1-pi;
    else
        theta4_2 = pi-theta4_1;
    end
    
    
    
%     theta6_2 = acos(R_4_6(1,3)/(sin(theta5_2)));
%     theta4_2 = real(asin(R_4_6(2,1)/(-sin(theta5_2))));
    
    

    theta_vector(i:i+1,4:6) = [theta4_1 theta5_1 theta6_1
                                theta4_2 theta5_2 theta6_2];
    
    if theta4_1>0
        theta4_1 = theta4_1 - pi;
    elseif theta4_1<0
        theta4_1 = theta4_1 + pi;
    end
    
    if theta4_2>0
        theta4_2 = theta4_2 - pi;
    elseif theta4_2<0
        theta4_2 = theta4_2 + pi;
    end
    
    theta_vector(i+4:i+5,4:6) = [theta4_1 theta5_1 theta6_1
                                theta4_2 theta5_2 theta6_2];
                            
%     for j = 1:2
%         R_result = R_matrix([-1,0,0],theta_vector(i,4))*R_matrix([0,1,0],theta_vector(i,5))*R_matrix([-1,0,0],theta_vector(i,6));
%         diff = R_result/R_4_6;
%     end
%     theta_456 = [theta4_1 theta5_1 theta6_1
%                  theta4_2 theta5_2 theta6_2];
end

end

function theta_23 = compute_theta23(axis456_pos,l1,l2)
    x = axis456_pos(1);
    y = axis456_pos(3);
    lxy = sqrt(x^2+y^2);
    alpha = real(acos(( l1^2 + l2^2 - (x^2 + y^2) )/(2*l1*l2)));
    theta3_1 = pi - alpha;
    theta3_2 = alpha - pi;
    
    beta = acos((l1^2 + lxy^2 - l2^2)/(2*l1*lxy));
    theta2_2 = atan2(y,x) - beta;
    theta2_1 = 2*beta + theta2_2;
    
    theta_23 = [-theta2_1,theta3_1;-theta2_2,theta3_2];  
end

% Rst_0 = gst_0(1:3,1:3);
% 
% Rst = gst(1:3,1:3)/(Rst_0);


