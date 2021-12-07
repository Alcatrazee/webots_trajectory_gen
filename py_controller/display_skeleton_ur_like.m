function display_skeleton_ur_like(q_vec,T_mat,gst_0,gst,color)

points_to_draw = [0,0,0;0,0,53;0,45,53;0,-14.5,173;0,45,173;0,-14.5,293;0,51.5,293;0,51.5,359.7;52.2,51.5,359.7]';

axis_len = 100;
dot_pos = zeros(3,9);
dot_pos(:,2) = points_to_draw(:,2);
temp = T_mat(:,:,1)*T_mat(:,:,2)*[points_to_draw(:,3);1];
dot_pos(:,3) = temp(1:3);
temp = T_mat(:,:,1)*T_mat(:,:,2)*[points_to_draw(:,4);1];
dot_pos(:,4) = temp(1:3);
temp = T_mat(:,:,1)*T_mat(:,:,2)*T_mat(:,:,3)*[points_to_draw(:,5);1];
dot_pos(:,5) = temp(1:3);
temp = T_mat(:,:,1)*T_mat(:,:,2)*T_mat(:,:,3)*T_mat(:,:,4)*[points_to_draw(:,6);1];
dot_pos(:,6) = temp(1:3);
temp = T_mat(:,:,1)*T_mat(:,:,2)*T_mat(:,:,3)*T_mat(:,:,4)*[points_to_draw(:,7);1];
dot_pos(:,7) = temp(1:3);
temp = T_mat(:,:,1)*T_mat(:,:,2)*T_mat(:,:,3)*T_mat(:,:,4)*T_mat(:,:,5)*[points_to_draw(:,8);1];
dot_pos(:,8) = temp(1:3);
temp = gst(1:3,4);
dot_pos(:,9) = temp(1:3);


pos_gst = [gst_0(1:3,4),gst_0(1:3,4),gst_0(1:3,4)];
pos_gst = pos_gst + eye(3)*axis_len;
pos_gst = T_mat(:,:,1)*T_mat(:,:,2)*T_mat(:,:,3)*T_mat(:,:,4)*T_mat(:,:,5)*T_mat(:,:,6)*[pos_gst;ones(1,3)];

% dot_pos = [dot_pos];
plot3(dot_pos(1,:),dot_pos(2,:),dot_pos(3,:));
hold on 
grid on
for i = 1:size(dot_pos,2)
   plot3(dot_pos(1,i),dot_pos(2,i),dot_pos(3,i),'marker','o','color',color) 
end
axis([-300,300,-300,300,0,400])
line([axis_len,0],[0,0],[0,0],'color','r')
line([0,0],[axis_len,0],[0,0],'color','g')
line([0,0],[0,0],[axis_len,0],'color','b')

line([pos_gst(1,1),gst(1,4)],[pos_gst(2,1),gst(2,4)],[pos_gst(3,1),gst(3,4)],'color','b')
line([pos_gst(1,2),gst(1,4)],[pos_gst(2,2),gst(2,4)],[pos_gst(3,2),gst(3,4)],'color','r')
line([pos_gst(1,3),gst(1,4)],[pos_gst(2,3),gst(2,4)],[pos_gst(3,3),gst(3,4)],'color','g')
