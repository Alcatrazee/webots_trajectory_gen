function display_skeleton(q_vec,T_mat,gst_0,gst,color)

axis_len = 200;
dot_pos = zeros(3,7);
dot_pos(:,2) = [0,0,400]';
temp = T_mat(:,:,1)*[q_vec(:,2);1];
dot_pos(:,3) = temp(1:3);
temp_q4 = T_mat(:,:,1)*T_mat(:,:,2)*[480 0 400 1]';
dot_pos(:,4) = temp_q4(1:3);
temp = (T_mat(:,:,1)*T_mat(:,:,2)*T_mat(:,:,3)*[480,0,435,1]');
dot_pos(:,5) = temp(1:3);
temp = (T_mat(:,:,1)*T_mat(:,:,2)*T_mat(:,:,3)*[q_vec(:,4);1]);
dot_pos(:,6) = temp(1:3);
dot_pos(:,7) = gst(1:3,4);

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
axis([-1000,1000,-1000,1000,0,2000])
line([axis_len,0],[0,0],[0,0],'color','r')
line([0,0],[axis_len,0],[0,0],'color','g')
line([0,0],[0,0],[axis_len,0],'color','b')

line([pos_gst(1,1),gst(1,4)],[pos_gst(2,1),gst(2,4)],[pos_gst(3,1),gst(3,4)],'color','b')
line([pos_gst(1,2),gst(1,4)],[pos_gst(2,2),gst(2,4)],[pos_gst(3,2),gst(3,4)],'color','r')
line([pos_gst(1,3),gst(1,4)],[pos_gst(2,3),gst(2,4)],[pos_gst(3,3),gst(3,4)],'color','g')
