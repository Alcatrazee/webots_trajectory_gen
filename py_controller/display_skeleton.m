function display_skeleton(q_vec,T_mat,gst)

dot_pos = zeros(3,5);

dot_pos(:,2) = q_vec(:,1);
temp = (T_mat(:,:,1)*T_mat(:,:,2)*[q_vec(:,3);1]);
dot_pos(:,3) = temp(1:3);
temp = (T_mat(:,:,1)*T_mat(:,:,2)*T_mat(:,:,3)*[q_vec(:,4);1]);
dot_pos(:,4) = temp(1:3);
dot_pos(:,5) = gst(1:3,4);

plot3(dot_pos(1,:),dot_pos(2,:),dot_pos(3,:));
hold on 
grid on
for i = 1:size(dot_pos,2)
   plot3(dot_pos(1,i),dot_pos(2,i),dot_pos(3,i),'marker','o','color','b') 
end
axis([-1000,1000,-1000,1000,0,2000])
line([400,0],[0,0],[0,0],'color','r')
line([0,0],[400,0],[0,0],'color','g')
line([0,0],[0,0],[400,0],'color','b')
