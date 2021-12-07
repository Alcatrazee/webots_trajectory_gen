function out = get_Jacobian(epsi_all,T_mat)

%function get Jacobian for inverse kinematic
%input para: epsi_all a vector of all epsi
%input para:exp_all :vector of all exp

out=[epsi_all(:,1)    Adg(T_mat(:,:,1))*epsi_all(:,2)    Adg(T_mat(:,:,1)*T_mat(:,:,2))*epsi_all(:,3)    Adg(T_mat(:,:,1)*T_mat(:,:,2)*T_mat(:,:,3))*epsi_all(:,4)    Adg(T_mat(:,:,1)*T_mat(:,:,2)*T_mat(:,:,3)*T_mat(:,:,4))*epsi_all(:,5)    Adg(T_mat(:,:,1)*T_mat(:,:,2)*T_mat(:,:,3)*T_mat(:,:,4)*T_mat(:,:,5))*epsi_all(:,6)];