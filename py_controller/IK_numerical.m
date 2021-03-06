function [theta_vector,succ] = IK_numerical(gst,g_st0,w_vec,q_vec,angle_limits,initial_guess)

E = 10e-10;
theta_vector = initial_guess;
eps_vec = [zeros(3,6);w_vec];
succ = 0;
for i = 1:6
    eps_vec(1:3,i) =  cross(-w_vec(:,i),q_vec(:,i));
end
for t = 1:100
    tic
    [gst_current,T_mat] = FK(theta_vector,w_vec,q_vec,g_st0);
%     tic
%     epsi_thk = vee(logm(gst_current/gst));
%     toc
    epsi_thk = log_T(gst_current/gst);
    Jacobian = get_Jacobian(eps_vec,T_mat);
    inv_Jacobian = pinv(Jacobian);
    theta_vector = theta_vector - inv_Jacobian * epsi_thk;
    Norm_of_phi=norm(epsi_thk);
    toc
    stem(t,Norm_of_phi);
    hold on
    %calculate
    if Norm_of_phi<E
        succ = 1;
        break;
    end
end

Norm_of_phi

theta_vector = theta_vector(:)';
theta_vector = limit_angles(theta_vector,angle_limits);

end

function theta_vec = limit_angles(theta_vec,angle_limits)
    for i = 1:size(angle_limits,1)
       if(theta_vec(i)<angle_limits(i,1))
           theta_vec(i) = theta_vec(i) + round(abs(theta_vec(i))/pi)*pi;
       elseif(theta_vec(i)>angle_limits(i,2))
           theta_vec(i) = theta_vec(i) - round(abs(theta_vec(i))/pi)*pi;
       end
    end
end