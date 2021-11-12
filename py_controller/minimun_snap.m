function cof = minmun_snap(initial_state,end_state,T,steps,middle_points)

% example input: initial_state = [0,0,0,0]
% example input: middle_points = [pos1,time1;pos2,time2 ...]

poly_n = 7;

b = [initial_state(:);end_state(:);middle_points(:,1)];
A = zeros(8+size(middle_points,1),poly_n+1);
A(1,8) = 1;
A(2,7) = 1;
A(3,6) = 1;
A(4,5) = 1;

A(5:8,:) = get_poly_n_derivative(poly_n,3,T);
for i = 9:size(A,1)
    k = 1;
    
    for j = 1:(poly_n+1)
        A(i,j) =  middle_points(k,2)^(poly_n-j+1);
    end
    k = k + 1;
end

x = pinv(A)*b;
t_series = linspace(0,T,steps);

s_series = polyval(x,t_series);

plot(t_series, s_series)


end

function cof_matrix = get_poly_n_derivative(poly_order,derivative_order,t)
% this function is to compute derivative_order_th derivative of poly_order_th of polynomial function

cof_matrix = ones(derivative_order+1,poly_order+1);

for i = 1:size(cof_matrix,1)
    for j = 1:size(cof_matrix,2)
        if j>=(poly_order + 3 - i)
            cof_matrix(i,j:end) = 0;
            continue
        else
            cof_matrix(i,j) = factorial(poly_order-j+1)/factorial(poly_order-(i-1)-j+1)*t^(poly_order-(i-1)-j+1);
        end
    end
end

end
