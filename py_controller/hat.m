function result = hat(vector)
% this is a function to transform vector form of a so(3) or se(3) into
% matrix form

vector = vector(:);
if size(vector,1) == 3
    result = zeros(3);
    result(1,2) = -vector(3);
    result(1,3) = vector(2);
    result(2,1) = vector(3);
    result(2,3) = -vector(1);
    result(3,1) = -vector(2);
    result(3,2) = vector(1);
elseif size(vector,1) == 6
    result = zeros(4);
    v = vector(1:3);
    w = vector(4:end);
    w_hat = hat(w);
    result(1:3,1:3) = w_hat;
    result(1:3,4) = v;
else
    error('please input a valid vector.')
end


end