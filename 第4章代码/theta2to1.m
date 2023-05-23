function theta1 = theta2to1(theta2, x, y)

L1 = 4; L2 = 3;

k1 = L1 + L2 * cos(theta2);
k2 = L2 * sin(theta2);

% r = sqrt( k1^2 + k2^2 );
% gamma = atan2(k2, k1);

if x == 0 && y == 0
    theta1 = inf;
else
    theta1 = atan2(y, x) - atan2(k2, k1);
end 

end