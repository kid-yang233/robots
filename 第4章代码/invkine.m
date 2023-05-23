function thetas = invkine(T0toH)

if sum( size(T0toH) == [4,4] ) ~= 2
    error("The input matrix should be in size (4, 4)");
end
L1 = 4; L2 = 3; L3 = 2;
THto3 = [
        1   0   0   -L3
        0   1   0   0
        0   0   1   0
        0   0   0   1
        ];
T0to3 = T0toH * THto3;

x = T0to3(1, 4);
y = T0to3(2, 4);
c2 = (x^2+y^2-L1^2-L2^2)/(2*L1*L2);
s2_1 = sqrt(1 - c2^2);
s2_2 = -1*sqrt(1-c2^2);
theta2_1 = atan2(s2_1, c2);
theta2_2 = atan2(s2_2, c2);

theta1_1 = theta2to1(theta2_1, x, y);
theta1_2 = theta2to1(theta2_2, x, y);

theta3_1 = atan2( T0to3(2,1), T0to3(1,1) ) - theta1_1 - theta2_1;
theta3_2 = atan2(T0to3(2,1), T0to3(1,1)) - theta1_2 - theta2_2;

thetas = [
        theta1_1, theta2_1, theta3_1;
        theta1_2, theta2_2, theta3_2
        ];


end