clear; 
L1 = 4; L2 = 3; L3 = 2;
Theta0 = [10; 20; 30]*pi/180;
delta_t = 0.1;

% 每一步中的雅可比矩阵
J = myjacobian(Theta0, L1, L2, L3);
% 期望速度
V_c = [0.2;-0.3;-0.2];

% 期望力和力矩
W = [1; 2; 3];


% 每一步中的角度
Theta = Theta0;

% 存放角度
Angle = Theta0;
% 存放关节的角速度
Omega = [0;0;0];

% 机械臂模型
link1 = Link([0, 0, 0, 0], 'modified');
link2 = Link([0, 0, L1, 0], 'modified');
link3 = Link([0, 0, L2, 0], 'modified');
linkH = Link([0, 0, L3, 0], 'modified');
bot = SerialLink([link1 link2 link3 linkH], 'name', 'T0toH');
T = bot.fkine([Theta', 0]);

% 存放末端的位置和姿态
X = T.t; 
phi = Theta(1) + Theta(2) + Theta(3);

% 存放雅可比矩阵的行列式
J_det = [det(J)];

% 存放关节力矩
tao = [J'*W];

for t = 0+delta_t:delta_t:5
    J = myjacobian(Theta, L1, L2, L3);
    if det(J) == 0
        print("singular point\n");
        continue;
    else
        dTheta = inv(J)*V_c;
        Theta = Theta + dTheta*delta_t;
        Omega = [Omega, dTheta];
        Angle = [Angle, Theta];
        
        T = bot.fkine([Theta', 0]);
        X = [X, T.t];
        phi = [phi, Theta(1) + Theta(2) + Theta(3)];
        
        J_det = [J_det, det(J)];
        
        tao = [tao, J'*W];
    end
end


t = 0:delta_t:5;
%plot(t, Angle)
%title("关节角与时间的关系")
%legend("theta1", "theta2", "theta3")
%xlabel("时间/s"); ylabel("角度/rad")

%plot(t, Omega)
%title("关节角速度与时间的关系")
%legend("w1", "w2", "w3")
%xlabel("时间/s"); ylabel("角速度/(rad·s^{-1})")

%plot(t, [X(1:2, :); phi])
%title("三个笛卡尔分量与实践的关系")
%legend("x", "y", "phi")
%xlabel("时间/s"); ylabel("m 或 rad")

%plot(t, J_det)
%title("雅可比矩阵行列式与时间的关系")

%plot(t, tao)
%title("关节力矩与时间的关系")
%legend("tau1", "tau2", "tau3")
%xlabel("时间/s"); ylabel("力或力矩/（N或N·m）")

angle_0 = Angle(:, 1);
angle_end = Angle(:, end);

J_0 = bot.jacob0([angle_0' 0]);
myjacobian(angle_0, L1, L2, L3)

J_end = bot.jacob0([angle_end' 0]);
myjacobian(angle_end, L1, L2, L3)