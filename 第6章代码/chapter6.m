clear; 
l1 = 4; l2 = 3; l3 = 2;
Theta = [10; 20; 30]*pi/180;
m1=20;m2=15;m3=10;
I1=0.5;I2=0.2;I3=0.1;

dtheta = [1;2;3];
ddtheta = [0.5;1;1.5];
G = [0;-9.81;0];
Omega = [0;0;0];

L1 = mylink(0,    0,   0,   0,     1);
L2 = mylink(0,    l1,   0,   0,     1);
L3 = mylink(0,    l2,   0,   0,     1);
T01 = L1.transMatrix(Theta(1));R01 = T01(1:3,1:3);R10 =inv(R01);
T12 = L2.transMatrix(Theta(2));R12 = T12(1:3,1:3);R21 =inv(R12);
T23 = L3.transMatrix(Theta(3));R23 = T23(1:3,1:3);R32 =inv(R23);
Th = [  1 0 0 l3
          0 1 0 0
          0 0 1 0
          0 0 0 1];R3h = Th(1:3,1:3);Rh3 =inv(R3h);
T_1 = newton_om(R10,Omega,[0;0;0],dtheta(1),ddtheta(1),[0;0;0],m1,I1,G,[l1/2;0;0]);
W1 = T_1(:,1); dW1=T_1(:,2);dv1=T_1(:,3);F1= T_1(:,4);N1= T_1(:,5);
T_2 = newton_om(R21,W1,dW1,dtheta(2),ddtheta(2),[l1;0;0],m2,I2,dv1,[l2/2;0;0]);
W2 = T_2(:,1); dW2=T_2(:,2);dv2=T_2(:,3);F2= T_2(:,4);N2= T_2(:,5);
T_3 = newton_om(R32,W2,dW2,dtheta(3),ddtheta(3),[l2;0;0],m3,I3,dv2,[l3/2;0;0]);
W3 = T_3(:,1); dW3=T_3(:,2);dv3=T_3(:,3);F3= T_3(:,4);N3= T_3(:,5);

f4 = [0;0;0];
n4 = [0;0;0];
F_3 = back(N3,n4,Rh3,[l3;0;0],F3,f4,[l3/2;0;0]);
f3 = F_3(:,1);n3 = F_3(:,2);

F_2 = back(N2,n3,R23,[l2;0;0],F2,f3,[l2/2;0;0]);
f2 = F_2(:,1);n2 = F_2(:,2);

F_1 = back(N1,n2,R12,[l1;0;0],F1,f2,[l1/2;0;0]);
f1 = F_1(:,1);n1 = F_1(:,2);

tau3 = n3.' * [0;0;1];
tau2 = n2.' * [0;0;1];
tau1 = n1.' * [0;0;1];
[tau1 tau2 tau3]


theta = [pi/18,pi/9,pi/6]'; % 初始角度
thetaDot = [1,2,3]'; % 初始角速度
v00Dot = [0;-9.81;0];
thetaDotDot = [0.5,1,1.5]';
L(1) = Link('alpha',0,'a',0,'d',0,'modified');
L(2) = Link('alpha',0,'a',l1,'d',0,'modified');
L(3) = Link('alpha',0,'a',l2,'d',0,'modified');
% 连杆质量
L(1).m = m1;
L(2).m = m2;
L(3).m = m3;
% 连杆质心位置
L(1).r = [l1/2;0;0];
L(2).r = [l2/2;0;0];
L(3).r = [l3/2;0;0];
% 连杆惯性张量，式（6.28)
L(1).I = [0,0,0; 0,0,0; 0,0,I1];
L(2).I = [0,0,0; 0,0,0; 0,0,I2];
L(3).I = [0,0,0; 0,0,0; 0,0,I3];
robot=SerialLink(L, 'name', 'robot');
TAU_1G_rtb = robot.rne(theta',thetaDot',thetaDotDot', 'gravity', v00Dot)
TAU_GL_rtb = robot.gravload(theta', v00Dot);



