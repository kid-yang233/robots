clear, clc;
% 已知参数
l1 = 4;
l2 = 3;
l3 = 2;
syms theta1 theta2 theta3;

%          alpha  a    d  theta  isrevolute
L1 = mylink(0,    0,   0,   0,     1);
L2 = mylink(0,    l1,   0,   0,     1);
L3 = mylink(0,    l2,   0,   0,     1);

L1.transMatrix(theta1);
L2.transMatrix(theta2);
L3.transMatrix(theta3);

T_3H = [  1 0 0 l3
          0 1 0 0
          0 0 1 0
          0 0 0 1];
        
H1 = L1.transMatrix(0)*L2.transMatrix(0)* L3.transMatrix(0) * T_3H;
H2 = L1.transMatrix(pi/18)*L2.transMatrix(pi/9)* L3.transMatrix(pi/6) * T_3H;
H3 = L1.transMatrix(pi/2)*L2.transMatrix(pi/2)* L3.transMatrix(pi/2) * T_3H;


LH = Link([0, 0, l3, 0], 'modified');
LH.A(0);
L(1)=Link([0,0,0,0],'modified');
L(2)=Link([0,0,4,0],'modified');
L(3)=Link([0,0,3,0],'modified');
threeLink = SerialLink(L,'name','ThreeLink');

T1 = threeLink.fkine([0,0,0])
T2 = threeLink.fkine([pi/18,pi/9,pi/6])
T3 = threeLink.fkine([pi/2,pi/2,pi/2])



