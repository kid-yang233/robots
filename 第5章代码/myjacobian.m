function J = myjacobian(Theta, L1, L2, L3)

t1 = Theta(1);
t2 = Theta(2);
t3 = Theta(3);

J = [
    -L1*sin(t1)-L2*sin(t1+t2)-L3*sin(t1+t2+t3),    -L2*sin(t1+t2)-L3*sin(t1+t2+t3),    -L3*sin(t1+t2+t3) ;
    L1*cos(t1)+L2*cos(t1+t2)+L3*cos(t1+t2+t3),      L2*cos(t1+t2)+L3*cos(t1+t2+t3),     L3*cos(t1+t2+t3);
    1,                                              1,                                  1
    ];




end