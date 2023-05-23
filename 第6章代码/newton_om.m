function T = newton_om(Ri,Wi,dWi,dthetai,ddthetai,Pi1,mi1,Ii1,dvi,Pci)

Wi1 = Ri*Wi + dthetai*[0;0;1];
dWi1 = Ri*dWi + cross(Ri*dWi,dthetai*[0;0;1]) + ddthetai*[0;0;1];
dvi1 = Ri*(cross(dWi,Pi1)+cross(Wi,cross(Wi,Pi1))+dvi);
dcvi1 = cross(dWi1,Pci)+cross(Wi1,cross(Wi1,Pci)) + dvi1;

Fi1 = mi1 * dcvi1;
Ni1 = Ii1*dWi1 + cross(Wi1,Ii1*Wi1);

T = [ Wi1 dWi1 dvi1 Fi1 Ni1 ];

end