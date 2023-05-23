function F = back(Ni,ni_1,Ri_1,Pi,Fi,fi_1,Pci)
fi = Ri_1*fi_1 + Fi;
ni = Ni + Ri_1*ni_1  + cross(Pi,Ri_1*fi_1) + cross(Pci,Fi);
F = [fi ni];
end