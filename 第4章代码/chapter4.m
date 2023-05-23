clc, clear
L1 = 4; L2 = 3; L3 = 2;
% DH = [ THETA D  A   ALPHA]
link1 = Link([0,  0, 0,  0], 'modified');
link2 = Link([0,  0, L1, 0], 'modified');
link3 = Link([0,  0, L2, 0], 'modified');
bot = SerialLink([link1 link2 link3], 'name', 'T0to3');

T0toH = [
        1   0   0   9
        0   1   0   0
        0   0   1   0
        0   0   0   1
        ];
% 逆运动学解，具体过程见函数invkine
tt = invkine(T0toH);
THto3 = [
        1   0   0   -L3
        0   1   0   0
        0   0   1   0
        0   0   0   1
        ];
T0to3 = T0toH * THto3;
bot.ikine(T0to3, 'mask', [1 1 0 0 0 1])

T0toH = [
        0.5     -0.866  0   7.5373
        0.866   0.5     0   3.9266
        0       0       1   0
        0       0       0   1
        ];
% 逆运动学解，具体过程见函数invkine
tt = invkine(T0toH);
THto3 = [
        1   0   0   -L3
        0   1   0   0
        0   0   1   0
        0   0   0   1
        ];
T0to3 = T0toH * THto3;
bot.ikine(T0to3, 'mask', [1 1 0 0 0 1])


T0toH = [
        0   1   0   -3
        -1  0   0   2
        0   0   1   0
        0   0   0   1
        ];
% 逆运动学解，具体过程见函数invkine
tt = invkine(T0toH);
THto3 = [
        1   0   0   -L3
        0   1   0   0
        0   0   1   0
        0   0   0   1
        ];
T0to3 = T0toH * THto3;
bot.ikine(T0to3, 'mask', [1 1 0 0 0 1])

%无解
T0toH = [
        0.866   -0.5  0   -3.1245
        -0.5   0.866     0   8.1674
        0       0       1   0
        0       0       0   1
        ];
% 逆运动学解，具体过程见函数invkine
%tt = invkine(T0toH);
THto3 = [
        1   0   0   -L3
        0   1   0   0
        0   0   1   0
        0   0   0   1
        ];
T0to3 = T0toH * THto3;
bot.ikine(T0to3, 'mask', [1 1 0 0 0 1])

