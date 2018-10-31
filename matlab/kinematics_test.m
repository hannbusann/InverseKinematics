clc
clear
close all
%%
% %脚的姿态分析
% r=0;
% p=-90;
% y=-45;
% ankle_from_ground = 3;
% %先绕Z_a转yaw角
% Y = rotz(y)
% %再绕Y_b转pitch角
% P = roty(p)
% %最后绕X_b转roll角
% R = rotx(r)
% %按照变换顺序获得旋转变换矩阵,右乘连体左乘基
% T_r = Y*P*R;
% %脚踝点在脚底坐标系B中的坐标
% P_b = [0 0 ankle_from_ground]'
% %脚踝点在脚底绝对坐标系A中的坐标
% P_a = T_r*P_b
% %经过测试,以上逻辑完全正确,可以得到脚踝点在脚底绝对坐标系A中的坐标

%%
%接下来对其进行符号运算式子的生成,rotx/y/z函数不支持符号运算
syms yy pp rr ankle_from_ground
Y_symbol=[  cos(yy)     -sin(yy)    0;
            sin(yy)     cos(yy)     0;
            0           0           1];

P_symbol=[  cos(pp)     0           sin(pp);
            0           1           0      ;
            -sin(pp)    0           cos(pp)];

R_symbol=[  1           0           0       ;
            0           cos(rr)     -sin(rr);
            0           sin(rr)     cos(rr) ];
        
        
T_r_symbol = Y_symbol * P_symbol * R_symbol;
P_b_symbol = [0; 0; ankle_from_ground]
P_a_symbol = T_r_symbol*P_b_symbol
% 这个P_a_symbol结果可以用于C++代码中结算脚踝














