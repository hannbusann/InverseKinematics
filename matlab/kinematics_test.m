clc
clear
close all
% %%
% %???姿�????
% r=0;
% p=-90;
% y=-45;
% ankle_from_ground = 3;
% %???Z_a�?aw�?
% Y = rotz(y);
% %???Y_b�?itch�?
% P = roty(p);
% %???�?_b�?oll�?
% R = rotx(r);
% %??????顺�??��???��????��?,?��?�??�????
% T_r = Y*P*R;
% %????��???????�?�?????
% P_b = [0 0 ankle_from_ground]';
% %????��????�?????�?�?????
% P_a = T_r*P_b
% %�??�??,以�??��?�??正确,??���??????��????�?????�?�?????
% 
% %%
% %?��??��??��?�???��?�??�?????,rotx/y/z?��?�??????��?�?% syms yy pp rr ankle_from_ground
% % Y_symbol=[  cos(yy)     -sin(yy)    0;
% %             sin(yy)     cos(yy)     0;
% %             0           0           1];
% % 
% % P_symbol=[  cos(pp)     0           sin(pp);
% %             0           1           0      ;
% %             -sin(pp)    0           cos(pp)];
% % 
% % R_symbol=[  1           0           0       ;
% %             0           cos(rr)     -sin(rr);
% %             0           sin(rr)     cos(rr) ];
% %         
% %         
% % T_r_symbol = Y_symbol * P_symbol * R_symbol;
% % P_b_symbol = [0; 0; ankle_from_ground];
% % P_a_symbol = T_r_symbol*P_b_symbol
% % % �?��P_a_symbol�????��?��?C++代�?�??�??�?% 
% 
% %%
% %�?????�??�???��?�?��?��????�?��??
% %???ankle_roll??��线�???????�????p=-45;
% y=45;
% %???Z_a�?aw�?Y = rotz(y);
% %???Y_b�?itch�?P = roty(p);
% %??????顺�??��???��????��?,?��?�??�????T_r = Y*P;
% %�?nkle_roll??��轴�?B???系中???��????�????P_b = [1 0 0]';
% %?��?ankle_roll�?��???对�??????P_a = T_r*P_b
% 
% %%
% %�??ankle_roll轴线?��????????�表达�?
% syms yy pp rr ankle_from_ground fpx fpy fpz hfox hfoy hfoz
% Y_symbol=[  cos(yy)     -sin(yy)    0;
%             sin(yy)     cos(yy)     0;
%             0           0           1];
% 
% P_symbol=[  cos(pp)     0           sin(pp);
%             0           1           0      ;
%             -sin(pp)    0           cos(pp)];
%         
% R_symbol=[  1           0           0       ;
%             0           cos(rr)     -sin(rr);
%             0           sin(rr)     cos(rr) ];
% 
% T_r_ankle_axis = Y_symbol * P_symbol;
% P_b_ankle_axis = [1; 0; 0];
% P_a_ankle_axis = T_r_ankle_axis*P_b_ankle_axis
% 
% T_r_symbol = Y_symbol * P_symbol * R_symbol;
% P_b_symbol = [0; 0; ankle_from_ground];
% P_a_symbol =[fpx;fpy;fpz]+ T_r_symbol*P_b_symbol-[hfox;hfoy;hfoz]
% 
% 
% %%
% %?��????hip_roll??ip_yaw�?????hip_pitch???
% syms a b c
% Y_symbol=[  cos(yy)     -sin(yy)    0;
%             sin(yy)     cos(yy)     0;
%             0           0           1];
%         
% R_symbol=[  1           0           0       ;
%             0           cos(rr)     -sin(rr);
%             0           sin(rr)     cos(rr) ];
% 
% T_r_symbol = Y_symbol * R_symbol;
% P_a_symbol = [a; b; c];
% P_b_symbol = simplify(T_r_symbol\P_a_symbol)

%%
%这里测试脚踝的结算
% syms yyfuck ppfuck rrfuck afuck bfuck cfuck
% Y_symbol=[  cos(yyfuck)     -sin(yyfuck)    0;
%             sin(yyfuck)     cos(yyfuck)     0;
%             0           0           1];
% 
% P_symbol=[  cos(ppfuck)     0           sin(ppfuck);
%             0           1           0      ;
%             -sin(ppfuck)    0           cos(ppfuck)];
%         
% R_symbol=[  1           0           0       ;
%             0           cos(rrfuck)     -sin(rrfuck);
%             0           sin(rrfuck)     cos(rrfuck) ];
% 
% 
% T_r_symbol = Y_symbol*R_symbol*P_symbol;
% P_a_symbol = [afuck; bfuck; cfuck];
% P_b_symbol = simplify(T_r_symbol\P_a_symbol)

%%
% P_b_symbol = [0;0;-1];
% P_a_symbol = T_r_symbol*P_b_symbol

%%
% 算全身逆运动学
syms rr pp yy rr1 pp1 yy1 
Y=       [  cos(yy1)     -sin(yy1)    0;
            sin(yy1)     cos(yy1)     0;
            0           0           1];

P=       [  cos(pp1)     0           sin(pp1);
            0           1           0      ;
            -sin(pp1)    0           cos(pp1)];

R=       [  1           0           0       ;
            0           cos(rr1)     -sin(rr1);
            0           sin(rr1)     cos(rr1) ];
% 反推     
Y2=[ cos(-yy)     -sin(-yy)    0;
            sin(-yy)     cos(-yy)     0;
            0           0           1];

P2=[ cos(-pp)     0           sin(-pp);
            0           1           0      ;
            -sin(-pp)    0           cos(-pp)];

R2=[ 1           0           0       ;
            0           cos(-rr)     -sin(-rr);
            0           sin(-rr)     cos(-rr) ];

T=simplify(R2*P2*Y2)
Q=simplify(Y*P*R)


%%
syms xxf yyf zzf
M=Y*P*R;
A_P = [xxf; yyf;zzf];
simplify(M\A_P)


