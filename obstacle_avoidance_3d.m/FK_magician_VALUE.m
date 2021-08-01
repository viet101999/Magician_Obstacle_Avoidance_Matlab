clc;
close all;

%syms l0 l1 l2 l3 l4 l5 l6 l7 the1 the2 the3 the4;
%l1 = 102.03; l2 = 177.5; l3 = 190; l4 = 81.3; l7 = 30.6; l6 = 0; l5=0; %l6 = 2.4;
l1 = 10; l2 = 10; l3 = 10; l4 = 0; l7 = 0; l6 = 0; l5=0; %l6 = 2.4;
%the1 = THETA(1); the2 = THETA(2); the3= THETA(3);
the1 = 0*pi/180; the2 = 0*pi/180; the3= -90*pi/180;
the4 = -(the2 + the3);

%DEFINE TABLE
DH = [  0         0      l1    the1;... 
        l7        pi/2   0     the2;...
        l2        0      0     the3;...
        l3        0      l6    the4;...
        l4        -pi/2  0     0;...
        0         0      -l5   0];

T_i = eye(4);
    
for i = 1:4,
    % EXTRACT DATA FROM DH TABLE
    a = DH(i,1); anp = DH(i,2); d = DH(i,3); the = DH(i,4);
    
    
    T_i_1_i =  [ cos(the)                -sin(the)               0          a;...
                (sin(the)*cos(anp))     (cos(the)*cos(anp))     -sin(anp)   -(sin(anp))*d;...
                (sin(the)*sin(anp))     (cos(the)*sin(anp))     cos(anp)    (cos(anp))*d;...
                0                       0                       0           1];
    T_i = T_i * T_i_1_i;
    P_4_EE = [0;0;0;1];
    P_0_EE(i,:) = (T_i * P_4_EE)';
end

P_0_EE_final = P_0_EE(2:4,1:3)
% P_0_EE(1) = P_0_EE(1)+l4;
% P_0_EE(3) = P_0_EE(3)-l5;
% P_0_EE
% P_2_3 = [l2;0;0;1];
% 
% P_0_3 = T_i * P_2_3
% 
% P_1_2 = [0;0;l1;1];
% 
% P_0_2 = T_i * P_1_2;