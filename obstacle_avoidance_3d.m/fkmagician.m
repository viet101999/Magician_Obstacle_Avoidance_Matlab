function [P_0_EE_final] = fkmagician(the1,the2,the3)

%syms l0 l1 l2 l3 l4 l5 l6 l7 the1 the2 the3 the4;
l1 = 102.03; l2 = 177.5; l3 = 190; l4 = 81.3; l7 = 30.6; l6 = 0; l5=l1; %l6 = 0;
%l1 = 10; l2 = 10; l3 = 10; l4 = 0; l7 = 0; l6 = 0; l5=0; %l6 = 2.4;
%the1 = THETA(1); the2 = THETA(2); the3= THETA(3);
% the1 = 90*pi/180; the2 = 118*pi/180; the3= -157*pi/180;
the4 = -(the2 + the3);

%DEFINE TABLE
DH = [  0         0      l1    the1;... 
        l7        pi/2   0     the2;...
        l2        0      0     the3;...
        l3        0      l6    the4;...
        l4        -pi/2  0     0;...
        0         0      -l5   0];

T_i = eye(4);
    
for i = 1:6,
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
P_0_EE_final = P_0_EE(2:6,1:3);
%POSITION OF END EFFECTOR W.R.T (3)
% P_4_EE = [0;0;0;1];
% 
% P_0_EE = T_i * P_4_EE;
end

