clear all;
clc;
% Establish linkage system
%     theta d     a   alpha    offset
L_1 = 102.03;
L_2 = 177.5;
L_3 = 190;
L_4 = 81.3;
L_7 = 30.6;
L1 = Link([0 L_1 L_7 pi/2]);
L2 = Link([0 0   L_2 0]);
L3 = Link([0 0   L_3 0]);
L4 = Link([0 0   L_4 0]);

%  Robot model object establishment
Rbt=SerialLink([L1 L2 L3 L4]);
theta=[0,0,0,0];
p=Rbt.fkine(theta)
q=Rbt.ikine(p,'mask', [1 1 1 0 0 1])

% 6Joint variable value of track point
posi0 = [200;300;0];
q0_temp = ikmagician(posi0(1),posi0(2),posi0(3));
q0 = q0_temp(1:4,1);

posi1 = [-200;300;0];
posi2 = [300;50;0];
posi3 = [-300;50;0];
posi4 = [-300;50;0];
posi5 = [-300;50;0];
posi6 = [-300;50;0];

qsq1_temp = ikmagician(posi1(1),posi1(2),posi1(3));
qsq2_temp = ikmagician(posi2(1),posi2(2),posi2(3));
qsq3_temp = ikmagician(posi3(1),posi3(2),posi3(3));
qsq4_temp = ikmagician(posi4(1),posi4(2),posi4(3));
qsq5_temp = ikmagician(posi5(1),posi5(2),posi5(3));
qsq6_temp = ikmagician(posi6(1),posi6(2),posi6(3));

qsq1 = qsq1_temp(1:4,1);
qsq2 = qsq2_temp(1:4,1);
qsq3 = qsq3_temp(1:4,1);
qsq4 = qsq4_temp(1:4,1);
qsq5 = qsq5_temp(1:4,1);
qsq6 = qsq6_temp(1:4,1);


%  Trajectory point planning Fifth degree polynomial to plan trajectory
t=0:.04:1;
%[q,qd,qdd] = jtraj(First_Theta,Final_Theta,step);
sqtraj1=jtraj_magician(q0,qsq1,t); 
sqtraj2=jtraj_magician(qsq1,qsq2,t); 
sqtraj3=jtraj_magician(qsq2,qsq3,t); 
sqtraj4=jtraj_magician(qsq3,qsq4,t);
sqtraj5=jtraj_magician(qsq4,qsq5,t);
sqtraj6=jtraj_magician(qsq5,qsq6,t);
sqtraj7=jtraj_magician(qsq6,q0,t);
hold on

%Variable initialization
atj=zeros(4,4);
view(-35,40)
xlim([-600,600])
ylim([-600,600])
zlim([0,600])

%  Draw1Segment trajectory
for i=1:1:length(t)
    atj=Rbt.fkine(sqtraj1(i,:));
    JTA(i,:)=transl(atj);  %  Extract the translation component of the pose matrix (3Element column vector) stored in JTA vector array
    jta=JTA 
    plot2(jta(i,:),'r.')   %  Draw track points (red points)
    Rbt.plot(sqtraj1(i,:)) %  Draw trajectory animation
    plot2(JTA,'b')         %  Draw a trajectory line (blue line)
    %text(JTA(i,1),JTA(i,2),JTA(i,3),['  (', num2str(JTA(i,1),3), ', ', num2str(JTA(i,2),3),', ', num2str(JTA(i,3),3), ')']);
end
% Draw2Segment trajectory
for i=1:1:length(t)
    atj2=Rbt.fkine(sqtraj2(i,:));
    JTA2(i,:)=transl(atj2);
    jta2=JTA2
    plot2(jta2(i,:),'r.')
    Rbt.plot(sqtraj2(i,:))
    plot2(JTA2,'b')
end
%  Draw3Segment trajectory
for i=1:1:length(t)
    atj3=Rbt.fkine(sqtraj3(i,:));
    JTA3(i,:)=transl(atj3);
    jta3=JTA3
    plot2(jta3(i,:),'r.')
    Rbt.plot(sqtraj3(i,:))
    plot2(JTA3,'b')
end
%  Draw4Segment trajectory
for i=1:1:length(t)
    atj4=Rbt.fkine(sqtraj4(i,:));
    JTA4(i,:)=transl(atj4);
    jta4=JTA4
    plot2(jta4(i,:),'r.')
    Rbt.plot(sqtraj4(i,:))
    plot2(JTA4,'b')
end
%  Draw5Segment trajectory
for i=1:1:length(t)
    atj5=Rbt.fkine(sqtraj5(i,:));
    JTA5(i,:)=transl(atj5);
    jta5=JTA5
    plot2(jta5(i,:),'r.')
    Rbt.plot(sqtraj5(i,:))
    plot2(JTA5,'b')
end
%  Draw6Segment trajectory
for i=1:1:length(t)
    atj6=Rbt.fkine(sqtraj6(i,:));
    JTA6(i,:)=transl(atj6);
    jta6=JTA6
    plot2(jta6(i,:),'r.')
    Rbt.plot(sqtraj6(i,:))
    plot2(JTA6,'b')
end
%  Draw7Segment trajectory
 for i=1:1:length(t)
    atj7=Rbt.fkine(sqtraj7(i,:));
    JTA7(i,:)=transl(atj7);
    jta7=JTA7
    plot2(jta7(i,:),'r.')
    Rbt.plot(sqtraj7(i,:))
    plot2(JTA7,'b')
end