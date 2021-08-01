clc;
clear all;

%L1 = 10; L2 = 10; L3 = 10; L4 = 0; L7 = 0; L6 = 0; L5=0;
L1 = 102.03; L2 = 177.5; L3 = 190; L4 = 81.3; L7 = 30.6; L6 = 0; L5=L1;
linkLen = [L1;L2;L3;L4;L5];  %lengths of each link

P0_EE = [250;150;0];
q = ikmagician(P0_EE(1),P0_EE(2),P0_EE(3)); %q start
P0_EE_goal = [-250;100;0];
qGoal = ikmagician(P0_EE_goal(1),P0_EE_goal(2),P0_EE_goal(3)); %q goal

rhoNot = 50; %: defines the distance of influence of the obstacle

% %%% this makes the random generator the same each run.  Useful for
% %%% debugging
% s = RandStream('mt19937ar','Seed',1);
% RandStream.setGlobalStream(s);

% TRAJECTORY
tv=0:.04:1;
qd0 = [0,2,0]; %velocity
qd1 = [0,0,0];
%[q,qd,qdd] = jtraj(First_Theta,Final_Theta,step);
qtraj=jtraj_magician(q(1:3,1),qGoal(1:3,1),tv,qd0,qd1);
P = size(qtraj,1);
Virtual_numerator = 1;
Virtual_denominator = 5; %divide qtraj into 'Virtual_denominator' parts
qVirtual_target = qtraj(ceil(Virtual_numerator/Virtual_denominator*P),:); %take the first part
oVirtual_target_temp = fkmagician(qVirtual_target(1),qVirtual_target(2),qVirtual_target(3));
oVirtual_target = oVirtual_target_temp(numel(linkLen),:);

moveWeight = flipud(cumsum(flipud(linkLen))); %moving a base link moves all further links, so moves of base links are penalized more than distal links
%q = rand(numel(linkLen),1)*2*pi;  %robot configuration (random)

%qGoal = rand(numel(linkLen),1)*2*pi; %TODO: some sort of check to make sure this isn't intersecting an obstacle

% oGoal = computeOrigins(qGoal); %robot goal DH origins
oGoal = fkmagician(qGoal(1),qGoal(2),qGoal(3)); %robot goal DH origins

q_recent = zeros(numel(q),4); %recent positions of robot arm

% Parameters  (should change these)
% zeta = flipud(.1*cumsum(ones(numel(q),1)));%zeta: vector parameter that scales the forces for each degree-of-freedom
zeta = [0.01;0.03;0.02;0.01;0.01];
alpha = 5; %step size for each iteration.  In motion planning problems, the choice for alpha is often made on an ad hoc or empirical basis, such as the distance to the nearest obstacle or goal
d = 100; %d: the distance that defines the transition from conic to parabolic
% eta = ones(numel(q),1); %eta: vector parameter that scales the repulsive forces for each degree-of-freedom
eta_test = 300000;
eta = [eta_test;eta_test;eta_test;eta_test;eta_test];
inLocalMinimum = false;
% t = 100;  %how many random steps to take?
% v = pi/8; % maximum random value at each step ( link length is max step size, 2*v)
eps_m = alpha+alpha/10; %epsilon_m:  limit for step sizes for determining local minima

fail = 1;

%setup figure
figure(1);clf;
%axis equal
view(140,30)
hold on


%draw_obstacle
ptObstacles_none = zeros(8,3);
ptObstacles1 = draw_obstacle(170,200,0,50,50,50);
ptObstacles2 = draw_obstacle(20,250,100,50,50,50);
ptObstacles3 = draw_obstacle(-170,200,100,50,50,50);
ptObstacles = [ptObstacles1;ptObstacles2;ptObstacles3];

%draw robot
hGline = line([0,0],[0,0],[0,1],'color','g','linewidth',4); % ve line setpoint
hGpts = plot3([0,0],[0,0],[0,1],'og','markersize',12); % ve hinh tron setpoint
hRline = line([0,0],[0,0],[0,1],'color','b','linewidth',4); % ve line robot hien tai
hRpts = plot3([0,0],[0,0],[0,1],'ob','markersize',12); % ve hinh tron robot hien tai
harr= quiver(0,0,1,2,'color',[0,0.8,0],'linewidth',2); % vecto keo
hrep= quiver(0,0,1,2,'color',[0,0,1],'linewidth',2); % vecto day
hold off
hTitle = title(num2str(0));
set(hTitle,'fontsize',24);
set(gca,'fontsize',20)
hClosestLine = zeros(numel(ptObstacles(:,2))*numel(linkLen),1);
for j = 1:numel(ptObstacles(:,2))*numel(linkLen);
    hClosestLine(j) = line([0,0],[0,0],[0,1],'color','m');
    set(hClosestLine(j),'marker','.');
end
Freps = zeros(numel(ptObstacles(:,2))*numel(linkLen),3);
RepsPts = zeros(numel(ptObstacles(:,2))*numel(linkLen),3);

updateArm_3d(oGoal,hGline,hGpts); % ve robot setpoint

maxIters = 1000;
for iteration = 1:maxIters  %each iteration performs gradient descent one time
    % change Obstacles
%     if iteration > 300
%         ptObstacles = zeros(16,3);
%         ptObstacles = [ptObstacles1;ptObstacles2];
%     else
%         ptObstacles = [ptObstacles1;ptObstacles2;ptObstacles3];
%     end

    %calulate error
    %qErr = sum(atan2(sin(q-qGoal),cos(q-qGoal)).^2); % error
    delta=divelo(radtodeg(q),radtodeg(qGoal));
    qErr=delta(5,1)^2;
    
    qVirtual_target = qtraj(ceil(Virtual_numerator/Virtual_denominator*P),:);
    oVirtual_target_temp = fkmagician(qVirtual_target(1),qVirtual_target(2),qVirtual_target(3));
    oVirtual_target = oVirtual_target_temp(numel(linkLen),:);
    delta_virtual=divelo(radtodeg(q(1:3,1)),radtodeg(qVirtual_target'));
    qErr_virtual=delta_virtual(5,1)^2; % error of current point and virtual point
    
    oR = fkmagician(q(1),q(2),q(3)); %x,y,z robot
    for j = 1:numel(ptObstacles(:,1))
        [Frep,clpts] = frepPtFloatingPoint(q, ptObstacles(j,:), eta, rhoNot); % tim luc day + diem tren robot gan obstacle
        for n = 1:numel(q)
            idx = (j-1)*numel(q)+n;
            % ve doan thang ngan nhat tu obstalce den robot
            %set(hClosestLine(idx), 'Xdata',[clpts(n,1),ptObstacles(j,1)],'Ydata',[clpts(n,2),ptObstacles(j,2)],'Zdata',[clpts(n,3),ptObstacles(j,3)]);
            Freps(idx,:) = Frep(n,:); % Luc day
            RepsPts(idx,:) =clpts(n,:); % nhung diem tren robot gan obstacle nhat
        end
    end
    Fatt = fatt(q, oVirtual_target_temp,zeta,d); % tinh luc keo robot den setpoint
    
    
    %update drawing
    updateArm_3d(oR,hRline,hRpts);
    set(harr, 'Xdata',oR(:,1),'Ydata',oR(:,2),'Zdata',oR(:,3),'Udata',Fatt(:,1),'Vdata',Fatt(:,2),'Wdata',Fatt(:,3)); % ve luc keo robot den setpoint
    set(hrep, 'Xdata',RepsPts(:,1),'Ydata',RepsPts(:,2),'Zdata',RepsPts(:,3),'Udata',Freps(:,1),'Vdata',Freps(:,2),'Wdata',Freps(:,3)); % ve luc day obstacle tac dung len robot
    set(hTitle,'String',[num2str(iteration),' of ', num2str(maxIters), ' error = ',num2str(qErr)])
    drawnow
    if qErr < 0.001 % if current point = target point -> stop robot
        break
    end
    
    if qErr_virtual < alpha % if current point close virtual point -> set a new virtual point
        if Virtual_numerator < Virtual_denominator
            Virtual_numerator = Virtual_numerator + 1 ;
        end
    end
    
    %map the workspace forces to joint torques (5.3.2)
    tau = zeros(3,1);
    for ic = 1:numel(q)
        % torques Fatt
        % tau = tau +  Jv(q, ic)'*Fatt(ic,:)'; %(5.20) page 174
        tau = tau +  Fatt(ic,:)'; %(5.20) page 174
    end
    % we need a new Jv for floating repulsion points
    for jc = 1:numel(ptObstacles(:,1))
        for ic = 1:numel(q)
            idx = (jc-1)*numel(q)+ic;
            % torques Fatt - torques Freps
            % tau = tau +  JvArbPt(q,ic,RepsPts(idx,:))'*Freps(idx,:)';
            tau = tau +  Freps(idx,:)';
        end
    end
    
    %Gradient descent algorithm, page 169
    o_temp= fkmagician(q(1),q(2),q(3));
    o_now = o_temp(numel(linkLen),:);
    amin = min(alpha, 1/2*max(abs(o_now-oVirtual_target)));
    o_now = o_now+(amin*tau/norm(tau))';
    q = ikmagician(o_now(1),o_now(2),o_now(3));
    
    hold on
    plot3(o_now(1),o_now(2),o_now(3),'or','markersize',2);
    plot3(oVirtual_target(1),oVirtual_target(2),oVirtual_target(3),'diamond','markersize',10);
    
    %detect a local minimum
    q_recent = [q_recent(:,2:end) q];
    if iteration > 3
        inLocalMinimum = (distance(q_recent(1:3,4),qVirtual_target') > eps_m && ...
            distance(q_recent(1:3,1),q_recent(1:3,2)) < eps_m && ...
            distance(q_recent(1:3,1),q_recent(1:3,3)) < eps_m && ...
            distance(q_recent(1:3,1),q_recent(1:3,4)) < eps_m);
    end
    
    if inLocalMinimum
%         fail = fail + 1;
%         if fail >= 10
%             break;
%         end
        qtraj=jtraj_magician(q(1:3,1),qGoal(1:3,1),tv,qd0,qd1);
        Virtual_numerator = ceil(rand*Virtual_denominator);
    end
    
    %pause(0.5)
end