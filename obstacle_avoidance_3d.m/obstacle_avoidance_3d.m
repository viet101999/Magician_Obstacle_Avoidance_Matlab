clc;
clear all;
%L1 = 10; L2 = 10; L3 = 10; L4 = 0; L7 = 0; L6 = 0; L5=0;
L1 = 102.03; L2 = 177.5; L3 = 190; L4 = 81.3; L7 = 30.6; L6 = 0; L5=L1;
linkLen = [L1;L2;L3;L4;L5];  %lengths of each link

P0_EE = [250;150;0];
q = ikmagician(P0_EE(1),P0_EE(2),P0_EE(3)); %robot configuration (set)
P0_EE_goal = [-300;50;0];
qGoal = ikmagician(P0_EE_goal(1),P0_EE_goal(2),P0_EE_goal(3));

rhoNot = 50; %: defines the distance of influence of the obstacle
isPolygonObs = true; % if true, uses polygonal obstacles

%%% this makes the random generator the same each run.  Useful for
%%% debugging
s = RandStream('mt19937ar','Seed',1);
RandStream.setGlobalStream(s);

% TRAJECTORY
t=0:.04:1;
%[q,qd,qdd] = jtraj(First_Theta,Final_Theta,step);
qtraj=jtraj_magician(q(1:3,1),qGoal(1:3,1),t);
P = size(qtraj,1);
qVirtual_target = qtraj(ceil(1/3*P),:);
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
t = 100;  %how many random steps to take?
v = pi/8; % maximum random value at each step ( link length is max step size, 2*v)
eps_m = 5.1; %epsilon_m:  limit for step sizes for determining local minima



%setup figure
figure(1);clf;
% r = sum(linkLen);
% rectangle('Position',[-r,-r,2*r,2*r],'Curvature',1,'FaceColor',[.9 .9 1]) %robot workspace
%axis equal
view(140,30)
hold on


%draw_obstacle
ptObstacles1 = draw_obstacle(150,80,0,50,50,50);
ptObstacles2 = draw_obstacle(50,160,0,50,50,50);
ptObstacles3 = draw_obstacle(-150,220,50,50,50,50);
ptObstacles = [ptObstacles1;ptObstacles2;ptObstacles3];

%draw robot
% oGoal = [0 0 10; 0 10 10; 0 20 10];
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

maxIters = 200;
for iteration = 1:maxIters  %each iteration performs gradient descent one time
    %calulate error
    %     qErr = sum(atan2(sin(q-qGoal),cos(q-qGoal)).^2); % error
    delta=divelo(radtodeg(q),radtodeg(qGoal));
    qErr=delta(5,1)^2;
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
    Fatt = fatt(q, oGoal,zeta,d); % tinh luc keo robot den setpoint
    
    
    %update drawing
    
    updateArm_3d(oR,hRline,hRpts);
    set(harr, 'Xdata',oR(:,1),'Ydata',oR(:,2),'Zdata',oR(:,3),'Udata',Fatt(:,1),'Vdata',Fatt(:,2),'Wdata',Fatt(:,3)); % ve luc keo robot den setpoint
    set(hrep, 'Xdata',RepsPts(:,1),'Ydata',RepsPts(:,2),'Zdata',RepsPts(:,3),'Udata',Freps(:,1),'Vdata',Freps(:,2),'Wdata',Freps(:,3)); % ve luc day obstacle tac dung len robot
    set(hTitle,'String',[num2str(iteration),' of ', num2str(maxIters), ' error = ',num2str(qErr)])
    drawnow
    if qErr < 0.001
        break
    end
    
    %map the workspace forces to joint torques (5.3.2)
    tau = zeros(3,1);
    for ic = 1:numel(q)
        % torques Fatt
        %         tau = tau +  Jv(q, ic)'*Fatt(ic,:)'; %(5.20) page 174
        tau = tau +  Fatt(ic,:)'; %(5.20) page 174
    end
    % we need a new Jv for floating repulsion points
    for jc = 1:numel(ptObstacles(:,1))
        for ic = 1:numel(q)
            idx = (jc-1)*numel(q)+ic;
            % torques Fatt - torques Freps
            %             tau = tau +  JvArbPt(q,ic,RepsPts(idx,:))'*Freps(idx,:)';
            tau = tau +  Freps(idx,:)';
        end
    end
    
    %Gradient descent algorithm, page 169
    o_temp= fkmagician(q(1),q(2),q(3));
    o_now = o_temp(numel(linkLen),:);
    amin = min(alpha, 1/2*max(abs(o_now-oGoal(numel(linkLen),:))));
    o_now = o_now+(amin*tau/norm(tau))';
    q = ikmagician(o_now(1),o_now(2),o_now(3));
    hold on
    plot3(o_now(1),o_now(2),o_now(3),'or','markersize',2);
    plot3(oVirtual_target(1),oVirtual_target(2),oVirtual_target(3),'diamond','markersize',10);
%     q(1) = q_now(1);
%     q(2) = q_now(2);
%     q(3) = q_now(3);
    
    %     amin = min(alpha, 1/2*max(abs(q-qGoal)));
    %     q = q+amin*tau/norm(tau)
    
    %Task 3  (5pts) detect a local minimum
        q_recent = [q_recent(:,2:end) q];
        if iteration > 3
            inLocalMinimum = (distance(q_recent(:,4),qGoal) > eps_m && ...
                distance(q_recent(:,1),q_recent(:,2)) < eps_m && ...
                distance(q_recent(:,1),q_recent(:,3)) < eps_m && ...
                distance(q_recent(:,1),q_recent(:,4)) < eps_m);
        end
    
        if inLocalMinimum
            % Task 4  (5pts) random walk:
            %execute a random walk.  If it results in collision, do not apply
            %it.
            break;
%             qprime = q;
%             for j =1:t
%                 %generate random +/-1 for each link
%                 vsign = -1+2*rand([numel(q) 1]);
%                 %vsign(vsign<0.5)=-1;
%                 %vsign(vsign>=0.5)=1;
%                 vsign = vsign./moveWeight;
%                 %only assign random walk steps to joints that are further than
%                 %eps_m from goal
%                 vsign((q_recent(:,1)-qGoal(:,1))<eps_m) = 0;
%                 %generate a step
%                 qstep = qprime+vsign*v;
%                 %check for a collision for each obstacle
%                 inCollision = false; % if any joint is in collision (defined as being too close to an obstacle) the move is rejected.
%                 for k = 1:numel(ptObstacles(:,1))
%                     if checkCollision(qstep,ptObstacles(k,:))
%                         inCollision=true;
%                     end
%                 end
%                 if inCollision == false
%                     qprime = qstep;
%                     oR = fkmagician(qprime(1),qprime(2),qprime(3));
%                     updateArm_3d(oR,hRline,hRpts);
%                     drawnow
%                 end
%             end
%             q = qprime;
        end
    
    %pause(0.5)
end