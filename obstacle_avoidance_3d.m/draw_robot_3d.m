clear;clc

linkLen = [10;10;10];

%setup figure
figure(1);clf;
% r = sum(linkLen);
% rectangle('Position',[-r,-r,2*r,2*r],'Curvature',1,'FaceColor',[.9 .9 1]) %robot workspace
axis equal
hold on

%draw_obstacle
x=12;y=13;z=15;
l=2;w=5;h=5;
[a,b,c]=meshgrid([0 1]);
polyObs1=alphaShape(l*a(:)-(l-x),w*b(:)-(w-y),h*c(:)-(0-z));
plot(polyObs1,'edgecolor','none')
xlabel('x');ylabel('y');zlabel('z');
camlight
grid on;
hold on;
x=2;y=13;z=15;
l=2;w=2;h=2;
[a,b,c]=meshgrid([0 1]);
polyObs2=alphaShape(l*a(:)-(l-x),w*b(:)-(w-y),h*c(:)-(0-z));
plot(polyObs2,'edgecolor','none')
xlabel('x');ylabel('y');zlabel('z');

axis([-31 31,-31,31,0,30]);

ptObstacles = [polyObs1.Points;polyObs2.Points]

%draw robot
oGoal = [0 0 10; 10 0 10; 10 0 0];
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
    hClosestLine(j) = line([0,0],[0,1],'color','m');
    set(hClosestLine(j),'marker','.');
end
Freps = zeros(numel(ptObstacles(:,2))*numel(linkLen),2);
RepsPts = zeros(numel(ptObstacles(:,2))*numel(linkLen),2);

updateArm_3d(oGoal,hGline,hGpts); % ve robot setpoint