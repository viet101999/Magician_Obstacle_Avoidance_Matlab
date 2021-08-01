function [ ptObstacles ] = draw_obstacle(x,y,z,l,w,h)
% draw obstacles
[a,b,c]=meshgrid([0 1]);
polyObs1=alphaShape(l*a(:)-(l-x),w*b(:)-(w-y),h*c(:)-(0-z));
plot(polyObs1,'edgecolor','none')
xlabel('x');ylabel('y');zlabel('z');
camlight
grid on;
hold on;

ptObstacles = polyObs1.Points;
end

