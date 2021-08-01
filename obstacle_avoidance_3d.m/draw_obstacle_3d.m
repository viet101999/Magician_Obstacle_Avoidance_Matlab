clear;clc
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

axis([0 21,0,21,0,20]);

ptObstacles = [polyObs1.Points;polyObs2.Points]