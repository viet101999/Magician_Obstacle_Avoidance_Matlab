function [ ptObstacles ] = obstacle(x,y,z,l,w,h)
% draw obstacles
[a,b,c]=meshgrid([0 1]);
polyObs1=alphaShape(l*a(:)-(l-x),w*b(:)-(w-y),h*c(:)-(0-z));

ptObstacles = polyObs1.Points;
end

