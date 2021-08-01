function [ velo ] = divelo(pos_start,pos_target)
%MOTION Summary of this function goes here
%   Detailed explanation goes here
dXYZ=pos_target-pos_start;
dX=dXYZ(1,1);
dY=dXYZ(2,1);
dZ=dXYZ(3,1);
EucXY=sqrt(dX^2+dY^2);
EucXYZ=sqrt(EucXY^2+dZ^2);
XYang=atan2d(dY,dX);
Zang=atan2d(dZ,EucXY);
dx=cosd(XYang)*EucXY;
dy=sind(XYang)*EucXY;
dz=sind(Zang)*EucXYZ;
velo=[dx;dy;dz;EucXY;EucXYZ;Zang];
assignin('base','velo',velo);
end