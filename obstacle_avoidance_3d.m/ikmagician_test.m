function [ theta ] = ikmagician_test(x,y,z)
L1 = 10; L2 = 10; L3 = 10; L4 = 0; L7 = 0; L6 = 0; L5=0;

X = x;
Y = y;
Z = z;

theta1 = basic_01(Y, -X, 0); % tinh theta 1 ra 2 nghiem

X = x-L4*cos(theta1(1))-L7*cos(theta1(1));
Y = y-L4*sin(theta1(1))-L7*sin(theta1(1));
Z = z+L5;

for i=1:2
    if((cos(theta1(i))>0.0000001)||(cos(theta1(i))<-0.0000001))
        a(i) = X/cos(theta1(i));
        b = Z - L1;

        c3(i) = (a(i)^2+b^2-L2^2-L3^2)/(2*L2*L3);
    end
    if((sin(theta1(i))>0.0000001)||(sin(theta1(i))<-0.0000001))
        a(i) = Y/sin(theta1(i));
        b = Z - L1;

        c3(i) = (a(i)^2+b^2-L2^2-L3^2)/(2*L2*L3);
    end
end

for i=1:2
    s3(1,i) =  sqrt(1-c3(i)^2);
    s3(2,i) =  -sqrt(1-c3(i)^2);
end

for i=1:4
    theta3(i,:) = atan2(s3(i),c3);
end

for i=1:8
    for j=1:2
        m(i) = L3*cos(theta3(i))+L2;
        n(i) = L3*sin(theta3(i));
    
        theta2(i,j) = atan2(b,a(j)) - atan2(n(i),m(i));
    end
end
theta4 = -theta2(2) - theta3(2);
theta = [theta1(1);theta2(2);theta3(2)];


end

