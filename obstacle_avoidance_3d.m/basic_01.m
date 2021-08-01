function out = basic_01(a,b,d) %a*cos(the)+b*sin(the)=d
anp = atan2(b,a);
the1_1 = anp + atan2(sqrt(abs(a*a+b*b-d*d)),d);
the1_2 = anp + atan2(-sqrt(abs(a*a+b*b-d*d)),d);
out = [the1_1 ;the1_2]
end