function y = optim(x)
t1 = x(1);
t2 = x(2);
t3 = x(3);
y(1) = 150*cos(t1)*sin(t2) - 100*sin(t1)*sin(t3) + 100*cos(t1)*cos(t2)*cos(t3);
y(2) = 100*cos(t1)*sin(t3) + 150*sin(t1)*sin(t2) + 100*cos(t2)*cos(t3)*sin(t1)-36.7073;
y(3) = 150*cos(t2) - 100*cos(t3)*sin(t2) + 400-576.5009;
% -36.7073 and -576.5009 were added after analysis to obtain the optimal
% values
