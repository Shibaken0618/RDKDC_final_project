clc;clear;
p_start = [25;60];
p_end = [40;70];

figure(1)
hold on
plot(p_start(1),p_start(2),'o')
plot(p_end(1),p_end(2),'o')

C = norm(p_end-p_start);
x = sqrt(C^2 - 15^2);
theta = atan2(x,15);
u_diag = (p_end - p_start)/C;
u_par = [cos(theta),-sin(theta);sin(theta),cos(theta)] * u_diag;
p_corner1 = p_start + 10*u_par;
u_perp = [-u_par(2);u_par(1)];
p_corner2 = p_corner1 - x*u_perp;

plot(p_corner1(1),p_corner1(2),'x')
plot(p_corner2(1),p_corner2(2),'x')

c1_error = norm(p_start- p_corner1);
c2_error = norm(p_corner2 - p_end);