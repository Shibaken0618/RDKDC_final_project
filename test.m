clc;clear;
p_start = [25;60];
p_end = [40;55];

figure(1)
hold on
plot(p_start(1),p_start(2),'o')
plot(p_end(1),p_end(2),'o')

theta = 0;
u_par = [cos(theta);sin(theta)];
u_perp = [-sin(theta);cos(theta)];

p_corner1 = p_start + 10 * u_par;
p_corner2 = p_end - 5 * u_par;


plot(p_corner1(1),p_corner1(2),'o')
plot(p_corner2(1),p_corner2(2),'o')

plot(35,60,'x')
plot(35,55,'x')