function [point1, point2] = intermediatePointCalc(g_start,g_end)
p_start = g_start(1:2,4);
p_end = g_end(1:2,4);


L1 = .1; %length of first line
L2 = .05; %length of last line

C = norm(p_end-p_start);

if C<.155
    L1 = 7/12 * C;
    L2 = 1/4 * C;
end

x = sqrt(C^2 - (L1+L2)^2);
theta = atan2(x, L1+L2);
u_diag = (p_end - p_start)/C;
u_par = [cos(theta),-sin(theta);sin(theta),cos(theta)] * u_diag;
p_corner1 = p_start + L1*u_par;
u_perp = [-u_par(2);u_par(1)];
p_corner2 = p_corner1 - x*u_perp;

point1 = g_start;
point1(1:2,4) = p_corner1;
point2 = g_start;
point2(1:2,4) = p_corner2;
end