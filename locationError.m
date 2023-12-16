function [dSO3,dR3] = locationError(g_desired,g_current)
R_d = g_desired(1:3,1:3);
R_c = g_current(1:3,1:3);
r_d = g_desired(1:3,4);
r_c = g_current(1:3,4);

dSO3 = sqrt(trace((R_c-R_d)*(R_c-R_d).'));
dR3 = norm(r_c-r_d);
end