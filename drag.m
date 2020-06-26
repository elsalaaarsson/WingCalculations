function out = drag(rho_air, v, S_wet, C_f)
D = 1/2 * rho_air * v^2 * S_wet * C_f;
out = D;
end