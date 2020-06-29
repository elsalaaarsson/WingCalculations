% Used in drag_coeff_skin.m
function out = drag_from_skin_fric_coeff(rho_air, v, S_wet, C_f)
D = 1/2 * rho_air * v^2 * S_wet * C_f;
out = D;
end