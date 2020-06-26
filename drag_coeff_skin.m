function out = drag_coeff_skin(Re_crit, rho_air, mu, v, L_char, S_wet, S_ref)
x_crit = Re_crit * mu  / (rho_air * v);
C_f_lam = 1.328 / sqrt(Re_crit);
C_f_lam_to_turb = 0.455 / (log(Re_crit)^2.58);

Re = rho_air * v * L_char / mu;
C_f_turb = 0.455 / (log(Re)^2.58);

% S_wet is the wetted area of the component in question.
S_wet_lam = S_wet * x_crit / L_char;  % The wetted area experiencing lam. flow [m2]
S_wet_turb = S_wet * (1-x_crit) / L_char;  % The wetted area experiencing turb. flow [m2]

D_lam = drag(rho_air, v, S_wet_lam, C_f_lam);  % Drag for the laminar part
D_lam_to_turb = drag(rho_air, v, S_wet_lam, C_f_lam_to_turb);
D_turb = drag(rho_air, v, S_wet_turb, C_f_turb);  % Drag if entire flow was turbulent [N]

D = D_lam - D_lam_to_turb + D_turb;

% S_ref is the 2D reference area for the component.
C_D = D / (1/2 * rho_air * v^2 * S_ref); % Drag coefficient for the component
out = C_D;
end