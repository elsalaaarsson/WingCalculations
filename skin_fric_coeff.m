% Used in WingDesign.m
function out = skin_fric_coeff(Re_crit, rho_air, mu_air, v, L_char, S_wet)
% The L_char is the characteristic length of the component in question, and
% x_crit is the length, less than L_char, where the flow transitions from
% laminar to turbulent. 

x_crit = Re_crit * mu_air  / (rho_air * v);  % Critical length where the laminar flow turns turbulent [m]
Re = rho_air * v * L_char / mu_air;  % Reynold's number for the turbulent flow

% Friction coefficients
C_f_lam = 1.328 / sqrt(Re_crit);  % Laminar friction coefficient 
C_f_lam_to_turb = 0.455 / (log(Re_crit)^2.58);  % Friction coefficient if the laminar part were turbulent
C_f_turb = 0.455 / (log(Re)^2.58);  % Friction coefficient for the turbulent flow area

% S_wet is the wetted area of the component in question, and is the surface
% area of a component, in contact with the air.
S_wet_lam = S_wet * x_crit / L_char;  % The wetted area experiencing lam. flow [m2]
S_wet_turb = S_wet * (1- x_crit / L_char);  % The wetted area experiencing turb. flow [m2]

% Drag for the different sections using the three different C_f
D_lam = drag_from_skin_fric_coeff(rho_air, v, S_wet_lam, C_f_lam);  % Drag for the laminar part
D_lam_to_turb = drag_from_skin_fric_coeff(rho_air, v, S_wet_lam, C_f_lam_to_turb);
D_turb = drag_from_skin_fric_coeff(rho_air, v, S_wet_turb, C_f_turb);  % Drag if entire flow was turbulent [N]

% Effective drag
D = D_lam - D_lam_to_turb + D_turb;

% Drag coefficient for the component
C_f = D / (1/2 * rho_air * v^2 * S_wet);
% S_ref is the 2D reference area for the component.
out = C_f;
end