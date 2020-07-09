% General constants
g = 9.81;       % Gravitational acceleration [m/s2]
rho_air = 1.28; % Density of air at 15˚C [kg/m3]
mu_air = 1.81e-5;  % Viscosity of air at 15°C [kg/(ms)]
Re_crit = 2e5;  % Critical Reynold's number for turbulence transition 

% Design generic
m = 6;          % Mass of the drone [kg]
v_cruise = 30;  % Cruise speed [m/s]
v_stall = 5;   % Stall speed  - arbitrarily chosen! [m/s]
power_loading = 12;  % Typical value of weight to propeller thrust ratio [N/hp] (AIRCRAFT DESIGN ; A Conceptual Approach BY DANIEL P RAYMER.pdf)
% - calculated values -
W = m * g;  % Weight of the drone [N]
gamma = 1.4;  % Ratio of specific heats (physical property of air)
R = 287;  % Specific gas constant for air [J/(kg K)]
T = 273 + 15; % Absolute temperature at 15°C [K]
Ma = v_cruise / sqrt(gamma * R * T);  % Mach number at cruise speed


% Body and wings
S = 0.55;       % Wing area [m2]
AR = 9;         % Aspect ratio
lambda = 0.72;  % Taper ratio (inner chord / tip chord)
% - calculated values - 
b = sqrt(S * AR);  % Wing span [m]
b_with_winglets = b / 1.05;  % Wing span with winglets, as they increase efficiency by 4-7%, 5% was chosen as estimation [m]
c = S / b;  % Mean chord length ("width" of the wing) [m]
c_r = (2 / (1 + lambda)) * c;  % Root chord [m]
c_t	= c_r * lambda;  % Tip chord [m]
percentage_thickness = 0.15;  % Thickness of the wing relative to chord length [frac]
h = percentage_thickness * c;  % Thickness [m]
x_t = 0.3 * c;  % Position of max thickness from the wing front [m]
length_fuselage = 0.4;  % Fuselage length [m]
dia_fuselage = 0.1;  % Fuselage diameter [m]
ang_quarter_chord = atan((c_r - c_t) / 4 / ((b - dia_fuselage) / 2));  % Quarter chord sweep angle [rad]
C_l_max = 1.2731;  % Airfoil maximum lift coefficient

% Components
% - VTOL propellers
N = 4;          % Number of propellers
D = 1;          % Diameter of propeller span [m] -- EDIT!!
C_07R = 1;      % Coefficient of drag of 0.7 length of the propeller -- EDIT!!
% - VTOL motors
dia_VTOL_motor = 0.0425;  % Motor diameter [m]
h_VTOL_motor = 0.042;  % Motor height [m]
% - tail propellers
eff_tail_prop = 1;  % Efficiency of tail propeller [frac]

% Lift coefficients
C_L_max = 0.9 * C_l_max * cos(ang_quarter_chord);  % Max lift coef. for the whole VTOL
C_L = W / (0.5 * rho_air * v_cruise^2 * S);

% List of components
% + Wing, inner (untapered)
% + Wing, outer (tapered) --- the wing is simplified as being one unity
% + Fuselage
% - Tail and tailplane
% - Tail motor
% - Winglets (ignore for now)
% + VTOL propellers
% + VTOL motors
% - Booms
% - Landing gear


% Performance calculations 
thrust_weight_ratio = 550 * eff_tail_prop / v_cruise * 1 / power_loading;  % [N/N]


% Reference areas (S_ref): 2D areas from top view relating to each 
% component
S_ref_wing = S - c_r * dia_fuselage;  % Wing ref area [m2]
S_ref_fuselage = length_fuselage * dia_fuselage;  % Fuselage ref area [m2]
S_ref_VTOL_motors = N * pi() * dia_VTOL_motor^2 / 4; % Total for all four [m2]
S_ref_prop = 0.43 * 0.02;  % Approximate 2D area from the top of the VTOL props [m2]

% Surface areas (S_wet): The surface area in contact with the air for each
% component
S_wet_wing = 2 * S_ref_wing * (1 + 0.25 * percentage_thickness);  % Approx. from German paper [m2]
S_wet_fuselage = pi() * dia_fuselage * length_fuselage ...
    * (1 - 2/(length_fuselage / dia_fuselage))^(2/3) ...
    * (1 + 1/(length_fuselage / dia_fuselage)^2);  % Approx. from german paper [m2]
S_wet_VTOL_motors = N * (pi() * dia_VTOL_motor^2 / 4 + pi() * dia_VTOL_motor * h_VTOL_motor);  % Approx. cylinder [m2]

% Skin friction coefficients
C_f_wing = drag_coeff_skin(Re_crit, rho_air, mu_air, v_cruise, c, S_wet_wing, S_ref_wing);
C_f_fuselage = drag_coeff_skin(Re_crit, rho_air, mu_air, v_cruise, length_fuselage, S_wet_fuselage, S_ref_fuselage);
C_f_VTOL_motors = drag_coeff_skin(Re_crit, rho_air, mu_air, v_cruise, dia_VTOL_motor, S_wet_VTOL_motors, S_ref_VTOL_motors);

% Form factors
FF_wing = (1 + 0.6/x_t * (h/c) + 100 * (h/c)^4) * (1.34 * Ma^0.18 * cos(ang_quarter_chord)^0.28);
FF_fuselage = 1 + 60 / (length_fuselage / dia_fuselage)^3 + (length_fuselage / dia_fuselage) / 400;
    %FF_VTOL_motors = ?;

% Zero-lift drag coefficients per component
C_D_0_wing = drag_coeff_0(C_f_wing, FF_wing, S_wet_wing, S_ref_wing);
C_D_0_fuselage = drag_coeff_0(C_f_fuselage, FF_fuselage, S_wet_fuselage, S_ref_fuselage);
C_D_0_motor = 0.4;  % Typical value for cylinder at Re = 90000 (large differences from different sources!)

% Wing loading, 2 methods:
% Stall speed constraint
wing_loading_1 = 0.5 * rho_air * v_stall^2 * C_L_max;
%
% Cruise speed constraint
e = 0.7;  % Oswald efficiency approximation for "extended slats, flaps and landing gear". Deemed equivalent of having propellers deployed.
K = 1 / (pi() * AR * e);  % Aerodynamic factor
C_D_induced = K * (2 * W / (v_cruise^2 * rho_air * S))^2;  % Induced drag coefficient
C_D_0_prop = 0.1 * N * D * C_07R / S_ref_prop;  % Drag coefficient from VTOL propellers

% Zero-lift (parasite) drag coefficient, independent of C_l
C_D_0 = C_D_0_wing + C_D_0_fuselage + C_D_0_prop;  % Zero-lift parasite drag coefficient, ignoring interference factor

% Parasite drag depending on C_L
    % C_parasite = k * C_L ^2;

% Induced drag
C_induced = C_L^2 / (pi() * AR * e);

% Total drag
    % C_D = C_D_0 + C_induced;