% General constants
g = 9.81;       % Gravitational acceleration [m/s2]
rho_air = 1.28; % Density of air at 15˚C [kg/m3]

% Design generic
m = 6;          % Mass of the drone [kg]
v_cruise = 30;  % Cruise speed [m/s]
v_stall = 5;   % Stall speed  - arbitrarily chosen! [m/s]
% - calculated values -
W = m * g;  % Weight of the drone [N]

% Body and wings
S = 0.55;       % Wing area [m2]
AR = 9;         % Aspect ratio
lambda = 0.72;  % Taper ratio
% - calculated values - 
b = sqrt(S * AR);  % Wing span [m]
c = S / b;  % Mean chord length [m]
c_r = (2 / (1 + lambda)) * c;  % Root chord [m]
c_t	= c_r * lambda;  % Tip chord [m]
h = 0.15 * c;  % Thickness [m]
length_fuselage = 0.4;  % Fuselage length [m]
dia_fuselage = 0.1;  % Fuselage diameter [m]
ang_quarter_chord = atan((c_r - c_t) / 4 / ((b - dia_fuselage) / 2));  % Quarter chord sweep angle [rad]
C_l_max = 1.2731;  % Airfoil maximum lift coefficient

% Propellers
N = 4;          % Number of propellers
D = 1;          % Diameter of propeller span [m] -- EDIT!!
C_07R = 1;      % Coefficient of drag of 0.7 length of the propeller -- EDIT!!


% Lift coefficients
C_L_max = 0.9 * C_l_max * cos(ang_quarter_chord);  % Max lift coef. for the whole VTOL
C_L = W / (0.5 * rho_air * v_cruise^2 * S);

% List of components
% - Wing, inner (untapered)
% - Wing, outer (tapered)
% - Fuselage
% - Tail and tailplane
% - Tail motor
% - Winglets
% - VTOL propellers
% - VTOL motors
% - Booms
% - Landing gear

% Skin friction coefficients



% Form factors



% Zero-lift drag coefficients per component
    % C_D_0_fuselage = drag_coefficient_0(C_f_fuselage, FF_fuselage, S_wet_fuselage, S_ref);

% Wing loading, 2 methods:
% Stall speed constraint
wing_loading_1 = 0.5 * rho_air * v_stall^2 * C_L_max;
%
% Cruise speed constraint
e = 0.7;  % Oswald efficiency approximation for "extended slats, flaps and landing gear". Deemed equivalent of having propellers deployed.
K = 1 / (pi() * AR * e);  % Aerodynamic factor
C_D_induced = K * (2 * W / (v_cruise^2 * rho_air * S))^2;  % Induced drag coefficient
C_D_0_prop = 0.1 * N * D * C_07R / S;  % Drag coefficient from VTOL propellers

% Zero-lift (parasite) drag coefficient, independent of C_l
    % C_D_0 = C_D_0_fuselage + C_D_0_prop;  % Zero-lift parasite drag coefficient, ignoring interference factor

% Parasite drag depending on C_L
    % C_parasite = k * C_L ^2;

% Induced drag
C_induced = C_L^2 / (pi() * AR * e);

% Total drag
    % C_D = C_D_0 + C_induced;