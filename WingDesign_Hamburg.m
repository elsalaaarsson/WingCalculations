clear
clc

% NACA0015 aerofoil.
% The wing is designed with 2˚ angle of incidence which equals approx. a
% C_L of 0.18. This means that C_D at that angle is approx. 0.008.

% General constants
g = 9.81;       % Gravitational acceleration [m/s2]
rho_air = 1.28; % Density of air at 15˚C [kg/m3]
mu_air = 1.81e-5;  % Viscosity of air at 15°C [kg/(ms)]
Re_crit = 2e5;  % Critical Reynold's number for turbulence transition (2)
gamma = 1.4;  % Ratio of specific heats (physical property of air)
R = 287;  % Specific gas constant for air [J/(kg K)]
temp = 273 + 15; % Absolute temperature at 15°C [K]


% Design generic
m = 6;          % Mass of the drone [kg]
v_cruise = 30;  % Cruise speed [m/s]
v_stall = 4;   % Stall speed  - arbitrarily chosen! [m/s]
% - calculated values -
W = m * g;  % Weight of the drone [N]
Ma = v_cruise / sqrt(gamma * R * temp);  % Mach number at cruise speed


% Body and wings
S = 0.55;       % Wing area, based on m and WS [m2]
AR = 9;         % Aspect ratio (1)
sweep_angle = 39.3 * Ma^2;  % Sweep angle estimate (HBG) [deg] - almost zero (5)
lambda = 0.72;  % Taper ratio (inner chord / tip chord) (1)
lambda_opt = 0.45 * exp(-0.036 * sweep_angle);  % Optimum taper ratio (HBG), ignored because of aesthetics and other sources
% - calculated values - 
b = sqrt(S * AR);  % Wing span [m]
b_with_winglets = b / 1.05;  % Wing span with winglets, as they increase efficiency by 4-7%, 5% was chosen as estimation [m]
c = S / b;  % Mean chord length (front to back length of the wing) [m]
c_r = (2 / (1 + lambda)) * c;  % Root chord [m]
c_t	= c_r * lambda;  % Tip chord [m]
rel_thickness_opt = -0.0439 * atan(3.3450 * Ma - 3.0231) + 0.0986;  % Thickness of the wing relative to chord length [frac]
rel_thickness = 0.15;  % Close to the calculated value above and a standard airfoil (NACA0015)
h = rel_thickness * c;  % Thickness [m]
x_t = 0.3 * c;  % Position of max thickness from the wing front [m]
length_fuselage = 0.435;  % Fuselage length, from CAD [m]
dia_fuselage = 0.15;  % Fuselage diameter, from CAD [m]
ang_quarter_chord = atan((c_r - c_t) / 4 / ((b - dia_fuselage) / 2));  % Quarter chord sweep angle [rad]
C_l_max = 1.2731;  % Airfoil maximum lift coefficient

% Components
% Numbers
n_props = 4;            % Number of propellers
n_vert_tail = 4;        % Number of vertical tail planes
n_landing_gear = 4;     % Number of landing gears

% - VTOL propellers
dia_prop = 0.4572;          % Diameter of propeller span [m]
% - VTOL motors
dia_VTOL_motor = 0.0425;    % Motor diameter [m]
h_VTOL_motor = 0.042;       % Motor height [m]
% - tail propellers
eff_tail_prop = 1;          % Efficiency of tail propeller [frac]
% - Landing gear
len_landing_gear = 0.3;     % Length of landing gear struts [m]
dia_landing_gear = 0.01;    % Diameter of landing gear struts [m]

% Lift coefficients
C_L_max = 0.9 * C_l_max * cos(ang_quarter_chord);  % Max lift coef. for the whole VTOL
C_L = W / (0.5 * rho_air * v_cruise^2 * S);  % Lift coefficient required at cruise flight

% Preliminary wing loading for wing sizing (1)
WS = 0.5 * rho_air * v_stall^2 * C_L_max;


%% Tail sizing
% Source: https://www.fzt.haw-hamburg.de/pers/Scholz/HOOU/Aircraft_Design_in_a_Nutshell.pdf

vol_coeff_hor_tail = 0.8;  % Conventional volume coefficient of horizontal tail 
vol_coeff_vert_tail = 0.07;  % Conventional volume coefficient of vertical tail 
hor_lev_arm = 0.660;  % Lever arm of the horizontal tailplane from CAD estimate [m]
vert_lev_arm = hor_lev_arm;
b_hor_tail = 0.940;  % Tail span


S_H = vol_coeff_hor_tail * S * c / hor_lev_arm;  % Horizontal tail area

S_V = vol_coeff_vert_tail * S * b / vert_lev_arm;  % Vertical tail area

c_hor_tail = S_H / b_hor_tail;  % Chord length of the horizontal tailplane

c_vert_tail = c_hor_tail;  % Vertical tail chord
b_vert_tail = S_V / c_vert_tail;  % Vertical tail total span

l_vert_tail = b_vert_tail / n_vert_tail;  % Length of each vertical tail

% - Tail
x_t_hor_tail = 0.3 * c_hor_tail;  % Position of max thickness from the wing front [m]
x_t_vert_tail = 0.3 * c_vert_tail;  % Position of max thickness from the wing front [m]
h_hor_tail = rel_thickness * c_hor_tail;  % Horizontal tail thickness [m]
h_vert_tail = rel_thickness * c_vert_tail;  % Vertical tail thickness [m]
ang_hor_tail = 0;  % No sweep angle
ang_vert_tail = acos(180/190);  % Vertical tail sweep angle approx. from CAD [rad]


%% Aerodynamic calculations

Re_root = rho_air * v_cruise * c_r / mu_air;  % Reynold's number over the rectangular part of the wing
Re_mean = rho_air * v_cruise * c / mu_air;  % Reynold's number using mean aerodyn. chord
Re_tip = rho_air * v_cruise * c_t / mu_air;  % Reynold's number at the wing tip


% List of components
% + Wing, inner (untapered)
% + Wing, outer (tapered) --- the wing is simplified as being one unity
% + Fuselage
% + Tail and tailplane
% - Tail motor
% - Winglets (ignore for now)
% + VTOL propellers
% + VTOL motors
% - Booms
% + Landing gear


% Reference areas (S_ref): 2D reference areas relating to each 
% component. Largest cross-sect. area of each component was used.
S_ref_wing = S - c_r * dia_fuselage;  % Wing ref area [m2]
S_ref_fuselage = length_fuselage * dia_fuselage;  % Fuselage ref area [m2]
S_ref_VTOL_motors = pi() * dia_VTOL_motor^2 / 4; % Total for all four [m2]
S_ref_prop = 0.43 * 0.02;  % Approximate 2D area from the top of the VTOL props (1) [m2]
S_ref_vert_tail = c_vert_tail * b_vert_tail;  % 2D ref area of vert tail from CAD [m2]
S_ref_hor_tail = c_hor_tail * b_hor_tail;  % 2D area of horizontal tail wing from CAD [m2]
S_ref_landing_gear = dia_landing_gear * len_landing_gear;  % 2D area of landing gear vert. bars from CAD, using the length-wise cross-section [m2]

S_ref = S_ref_wing +  S_ref_fuselage + n_props * (S_ref_VTOL_motors + S_ref_prop) ...
    + n_vert_tail * S_ref_vert_tail + S_ref_hor_tail + n_landing_gear * S_ref_landing_gear;


% Wetted areas (S_wet): The surface area in contact with the air for each
% component
S_wet_wing = 2 * S_ref_wing * (1 + 0.25 * rel_thickness);  % Approx. from (5) [m2]
S_wet_fuselage = pi() * dia_fuselage * length_fuselage ...
    * (1 - 2/(length_fuselage / dia_fuselage))^(2/3) ...
    * (1 + 1/(length_fuselage / dia_fuselage)^2);  % Approx. from (5) [m2]
S_wet_VTOL_motors = (pi() * dia_VTOL_motor^2 / 4 + pi() * dia_VTOL_motor * h_VTOL_motor);  % Approx. cylinder [m2]
S_wet_prop = 0.010817;  % Propeller surface area according to CAD [m2]
S_wet_vert_tail = 0.058927;  % Vertical tail wing surface area according to CAD (only one) [m2]
S_wet_hor_tail = 0.341357;  % Horizontal tail wing surface area according to CAD [m2]
S_wet_landing_gear = 9.5033e-3;  % Vertical bar of landing gear surf. area from CAD [m2]


% Form factors (1)
FF_wing = (1 + 0.6/x_t * (h/c) + 100 * (h/c)^4) * (1.34 * Ma^0.18 * cos(ang_quarter_chord)^0.28);
FF_fuselage = 1 + 60 / (length_fuselage / dia_fuselage)^3 + (length_fuselage / dia_fuselage) / 400;
FF_vert_tail = (1 + 0.6/x_t_vert_tail * (h_vert_tail/c_vert_tail) + 100 * (h_vert_tail/c_vert_tail)^4) * (1.34 * Ma^0.18 * cos(ang_vert_tail)^0.28);
FF_hor_tail = (1 + 0.6/x_t_hor_tail * (h_hor_tail/c_hor_tail) + 100 * (h_hor_tail/c_hor_tail)^4) * (1.34 * Ma^0.18 * cos(ang_hor_tail)^0.28);


% Skin friction coefficients
C_f_wing = skin_fric_coeff(Re_crit, rho_air, mu_air, v_cruise, c, S_wet_wing);
C_f_fuselage = skin_fric_coeff(Re_crit, rho_air, mu_air, v_cruise, length_fuselage, S_wet_fuselage);
C_f_vert_tail = skin_fric_coeff(Re_crit, rho_air, mu_air, v_cruise, c_vert_tail, S_wet_vert_tail);
C_f_hor_tail = skin_fric_coeff(Re_crit, rho_air, mu_air, v_cruise, c_hor_tail, S_wet_hor_tail);


% Zero-lift drag coefficients per component
C_D_0_wing = drag_coeff_0(C_f_wing, FF_wing, S_wet_wing, S_ref_wing);
C_D_0_fuselage = drag_coeff_0(C_f_fuselage, FF_fuselage, S_wet_fuselage, S_ref_fuselage);
C_D_0_vert_tail = drag_coeff_0(C_f_vert_tail, FF_vert_tail, S_wet_vert_tail, S_ref_vert_tail);
C_D_0_hor_tail = drag_coeff_0(C_f_hor_tail, FF_hor_tail, S_wet_hor_tail, S_ref_hor_tail);
% Drag coefficient estimates for components difficult to calculate
C_D_0_landing_gear = 0.015;  % General landing gear drag according to Ref (4)
C_D_0_VTOL = 0.021;  % Drag coefficient from motors and propellers related to the VTOL system, according to Ref (1)


% Zero-lift (parasite) drag coefficient, independent of C_l, ignoring
% interference factor. Take care to multiply by the number of units
% included!
C_D_0 = C_D_0_wing + C_D_0_fuselage + n_vert_tail * C_D_0_vert_tail ... 
    + C_D_0_hor_tail + C_D_0_landing_gear + C_D_0_VTOL; 

% Parasite drag depending on C_L
    % C_parasite = K * C_L ^2;  % K is not the same as k!

e = 0.7;  % Oswald efficiency approximation for "extended slats, flaps and landing gear" (1). Deemed equivalent of having propellers deployed.
e_eq = 1.78 * (1 - 0.045*AR^0.68) - 0.64;  % Idealised equation for Oswald efficiency factor (1)

% Induced drag
k = pi() * AR * e;  % Constant
C_D_induced = C_L^2 / k;  % Induced drag coefficient

% Total drag
C_D = C_D_0 + C_D_induced;
D = 1/2 * rho_air * v_cruise^2 * C_D * S_ref;



%% Performance calculations

wing_loading = m / S;

T_cruise = D;  % The thrust required for stable cruise is equal to the drag [N]
P_cruise = T_cruise * v_cruise;  % The power required at cruise is equal to the thrust * the velocity [W]

k_takeoff = 1.2;  % Scaling factor for the amount of thrust necessary to lift the UAV, relative to the weight (1)
T_takeoff = W * k_takeoff;  % Takeoff thrust [N]

v_mindrag = (W / (1/2 * rho_air * S))^0.5 * (k / C_D_0)^0.25;  % Velocity of minimum drag from Strathclyde Aero Design notes [m/s]
v_minpower = v_mindrag / 3^0.25;


%% References

% (1) Özgür Dündar, Mesut Bilici, Tarık Ünler, Design and performance 
% analyses of a fixed wing battery VTOL UAV. Engineering Science and 
% Technology, an International Journal [Online], 2020, available from
% https://www.sciencedirect.com/science/article/pii/S2215098619316489

% (2) M T Stickland, Aero Design 1. ME201 and 16159 class notes, University
% of Strathclyde, 2003

% (3) NACA0015. AirfoilTools, 2020, available from 
% http://airfoiltools.com/airfoil/details?airfoil=naca0015-il

% (4) Dieter Scholz, Drag Prediction: Aircraft Design Lecture Notes. 
% University of Hamburg [Online], available from 
% https://www.fzt.haw-hamburg.de/pers/Scholz/HOOU/AircraftDesign_13_Drag.pdf

% (5) Dieter Scholz, Aircraft Design in a Nutshell: Aircraft Design Lecture 
% Notes. University of Hamburg [Online], available from 
% https://www.fzt.haw-hamburg.de/pers/Scholz/HOOU/Aircraft_Design_in_a_Nutshell.pdf