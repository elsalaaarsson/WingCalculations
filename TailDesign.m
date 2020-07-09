% Source: https://www.fzt.haw-hamburg.de/pers/Scholz/HOOU/
WingDesign

C_H = 0.8;  % Conventional volume coefficient of horizontal tail 
C_V = 0.07;  % Conventional volume coefficient of vertical tail 
l_H = 0.5 * length_fuselage;  % Lever arm of the horizontal tailplane [m]
l_V = l_H;

% ------!!! These equations produce weird results !!!---------
% S_H = C_H * S * c / l_H;  % Horizontal tail sizing
% % C_H is the conventional volume coefficient of horizontal tail 
% % S is the wing area
% % c is the mean aerodynamic chord
% % l_H is the lever arm of the horizontal tailplane - distance between
% % aerodynamic centres of the wing and horizontal tailplane
% 
% S_V = C_V * S * b / l_V;
% % C_V is the conventional volume coefficient of vertical tail 
% % b is the wing span
% % l_V is the distance between aerodynamic centres of the wing and 
% % horizontal tailplane

% Source: https://www.researchgate.net/publication/332401073_Conceptual_Design_of_a_QuadPlane_Hybrid_Unmanned_Aerial_Vehicle
% The source uses a boom-mounted tail.
S_H = 0.11 * S;  % Horizontal tail area [m2]
S_V = 0.19 * S;  % Vertical fin area (for both tails) [m2]
