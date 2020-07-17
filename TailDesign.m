% Source: https://www.fzt.haw-hamburg.de/pers/Scholz/HOOU/Aircraft_Design_in_a_Nutshell.pdf
WingDesign_Hamburg

C_H = 0.8;  % Conventional volume coefficient of horizontal tail 
C_V = 0.07;  % Conventional volume coefficient of vertical tail 
l_H = 0.660;  % Lever arm of the horizontal tailplane from CAD estimate [m]
l_V = l_H;
b_H = 0.940;  % Tail span


S_H = C_H * S * c / l_H;  % Horizontal tail sizing
% C_H is the conventional volume coefficient of horizontal tail 
% S is the wing area
% c is the mean aerodynamic chord
% l_H is the lever arm of the horizontal tailplane - distance between
% aerodynamic centres of the wing and horizontal tailplane

S_V = C_V * S * b / l_V;
% C_V is the conventional volume coefficient of vertical tail 
% b is the wing span
% l_V is the distance between aerodynamic centres of the wing and 
% horizontal tailplane

c_H = S_H / b_H;  % Chord length of the horizontal tailplane

c_V = c_H;
b_V = S_V / c_V;

l_vert_tail = b_V / 4;