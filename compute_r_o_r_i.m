clear
clc

% For better visulization, use .mlx file.
syms E r_o r_i r_prev_o r_prev_i pi F nu

r_prev_i = 3.6/2;
r_prev_o = 9/2;
E = 0.8;
% nu = .45;

% Solve the eq1 and eq2
eq1 = r_o - r_prev_o*(1-((F*nu)/(E*pi*(r_o^2-r_i^2)))) == 0;
eq2 = r_i - r_prev_i*(1-((F*nu)/(E*pi*(r_o^2-r_i^2)))) == 0;
s = (solve(eq1,eq2,r_o,r_i,'ReturnConditions',true,'MaxDegree', 3))
r_o = s.r_o(4,1)
r_i = s.r_i(4,1)
 
% r_i = s.r_i
% A = pi*(r_o^2 - r_i^2)



%%
clear
% test the numerical correctness
for F = -1:.01:-0.01

 r_o =  9/(10*(- (27*F)/(280*pi) + (((27*F)/(280*pi) - 27/125)^2 - 729/15625)^(1/2) + 27/125)^(1/3)) + (5*((((27*F)/(280*pi) - 27/125)^2 - 729/15625)^(1/2) - (27*F)/(280*pi) + 27/125)^(1/3))/2 + 3/2
 r_i =  9/(25*(- (27*F)/(280*pi) + (((27*F)/(280*pi) - 27/125)^2 - 729/15625)^(1/2) + 27/125)^(1/3)) + ((((27*F)/(280*pi) - 27/125)^2 - 729/15625)^(1/2) - (27*F)/(280*pi) + 27/125)^(1/3) + 3/5;
 A = pi*(r_o^2-r_i^2);
 
end
