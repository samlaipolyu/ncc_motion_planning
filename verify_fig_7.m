clear
clc

%input = -[.81 .81 .80 .82 .83 .82 -50];
 input = -[.8 .8 .81 .8 .8 .82 -50];

[p_02,s_1,s_2,K_A] = fwk(input);
[draw, tip, sim_angle] = drawAnimation(input,[0 0 0]);

zlim([0 200])
view([150 50])