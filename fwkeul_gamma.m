function euler = fwkeul_gamma(x)


L = 1:1:50; % mm
K_A = 0.8550; % K_A = EA/L = 0.8*pi*(4.5^2-1.8^2)/50 = 0.8550 N/mm

K_B = 5.0211;  % 0.8*313.8175/50 (EI/l=-M/theta_B) Nmmrad-1 //
r = 3.15;

ds_2 = (x(4) + x(5) + x(6))/(3*K_A);
s_2 = L + ds_2;
ds_1 = (x(1) + x(2) + x(3)) + ((x(4) + x(5) + x(6)))/(3*K_A);
s_1 = L + ds_1;
d = 0;

theta_2 = (r*s_2(end)/K_B)*sqrt(x(4)^2+x(5)^2+x(6)^2-x(4)*x(5)-x(4)*x(6)-x(5)*x(6));
phi_2 = atan2(3*(x(5)-x(6)),sqrt(3)*(x(5)+x(6)-2*x(4)));
phi_1 = atan2(3*(x(2)-x(3)),sqrt(3)*(x(2)+x(3)-2*x(1)));

theta_2 = theta_2(1,end);
A = s_1(end)*r*sqrt(x(1)^2+x(2)^2+x(3)^2-x(1)*x(2)-x(1)*x(3)-x(2)*x(3));

Theta_1 = (A^2 - 2*cos(phi_1 - phi_2)*A*K_B*theta_2 + K_B^2*theta_2^2)^(1/2)/K_B;
Phi_1 = deg2rad(180) - angle(((A*cos(phi_1) - K_B*theta_2*cos(phi_2))*1i)/(A^2 - 2*cos(phi_1 - phi_2)*A*K_B*theta_2 + K_B^2*theta_2^2)^(1/2) + (A*sin(phi_1) - K_B*theta_2*sin(phi_2))/(A^2 - 2*cos(phi_1 - phi_2)*A*K_B*theta_2 + K_B^2*theta_2^2)^(1/2));

theta_1 = Theta_1;
phi_1 = real(Phi_1);

% geometry pre-process
x_01 = (s_1(end)/theta_1) * (1-cos(theta_1))*cos(phi_1);
y_01 = (s_1(end)/theta_1) * (1-cos(theta_1))*sin(phi_1);
z_01 = (s_1(end)/theta_1) * sin(theta_1) + d;
p_01 = [x_01; y_01; z_01];

R_0b = rotz(rad2deg(phi_1));
R_01 = rotz(rad2deg(phi_1))*roty(rad2deg(theta_1))*rotz(rad2deg(-phi_1));

x_12 = (s_2(end)/theta_2) * (1-cos(theta_2))*cos(phi_2);
y_12 = (s_2(end)/theta_2) * (1-cos(theta_2))*sin(phi_2);
z_12 = (s_2(end)/theta_2) * sin(theta_2);
p_12 = [x_12; y_12; z_12];

R_12 = rotz(rad2deg(phi_2))*roty(rad2deg(theta_2))*rotz(rad2deg(-phi_2));
R_02 = R_01*R_12;
euler = rotm2eul(R_02)';

p_02 = (p_01 + R_01*p_12);
p_02 = p_02(:,end);

euler = rotm2eul(R_02);
euler = euler(1,3);

end

