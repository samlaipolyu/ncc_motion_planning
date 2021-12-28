function [draw, tip, sim_angle] = drawAnimation(q,roi_loc)
cbb = tab20;
c = [cbb(5,:);cbb(5,:)];
k = 10;
p_00 = zeros(1,3);
R_00 = eye(3);
ax = 100;

R_0b = cell(1,50);
R_01 = cell(1,50);
R_12 = cell(1,50);
R_02 = cell(1,50);
x_0b = zeros(1);
x_02 = zeros(1);
p_02 = zeros(3,1);

f_11 = q(1);
f_12 = q(2);
f_13 = q(3);
f_21 = q(4);
f_22 = q(5);
f_23 = q(6);
d    = q(7);

L = 1:1:50; % mm

 K_A1 = 0.8*pi*(4.5^2-1.8^2)/(50*(1+0.45*((f_11 + f_12 + f_13 + f_21 + f_22 + f_23))/(0.8*pi*(4.5^2-1.8^2))));
   K_A2 = 0.8*pi*(4.5^2-1.8^2)/(50*(1+0.45*((f_21 + f_22 + f_23))/(0.8*pi*(4.5^2-1.8^2))));
 K_A = 0.8550; % K_A = EA/L = 0.8*pi*(4.5^2-1.8^2)/50 = 0.8550 N/mm

I = (pi/4)*(4.5^4-1.8^4); % 313.8175 mm^4
K_B = 5.0211;  % 0.8*313.8175/50 (EI/l=-M/theta_B) Nmmrad-1 // 0.8MPA

r = 3.15;


ds_2 = (f_21 + f_22 + f_23)/(3*K_A2);
s_2 = L + ds_2;
%ds_1 = (f_11 + f_12 + f_13) + ((f_21 + f_22 + f_23))/(3*K_A);
ds_1 = (f_11 + f_12 + f_13 + f_21 + f_22 + f_23)/(3*K_A1);
s_1 = L + ds_1;

 K_B2 = 1.0*0.8*313.8175/s_2(end);
 K_B1 = 1.0*0.8*313.8175/s_1(end);
     %  K_B2 = 0.8*313.8175/50;
    %    K_B1 = 0.8*313.8175/50;

s_1_store = s_1(end);
s_2_store = s_2(end);

theta_2 = (r*s_2_store/K_B2)*sqrt(f_21^2+f_22^2+f_23^2-f_21*f_22-f_21*f_23-f_22*f_23);
phi_2 = atan2(3*(f_22-f_23),sqrt(3)*(f_22+f_23-2*f_21));
A = s_1_store*r*sqrt(f_11^2+f_12^2+f_13^2-f_11*f_12-f_11*f_13-f_12*f_13);

theta_2 = theta_2(end);

phi_1 = atan2(3*(f_12-f_13),sqrt(3)*(f_12+f_13-2*f_11));% deg2rad(60);

Theta_1 = (A^2 - 2*cos(phi_1 - phi_2)*A*K_B2*theta_2 + K_B2^2*theta_2^2)^(1/2)/K_B1;
Phi_1 = deg2rad(180) -  atan2((A*cos(phi_1) - K_B2*theta_2*cos(phi_2))/sqrt(A^2 - 2*cos(phi_1 - phi_2)*A*K_B2*theta_2 + K_B2^2*theta_2^2), (A*sin(phi_1) - K_B2*theta_2*sin(phi_2))/sqrt(A^2 - 2*cos(phi_1 - phi_2)*A*K_B2*theta_2 + K_B2^2*theta_2^2));
theta_1 = Theta_1;
phi_1 = real(Phi_1);
theta_1 = theta_1(end);

% geometry pre-process
theta_1 = linspace(0,theta_1,50);
theta_2 = linspace(0,theta_2,50);


x_01 = (s_1/theta_1) * (1-cos(theta_1))*cos(phi_1);
y_01 = (s_1/theta_1) * (1-cos(theta_1))*sin(phi_1);
z_01 = (s_1/theta_1) * sin(theta_1) + d;
p_01 = [x_01; y_01; z_01];




draw = figure(520);
%movegui('west');


plot3(x_01,y_01,z_01,'-','LineWidth',4,'color',c(1,:));hold on
xlabel('x [mm]');ylabel('y [mm]');zlabel('z [mm]');

% plot the reference path ****************************************
% tspan = 0:2*deg2rad(1):2*pi;
% z_tip = 150*ones(1,length(tspan));
% y_tip = 40*sign((cos(tspan))).*(sin(tspan)).*cos(tspan).^2;
% x_tip = 40*sign(cos(tspan)).*cos(tspan).^2;
% plot3(x_tip,y_tip,z_tip,'r--');
% plot the reference path ****************************************

for i = 1:1:50
    R_0b{i} = rotz(rad2deg(phi_1-phi_1));
end

for i = 1:1:50
    R_01{i} = rotz(rad2deg(phi_1))*roty(rad2deg(theta_1(1,i)))*rotz(rad2deg(-phi_1));
end

x_0b = zeros(1,length(R_0b));
y_0b = zeros(1,length(R_0b));
z_0b = ones(1,length(R_0b))*d;
p_0b = [x_0b; y_0b; z_0b];

x_12 = (s_2/theta_2) * (1-cos(theta_2))*cos(phi_2);
y_12 = (s_2/theta_2) * (1-cos(theta_2))*sin(phi_2);
z_12 = (s_2/theta_2) * sin(theta_2);
p_12 = [x_12; y_12; z_12];

for i = 1:1:50
    R_12{i} = rotz(rad2deg(phi_2))*roty(rad2deg(theta_2(1,i)))*rotz(rad2deg(-phi_2));
    R_02{i} = R_01{end}*R_12{i};
end

for i = 1:1:50% i = 1:length(R_02)
    p_02(:,i) = (p_01(:,end) + R_01{end}*p_12(:,i));
end

x_02 = p_02(1,:);
y_02 = p_02(2,:);
z_02 = p_02(3,:);

plot3(x_02,y_02,z_02,'-','LineWidth',4,'color',c(2,:));hold on

% Side view
plot3(-ax*ones(1,length(x_02)),y_02,z_02,'-','LineWidth',4,'color',[0.8 0.8 0.8]);hold on
plot3(-ax*ones(1,length(x_01)),y_01,z_01,'-','LineWidth',4,'color',[0.7 0.7 0.7]);hold on
plot3(x_01,-ax*ones(1,length(x_01)),z_01,'-','LineWidth',4,'color',[0.7 0.7 0.7]);hold on
plot3(x_02,-ax*ones(1,length(x_02)),z_02,'-','LineWidth',4,'color',[0.8 0.8 0.8]); hold on

rod_x = [p_00, p_00+d*R_00(1,3)];
rod_y = [p_00, p_00+d*R_00(2,3)];
rod_z = [p_00, p_00+d*R_00(3,3)];
plot3(rod_x,rod_y,rod_z,'k-','LineWidth',4);
plot3(rod_x,-ax*ones(1,length(rod_y)),rod_z,'color',[0.6 0.6 0.6],'LineWidth',4);
plot3(-ax*ones(1,length(rod_x)),rod_y,rod_z,'color',[0.6 0.6 0.6],'LineWidth',4);

tri_x1_base = [p_00, p_00+k*R_00(1,1)];
tri_y1_base = [p_00, p_00+k*R_00(2,1)];
tri_z1_base = [p_00, p_00+k*R_00(3,1)];
tri_x2_base = [p_00, p_00+k*R_00(1,2)];
tri_y2_base = [p_00, p_00+k*R_00(2,2)];
tri_z2_base = [p_00, p_00+k*R_00(3,2)];
tri_x3_base = [p_00, p_00+k*R_00(1,3)];
tri_y3_base = [p_00, p_00+k*R_00(2,3)];
tri_z3_base = [p_00, p_00+k*R_00(3,3)];
plot3(tri_x1_base,tri_y1_base,tri_z1_base,'r-','LineWidth',1);
plot3(tri_x2_base,tri_y2_base,tri_z2_base,'g-','LineWidth',1);
plot3(tri_x3_base,tri_y3_base,tri_z3_base,'b-','LineWidth',1);

tri_x1_base_0b = [p_0b(1,i), p_0b(1,i)+k*R_0b{i}(1,1)];
tri_y1_base_0b = [p_0b(2,i), p_0b(2,i)+k*R_0b{i}(2,1)];
tri_z1_base_0b = [p_0b(3,i), p_0b(3,i)+k*R_0b{i}(3,1)];
tri_x2_base_0b = [p_0b(1,i), p_0b(1,i)+k*R_0b{i}(1,2)];
tri_y2_base_0b = [p_0b(2,i), p_0b(2,i)+k*R_0b{i}(2,2)];
tri_z2_base_0b = [p_0b(3,i), p_0b(3,i)+k*R_0b{i}(3,2)];
tri_x3_base_0b = [p_0b(1,i), p_0b(1,i)+k*R_0b{i}(1,3)];
tri_y3_base_0b = [p_0b(2,i), p_0b(2,i)+k*R_0b{i}(2,3)];
tri_z3_base_0b = [p_0b(3,i), p_0b(3,i)+k*R_0b{i}(3,3)];
plot3(tri_x1_base_0b,tri_y1_base_0b,tri_z1_base_0b,'r-','LineWidth',1);
plot3(tri_x2_base_0b,tri_y2_base_0b,tri_z2_base_0b,'g-','LineWidth',1);
plot3(tri_x3_base_0b,tri_y3_base_0b,tri_z3_base_0b,'b-','LineWidth',1);

tri_x1_mid = [p_01(1,i), p_01(1,i)+k*R_01{i}(1,1)];
tri_y1_mid = [p_01(2,i), p_01(2,i)+k*R_01{i}(2,1)];
tri_z1_mid = [p_01(3,i), p_01(3,i)+k*R_01{i}(3,1)];
tri_x2_mid = [p_01(1,i), p_01(1,i)+k*R_01{i}(1,2)];
tri_y2_mid = [p_01(2,i), p_01(2,i)+k*R_01{i}(2,2)];
tri_z2_mid = [p_01(3,i), p_01(3,i)+k*R_01{i}(3,2)];
tri_x3_mid = [p_01(1,i), p_01(1,i)+k*R_01{i}(1,3)];
tri_y3_mid = [p_01(2,i), p_01(2,i)+k*R_01{i}(2,3)];
tri_z3_mid = [p_01(3,i), p_01(3,i)+k*R_01{i}(3,3)];
plot3(tri_x1_mid,tri_y1_mid,tri_z1_mid,'r-','LineWidth',1);
plot3(tri_x2_mid,tri_y2_mid,tri_z2_mid,'g-','LineWidth',1);
plot3(tri_x3_mid,tri_y3_mid,tri_z3_mid,'b-','LineWidth',1);

tri_x1_tip = [p_02(1,i), p_02(1,i)+k*R_02{i}(1,1)];
tri_y1_tip = [p_02(2,i), p_02(2,i)+k*R_02{i}(2,1)];
tri_z1_tip = [p_02(3,i), p_02(3,i)+k*R_02{i}(3,1)];
tri_x2_tip = [p_02(1,i), p_02(1,i)+k*R_02{i}(1,2)];
tri_y2_tip = [p_02(2,i), p_02(2,i)+k*R_02{i}(2,2)];
tri_z2_tip = [p_02(3,i), p_02(3,i)+k*R_02{i}(3,2)];
tri_x3_tip = [p_02(1,i), p_02(1,i)+k*R_02{i}(1,3)];
tri_y3_tip = [p_02(2,i), p_02(2,i)+k*R_02{i}(2,3)];
tri_z3_tip = [p_02(3,i), p_02(3,i)+k*R_02{i}(3,3)];
plot3(tri_x1_tip,tri_y1_tip,tri_z1_tip,'r-','LineWidth',1);
plot3(tri_x2_tip,tri_y2_tip,tri_z2_tip,'g-','LineWidth',1);
plot3(tri_x3_tip,tri_y3_tip,tri_z3_tip,'b-','LineWidth',1);
% p_02_prev = [p_02_prev p_02(:,end)];
% plot3(p_02_prev(1,1:j),p_02_prev(2,1:j),p_02_prev(3,1:j),'-','linewidth',1,'color',[0.4940 0.1840 0.5560])
% hold on
% plot3(x_tip(1,:),y_tip(1,:),z_tip(1,:),'r-'); hold on

% tspan = 0:2*pi/180:2*pi;
% z_tip = 140*ones(1,length(tspan));
% x_tip = 40*sign((cos(tspan))).*(sin(tspan)).*cos(tspan).^2;
% y_tip = 40*sign(cos(tspan)).*cos(tspan).^2;
% viaPoint = [x_tip; y_tip; z_tip];
% plot3(x_tip,y_tip,z_tip,'r-');

%plot3(-10,-20,140,'r.');

sim_angle = rotm2eul(R_02{end});

tip = p_02(:,50);
scatter3(roi_loc(:,1),roi_loc(:,2),roi_loc(:,3),6,roi_loc(:,3),'filled')
colormap(viridis);
axis equal
box on
xlim([-ax ax]);
ylim([-ax ax]);
zlim([0 250]);
set(gca,'zdir','reverse')
camproj('perspective')
view([153 33]);
%view([20 60]);
%view([90 90])
drawnow;
%hold off


% subplot(1,2,2)
% plot3(x_01,y_01,z_01,'-','LineWidth',4,'color',[0.0265 0.6137 0.8135]);hold on
% plot3(x_02,y_02,z_02,'-','LineWidth',4,'color',[0.9856 0.7372 0.2537]);hold on
% plot3(rod_x,rod_y,rod_z,'k-','LineWidth',4);hold on
% plot3(tri_x1_tip,tri_y1_tip,tri_z1_tip,'r-','LineWidth',2);
% plot3(tri_x2_tip,tri_y2_tip,tri_z2_tip,'g-','LineWidth',2);
% plot3(tri_x3_tip,tri_y3_tip,tri_z3_tip,'b-','LineWidth',2);
% % plot3(x_tip,y_tip,z_tip,'r--');
% axis equal
% xlim([-ax ax]);
% ylim([-ax ax]);
% zlim([0 180]);
% 
% camproj('perspective')
% view([0 80]);
% box on
% hold off



%     figure(2)
%
%     if j > 1
%         subplot(4,2,2)
%         xlim([0 length(q)])
%         t = toc;
%         title('tip err [mm]')
%         line([j-1,j],[x_tip(1,j-1)-p_02_prev(1,end-1),x_tip(1,j)-x_02(1,end)],'Color','r','LineWidth',1); hold on
%
%         subplot(4,2,4)
%         xlim([0 length(q)]);
%         line([j-1,j],[y_tip(1,j-1)-p_02_prev(2,end-1),y_tip(1,j)-y_02(1,end)],'Color','r','LineWidth',1); hold on
%         subplot(4,2,6)
%         xlim([0 length(q)]);
%         line([j-1,j],[z_tip(1,j-1)-p_02_prev(3,end-1),z_tip(1,j)-z_02(1,end)],'Color','r','LineWidth',1); hold on
%
%         subplot(4,2,8)
%         xlim([0 length(q)])
%         line([j-1 j],[abs(sqrt(x_tip(1,j-1)^2+y_tip(1,j-1)^2+z_tip(1,j-1)^2)-norm(p_02_prev(:,end-1))),...
%             abs(sqrt(x_tip(1,j).^2+y_tip(1,j)^2+z_tip(1,j)^2)-norm(p_02_prev(:,end)))],'Color','b','LineWidth',1); hold on
%         xlabel('step')
%         set(gcf,'color','w');
%     end
%     j = j + 1;
set(gcf,'color','w');


end



