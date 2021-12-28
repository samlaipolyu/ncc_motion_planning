clear
clc
tic

% 8 
tspan = 0:deg2rad(3):2*pi;
z_tip = 110*ones(1,length(tspan));
x_tip = 40*sign((cos(tspan))).*(sin(tspan)).*cos(tspan).^2;
y_tip = 40*sign(cos(tspan)).*cos(tspan).^2;
viaPoint = [x_tip; y_tip; z_tip];

% O
% tspan = 0:deg2rad(3):2*pi;
% z_tip = 110*ones(1,length(tspan));
% x_tip = 35*cos(tspan);
% y_tip = 40*sin(tspan);
% viaPoint = [x_tip; y_tip; z_tip];

% Oval%
% tspan = 0:deg2rad(360/((10/0.16))):6*pi;
% z_tip = (2*tspan+90)*.5+ 60;
% x_tip = 30*cos(tspan-2);
% y_tip = 35*sin(tspan);
% viaPoint = [x_tip; y_tip; z_tip];
% plot3(viaPoint(1,:),viaPoint(2,:),viaPoint(3,:))

% Spinning
% interval = deg2rad(5);
% tspan = 0:interval:5*pi;
% x_tip = 3*sign(cos(tspan)).*tspan.*cos(tspan).^2;
% y_tip = -3*sign(sin(tspan)).*tspan.*sin(tspan).^2;
% z_tip = 120*ones(1,length(tspan));
% viaPoint = [x_tip; y_tip; z_tip];

% 7-star 
% tspan = 0:deg2rad(16):10*pi;
% x_tip = 6*((7-5)*cos(tspan) + 6*cos((2/5)*tspan));
% y_tip = 6*((7-5)*sin(tspan) - 6*sin((2/5)*tspan));
% z_tip = 110*ones(1,length(tspan));
% viaPoint = [x_tip; y_tip; z_tip];

% Square
% interval = deg2rad(2);
% tspan = 0:interval:2*pi;
% x_tip = 20*sign(cos(tspan)).*cos(tspan).^2;
% y_tip = 20*sign(sin(tspan)).*sin(tspan).^2;
% z_tip = 110*ones(1,length(tspan));
% viaPoint = [x_tip; y_tip; z_tip];

x0 = -[0.01 0.02 0.01 0.01 0.02 0.02 -1]';
x_prev = [];
x = [];
center  = [-10;-15;95];
obj_rad = 7.5;

var = 20; % in degree
var_angle = rotz(var);
set_angle = rotm2eul( eye(3)*var_angle,'ZYZ' );

%%
for i = 1:length(viaPoint) 
    
    if isempty(x_prev)
        x_prev = x0;
    else
        x_prev = x;
    end
    
    task = viaPoint(:,i);
    if isempty(x)
        x = x_prev;
        J{i} = pinv(task/x(:,i));
    else
        J{i} = pinv(task/x(:,i-1));
    end
    
    %     [u s v] = svd(J{i});
    %     lambda(i) = max((eig(s'*s)));
    
    % Global Search algorithm
    %     gs = GlobalSearch;
    %     fun = @(x)((norm(task-fwk(x(:,end))))^2 +  (0.1)*(norm(x(1:6)-x_prev(1:6)))^2   +...
    %         norm(fwkeul_alpha(x(1:6,end)) - (set_angle(1,1)))^2 +...
    %         norm(fwkeul_beta(x(1:6,end))  - (set_angle(1,2)))^2  +...
    %         norm(fwkeul_gamma(x(1:6,end)) - (set_angle(1,3)))^2 );
    %     problem = createOptimProblem('fmincon','x0',-[0.02 0.01 0.02 0.02 0.01 0.02 -1]',...
    %         'objective',fun,'lb',-[2; 2; 2; 2; 2; 2; 20],'ub',[-eps; -eps; -eps; -eps; -eps; -eps; 60]);
    %     x(:,i) = run(gs,problem);
    
    %   fun = @(x)((task-fwk(x(:,end)))'*eye(3)*(task-fwk(x(:,end))) + 2*(norm(x(1:6)-x_prev(1:6))));
    
    fun = @(x)((norm(task-fwk(x(:,end))))^2 +  (0.1)*(norm(x(1:6)-x_prev(1:6)))^2   +...
        norm(fwkeul_alpha(x(1:6,end)) - (set_angle(1,1)))^2 +...
        norm(fwkeul_beta(x(1:6,end))  - (set_angle(1,2)))^2  +...
        norm(fwkeul_gamma(x(1:6,end)) - (set_angle(1,3)))^2 );
                   %  (0.1)*norm(manipulator(x,center,obj_rad)).^2   );

    % A = diag((1e-8)*ones(1,7));
    % b = (1e-2)*ones(7,1);
    A = [1 1 1 1 1 1 (10^-3)];
    b = 10^-2;
    ub = [-eps; -eps; -eps; -eps; -eps; -eps; 60];
    lb = -[2; 2; 2; 2; 2; 2; 20];
    
    option = optimoptions('fmincon','Display','final','Algorithm','interior-point');
    [x(:,i), fval(i)] = fmincon(fun,x0,A,b,[],[],lb,ub,[],option);   % //'interior-point-convex';
    x_prev = x(:,end);
    
end

toc
%%
figure(1)
hold on
subplot(3,3,1)
plot(x(1,:),'linewidth',1)

subplot(3,3,2)
plot(x(2,:),'linewidth',1)

subplot(3,3,3)
plot(x(3,:),'linewidth',1)

subplot(3,3,4)
plot(x(4,:),'linewidth',1)

subplot(3,3,5)
plot(x(5,:),'linewidth',1)

subplot(3,3,6)
plot(x(6,:),'linewidth',1)

subplot(3,3,[7 8 9])
plot(x(7,:),'linewidth',1)

figure(3)
subplot(2,1,1)
hold on
plot(x(1,:),'linewidth',1)
plot(x(2,:),'linewidth',1)
plot(x(3,:),'linewidth',1)

subplot(2,1,2)
hold on
plot(x(4,:),'linewidth',1)
plot(x(5,:),'linewidth',1)
plot(x(6,:),'linewidth',1)

%% animation
x_cal = x;
%%
% smoothie = 6;
% x_use(1,:) = smoothdata(x_cal(1,:),'movmedian',smoothie);
% x_use(2,:) = smoothdata(x_cal(2,:),'movmedian',smoothie);
% x_use(3,:) = smoothdata(x_cal(3,:),'movmedian',smoothie);
% x_use(4,:) = smoothdata(x_cal(4,:),'movmedian',smoothie);
% x_use(5,:) = smoothdata(x_cal(5,:),'movmedian',smoothie);
% x_use(6,:) = smoothdata(x_cal(6,:),'movmedian',smoothie);
% x_use(7,:) = smoothdata(x_cal(7,:),'movmedian',smoothie);
% 
% x_use(1,:) = smoothdata(x_use(1,:),'sgolay');
% x_use(2,:) = smoothdata(x_use(2,:),'sgolay');
% x_use(3,:) = smoothdata(x_use(3,:),'sgolay');
% x_use(4,:) = smoothdata(x_use(4,:),'sgolay');
% x_use(5,:) = smoothdata(x_use(5,:),'sgolay');
% x_use(6,:) = smoothdata(x_use(6,:),'sgolay');
% x_use(7,:) = smoothdata(x_use(7,:),'sgolay');
%%
figure(1)
% 
% subplot(3,3,1)
% hold on;
% plot(x(1,:),'--','linewidth',1);
% plot(x_use(1,:),'linewidth',1);
% 
% subplot(3,3,2)
% hold on;
% plot(x(2,:),'--','linewidth',1);
% plot(x_use(2,:),'linewidth',1);
% 
% subplot(3,3,3)
% hold on;
% plot(x(3,:),'--','linewidth',1);
% plot(x_use(3,:),'linewidth',1);
% 
% subplot(3,3,4)
% hold on;
% plot(x(4,:),'--','linewidth',1);
% plot(x_use(4,:),'linewidth',1);
% 
% subplot(3,3,5)
% hold on;
% plot(x(5,:),'--','linewidth',1);
% plot(x_use(5,:),'linewidth',1);
% 
% subplot(3,3,6)
% hold on;
% plot(x(6,:),'--','linewidth',1);
% plot(x_use(6,:),'linewidth',1);
% 
% subplot(3,3,[7 8 9])
% hold on;
% plot(x(7,:),'--','linewidth',1);
% plot(x_use(7,:),'linewidth',1);

%%
% figure(3)
% subplot(2,1,1)
% hold on
% plot(x_use(1,:),'linewidth',1)
% plot(x_use(2,:),'linewidth',1)
% plot(x_use(3,:),'linewidth',1)
% 
% subplot(2,1,2)
% hold on
% plot(x_use(4,:),'linewidth',1)
% plot(x_use(5,:),'linewidth',1)
% plot(x_use(6,:),'linewidth',1)

%%
%x = x_use;
%%
%save('x_obj_v1.mat','x')

%%
s_1_store = [];
s_2_store = [];
store_phi_1 = [];
store_phi_2 = [];
p_02_prev = [];
%
ax = 80;
tic
j = 1;
fs = figure(2);

while j <= length(x)
    tic
    for i = 1:length(x)

        drawnow;
        f_11 = x(1,i);
        f_12 = x(2,i);
        f_13 = x(3,i);
        f_21 = x(4,i);
        f_22 = x(5,i);
        f_23 = x(6,i);
        d    = x(7,i);
     
        L = 1:1:50; % mm
        % K_A = 0.8550; % K_A = EA/L = 0.8*pi*(4.5^2-1.8^2)/50 = 0.8550 N/mm
        K_A1 = 0.8*pi*(4.5^2-1.8^2)/(50*(1+0.45*((f_11 + f_12 + f_13 + f_21 + f_22 + f_23))/(0.8*pi*(4.5^2-1.8^2))));
        K_A2 = 0.8*pi*(4.5^2-1.8^2)/(50*(1+0.45*((f_21 + f_22 + f_23))/(0.8*pi*(4.5^2-1.8^2))));
        I = (pi/4)*(4.5^4-1.8^4); % 313.8175 mm^4
        % K_B = 5.0211;  % 0.8*313.8175/50 (EI/l=-M/theta_B) Nmmrad-1 // 0.8MPA
        r = 3.15;
        eta = 1;
        
        ds_2 = (f_21 + f_22 + f_23)/(3*K_A2);
        s_2 = L + ds_2;
        ds_1 = (f_11 + f_12 + f_13 + f_21 + f_22 + f_23)/(3*K_A1);
        s_1 = L + ds_1;
        
        K_B2 = 0.8*313.8175/s_2(end);
        K_B1 = 0.8*313.8175/s_1(end);

        s_1_store = s_1(end);
        s_2_store = s_2(end);

        theta_2 = eta*(r*s_2_store/K_B2)*sqrt(f_21^2+f_22^2+f_23^2-f_21*f_22-f_21*f_23-f_22*f_23);
        phi_2 = atan2(3*(f_22-f_23),sqrt(3)*(f_22+f_23-2*f_21));
        A = eta*s_1_store*r*sqrt(f_11^2+f_12^2+f_13^2-f_11*f_12-f_11*f_13-f_12*f_13);
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
        
        % geometry pre-process
        theta_1 = 0:theta_1/(length(L)-1):theta_1;
        theta_2 = 0:theta_2/(length(L)-1):theta_2;
        
        x_01 = (s_1/theta_1) * (1-cos(theta_1))*cos(phi_1);
        y_01 = (s_1/theta_1) * (1-cos(theta_1))*sin(phi_1);
        z_01 = (s_1/theta_1) * sin(theta_1) + d;
        p_01 = [x_01; y_01; z_01];

        subplot(4,2,[1 3 5 7])
        
        plot3(x_01,y_01,z_01,'-','LineWidth',4,'color',[0.0265 0.6137 0.8135]);hold on
        xlabel('x [mm]');ylabel('y [mm]');zlabel('z [mm]');
        
        for i = 1:length(theta_1)
            R_0b{i} = rotz(rad2deg(phi_1-phi_1));
        end
        
        for i = 1:length(theta_1)
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
        
        for i = 1:length(theta_2)
            R_12{i} = rotz(rad2deg(phi_2))*roty(rad2deg(theta_2(1,i)))*rotz(rad2deg(-phi_2));
            R_02{i} = R_01{end}*R_12{i};
        end
        
        sim_angle(:,j) = rotm2eul(R_02{1,end},'ZYX');
        
        for i = 1:length(R_02)
            p_02(:,i) = (p_01(:,end) + R_01{end}*p_12(:,i));
        end
        
        x_02 = p_02(1,:);
        y_02 = p_02(2,:);
        z_02 = p_02(3,:);
        
        plot3(x_02,y_02,z_02,'-','LineWidth',4,'color',[0.9856 0.7372 0.2537]);hold on
        
        plot3(-ax*ones(1,length(x_02)),y_02,z_02,'-','LineWidth',4,'color',[0.8 0.8 0.8]);hold on
        plot3(-ax*ones(1,length(x_01)),y_01,z_01,'-','LineWidth',4,'color',[0.7 0.7 0.7]);hold on
        plot3(x_01,ax*ones(1,length(x_01)),z_01,'-','LineWidth',4,'color',[0.7 0.7 0.7]);hold on
        plot3(x_02,ax*ones(1,length(x_02)),z_02,'-','LineWidth',4,'color',[0.8 0.8 0.8]); hold on
        
        k = 10;
        p_00 = [0 0 0];
        R_00 = eye(3);
        
        rod_x = [p_00, p_00+d*R_00(1,3)];
        rod_y = [p_00, p_00+d*R_00(2,3)];
        rod_z = [p_00, p_00+d*R_00(3,3)];
        plot3(rod_x,rod_y,rod_z,'k-','LineWidth',4);
        plot3(rod_x,ax*ones(1,length(rod_y)),rod_z,'color',[0.6 0.6 0.6],'LineWidth',4);
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
        plot3(tri_x1_base,tri_y1_base,tri_z1_base,'r-','LineWidth',2);
        plot3(tri_x2_base,tri_y2_base,tri_z2_base,'g-','LineWidth',2);
        plot3(tri_x3_base,tri_y3_base,tri_z3_base,'b-','LineWidth',2);
        
        tri_x1_base_0b = [p_0b(1,i), p_0b(1,i)+k*R_0b{i}(1,1)];
        tri_y1_base_0b = [p_0b(2,i), p_0b(2,i)+k*R_0b{i}(2,1)];
        tri_z1_base_0b = [p_0b(3,i), p_0b(3,i)+k*R_0b{i}(3,1)];
        tri_x2_base_0b = [p_0b(1,i), p_0b(1,i)+k*R_0b{i}(1,2)];
        tri_y2_base_0b = [p_0b(2,i), p_0b(2,i)+k*R_0b{i}(2,2)];
        tri_z2_base_0b = [p_0b(3,i), p_0b(3,i)+k*R_0b{i}(3,2)];
        tri_x3_base_0b = [p_0b(1,i), p_0b(1,i)+k*R_0b{i}(1,3)];
        tri_y3_base_0b = [p_0b(2,i), p_0b(2,i)+k*R_0b{i}(2,3)];
        tri_z3_base_0b = [p_0b(3,i), p_0b(3,i)+k*R_0b{i}(3,3)];
        plot3(tri_x1_base_0b,tri_y1_base_0b,tri_z1_base_0b,'r-','LineWidth',2);
        plot3(tri_x2_base_0b,tri_y2_base_0b,tri_z2_base_0b,'g-','LineWidth',2);
        plot3(tri_x3_base_0b,tri_y3_base_0b,tri_z3_base_0b,'b-','LineWidth',2);
        
        tri_x1_mid = [p_01(1,i), p_01(1,i)+k*R_01{i}(1,1)];
        tri_y1_mid = [p_01(2,i), p_01(2,i)+k*R_01{i}(2,1)];
        tri_z1_mid = [p_01(3,i), p_01(3,i)+k*R_01{i}(3,1)];
        tri_x2_mid = [p_01(1,i), p_01(1,i)+k*R_01{i}(1,2)];
        tri_y2_mid = [p_01(2,i), p_01(2,i)+k*R_01{i}(2,2)];
        tri_z2_mid = [p_01(3,i), p_01(3,i)+k*R_01{i}(3,2)];
        tri_x3_mid = [p_01(1,i), p_01(1,i)+k*R_01{i}(1,3)];
        tri_y3_mid = [p_01(2,i), p_01(2,i)+k*R_01{i}(2,3)];
        tri_z3_mid = [p_01(3,i), p_01(3,i)+k*R_01{i}(3,3)];
        plot3(tri_x1_mid,tri_y1_mid,tri_z1_mid,'r-','LineWidth',2);
        plot3(tri_x2_mid,tri_y2_mid,tri_z2_mid,'g-','LineWidth',2);
        plot3(tri_x3_mid,tri_y3_mid,tri_z3_mid,'b-','LineWidth',2);
        
        tri_x1_tip = [p_02(1,i), p_02(1,i)+k*R_02{i}(1,1)];
        tri_y1_tip = [p_02(2,i), p_02(2,i)+k*R_02{i}(2,1)];
        tri_z1_tip = [p_02(3,i), p_02(3,i)+k*R_02{i}(3,1)];
        tri_x2_tip = [p_02(1,i), p_02(1,i)+k*R_02{i}(1,2)];
        tri_y2_tip = [p_02(2,i), p_02(2,i)+k*R_02{i}(2,2)];
        tri_z2_tip = [p_02(3,i), p_02(3,i)+k*R_02{i}(3,2)];
        tri_x3_tip = [p_02(1,i), p_02(1,i)+k*R_02{i}(1,3)];
        tri_y3_tip = [p_02(2,i), p_02(2,i)+k*R_02{i}(2,3)];
        tri_z3_tip = [p_02(3,i), p_02(3,i)+k*R_02{i}(3,3)];
        plot3(tri_x1_tip,tri_y1_tip,tri_z1_tip,'r-','LineWidth',2);
        plot3(tri_x2_tip,tri_y2_tip,tri_z2_tip,'g-','LineWidth',2);
        plot3(tri_x3_tip,tri_y3_tip,tri_z3_tip,'b-','LineWidth',2);
        p_02_prev = [p_02_prev p_02(:,end)];
        
        plot3(p_02_prev(1,1:j),p_02_prev(2,1:j),p_02_prev(3,1:j),'-','linewidth',1,'color',[0.4940 0.1840 0.5560])
        
        hold on
        
%         % Obstacle display
%         [xball,yball,zball] = sphere(12);
%         xball = xball*(2*obj_rad) + center(1,1);
%         yball = yball*(2*obj_rad) + center(2,1);
%         zball = zball*(2*obj_rad) + center(3,1);
%         ball = surf(xball,yball,zball);
%         set(ball, 'FaceColor', [1 0.5 0.5])
%         set(ball, 'FaceAlpha',.51,'EdgeAlpha', 0);
%         [~,min_beta] = manipulator(x(:,j),center,obj_rad);
%         minDist(j,:) = min_beta;
        
        %figure(2)
        plot3(x_tip(1,:),y_tip(1,:),z_tip(1,:),'r-'); hold on
        
        axis equal
        box on
        xlim([-ax ax]);
        ylim([-ax ax]);
        zlim([0 160]);
        camproj('perspective')
        view([50.5 31.6]);
        hold off
    
        figure(2)

        if j > 1
            subplot(4,2,2)
            xlim([0 length(x)])
            t = toc;
            title('tip err [mm]')
            line([j-1,j],[x_tip(1,j-1)-p_02_prev(1,end-1),x_tip(1,j)-x_02(1,end)],'Color','r','LineWidth',1); hold on
            
            subplot(4,2,4)
            xlim([0 length(x)]);
            line([j-1,j],[y_tip(1,j-1)-p_02_prev(2,end-1),y_tip(1,j)-y_02(1,end)],'Color','r','LineWidth',1); hold on
            subplot(4,2,6)
            xlim([0 length(x)]);
            line([j-1,j],[z_tip(1,j-1)-p_02_prev(3,end-1),z_tip(1,j)-z_02(1,end)],'Color','r','LineWidth',1); hold on
            
            subplot(4,2,8)
            xlim([0 length(x)])
            line([j-1 j],[abs(sqrt(x_tip(1,j-1)^2+y_tip(1,j-1)^2+z_tip(1,j-1)^2)-norm(p_02_prev(:,end-1))),...
                abs(sqrt(x_tip(1,j).^2+y_tip(1,j)^2+z_tip(1,j)^2)-norm(p_02_prev(:,end)))],'Color','b','LineWidth',1); hold on 
            xlabel('step')
            set(gcf,'color','w');  
        end
        j = j + 1;
        set(gcf,'color','w');

    end
end

%%
figure(7)

subplot(3,1,1)
hold on
plot(rad2deg(sim_angle(1,:)));
plot(rad2deg(set_angle(1,1)*ones(1,length(sim_angle))),'--');
xlim([0 length(x)])
%ylim([-20 20])
box on

subplot(3,1,2)
hold on
plot(rad2deg(sim_angle(2,:)));
plot(rad2deg(set_angle(1,2)*ones(1,length(sim_angle))),'--');
xlim([0 length(x)])
%ylim([-20 20])
box on

subplot(3,1,3)
hold on
plot(rad2deg(sim_angle(3,:)));
plot(rad2deg(set_angle(1,3)*ones(1,length(sim_angle))),'--');
xlim([0 length(x)])
box on

legend('\alpha','\beta','\gamma','\alpha_{desired}','\beta_{desired}','\gamma_{desired}')


%% collision detection
% 
% for j = 1:length(minDist)
%     if minDist(j) > (obj_rad + 4.5)  % safe
%       collisionDect(j) = -1;
%     elseif minDist(j) == (obj_rad + 4.5) % critical
%       collisionDect(j) = 0;
%     elseif minDist(j) < (obj_rad + 4.5) % collide
%       collisionDect(j) = 1;
%     end
% end
% 
% figure(8)
% stairs(collisionDect,'m-','linewidth',1)
% yticks([-1 0 1])
% ylim([-1.5 1.5])
% xlim([0 length(collisionDect)])



