% Plot Graphs from code
% Dynamic Trajectory Control of Glider
 
% // Created: BJGW DU PLESSIS
% // Student Number: 18989780 
% // Modified: 2019/08/08
% // Version: 0.1

%Initial Conditions
Mass = 2.5;  % Glider Mass (kg)
Sw = 0.6106;                    % Wing area

% Degrees to Rad
d2r = pi/(180);

% Plot CD/CL vs alpha Beta=0

% Angle of Attack input (Deg)
d_alpha= 0.01;
Alpha = -45:d_alpha:45;

% Altitude (m)
d_alt = 1;  
alt = 0:d_alt:10500;  

% Sidelsip Angle input (Deg)
d_beta= 0.01;
Beta = -45:d_beta:45;

% Beta = 0
CL1 = 4.883*(Alpha*d2r + 0.0436);
CC1 = 0.44*0;
CD1 = 0.016 + 0.05*(CL1-0.4).^2 + CL1.^2/33.05 + 0.59*CC1.^2;
CL_div_CD1 = CL1./CD1;

% Alhpa = 0
CL2 = 4.883*(0 + 0.0436);
CC2 = 0.44*Beta*d2r;
CD2 = 0.016 + 0.05*(CL2-0.4).^2 + CL2.^2/33.05 + 0.59*CC2.^2;
CL_div_CD2 = CL2./CD2;

% Atmosphere Density vs altitude
rho = 1.225+1.134e-4*-(alt);

% % L/D max alpha = 4.2 deg
% CL1_alpha = 4.883*(4.2*d2r + 0.0436);
% 
% % Atmosphere Density, @2000m
% rho_2000 = 1.225+1.134e-4*-(2000); 
% 
% % Trim speed vs alpha, @rho(2000m)
% Vtrim_alpha = sqrt((Mass*9.81)./(0.5*rho_2000*Sw*CL1));
% 
% % Trim speed vs rho, @L/D max alpha=4.2deg
% Vtrim_rho = sqrt((Mass*9.81)./(0.5*rho*Sw*CL1_alpha));
% 
% figure(1)
% plot(Alpha,CL_div_CD1);
% grid on
% title('CL/CD Ratio vs Angle of Attack ');
% xlabel('alpha (deg)');
% ylabel('CL/CD');
% 
% figure(2)
% plot(Beta,CL_div_CD2);
% grid on
% title('CL/CD Ratio vs Sideslip Angle ');
% xlabel('beta (deg)');
% ylabel('CL/CD');
% 
% figure(3)
% plot(Alpha,Vtrim_alpha);
% grid on
% title('Vtrim vs Angle of Attack @2000m ');
% xlabel('alpha (deg)');
% ylabel('Vtrim (m/s)');
% 
% figure(4)
% plot(alt,Vtrim_rho);
% grid on
% title('Vtrim vs Altitude @L/D max, AoA = 4.2 deg ');
% xlabel('Altitude (m))');
% ylabel('Vtrim (m/s)');

% MATLAB Estimate Transfer Function at parameters of 11.8 ms/s, 2000m, alpha= 4.2deg(@max_lift),
Vel_ol_response_data = iddata(Vel_y(:,1),DelE_u,0.1);  % Create iddata object for transfer function estimation
E_V_tf = tfest(Vel_ol_response_data,2,0)

% Convert MATLAB Estimate tf to ss
b = [536];
a = [1 0.1137 0.1385];
[A,B,C,D] = tf2ss(b,a);

b2 = [51.41];
a2 = [1 0.1128 0.1267];
[A,B,C,D] = tf2ss(b,a);

rlocus(b2,a2);



