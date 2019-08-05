
d2r = pi/180;
% Connect States to variables
XYZ = [0 0 1000]; VV = 10; AZ = 0; EL = -3*d2r; BA = 0;

% Connect Inputs to variables
d2r = pi/180;
Alfa = 4*d2r; Beta = 0*d2r;

% Wind-relative frame input variables
xw_hat_1 = 2;   % x_wind (m/s)
xw_hat_2 = 0;   % y_wind (m/s)
xw_hat_3 = 0;   % z_wind (m/s)

Mass = 2.25;
BA_w = 0;

% Inertial Frame Wind angles
EL_w = asin(-xw_hat_3);                                           % Elevation Angle
AZ_w = sign((xw_hat_2)/(cos(EL_w)))*acos((xw_hat_1)/(cos(EL_w))); % Azimuth Angle

% Set rotations all about the original inertial frame to the velocity frame
cb = cos(BA); sb = sin(BA);
ce = cos(EL); se = sin(EL);
ca = cos(AZ); sa = sin(AZ);
Rvi = [ca -sa 0;sa ca 0;0 0 1]*[ce 0 se;0 1 0;-se 0 ce]*[1 0 0;0 cb -sb;0 sb cb];

% Set rotations all about the velocity frame to the vehicle body axes frame
c_alfa = cos(Alfa); s_alfa = sin(Alfa);
c_beta = cos(Beta); s_beta = sin(Beta);
Rbv = [c_alfa 0 s_alfa;0 1 0; -s_alfa 0 c_alfa]*[c_beta s_beta 0;-s_beta c_beta 0;0 0 1];

% Set rotations all about the original inertial frame to the wind-relative frame
ce_w = cos(EL_w); se_w = sin(EL_w);
ca_w = cos(AZ_w); sa_w = sin(AZ_w); 
cb_w = cos(BA_w); sb_w = sin(BA_w);
Rwi_12_inv = [ce_w 0 -se_w;0 1 0; se_w 0 ce_w]*[ca_w sa_w 0;-sa_w ca_w 0;0 0 1]; % Rwi equation (36) contains non inverse matrices. The firtst two matrices' inverse used for eq (41)

% Calculate the rest of the wind-relative angles
M = Rwi_12_inv*Rvi*Rbv;
Alfa_w = asin(M(1,3));
BA_w = sign((-M(2,3))/(cos(Alfa_w)))*acos((M(3,3))/(cos(Alfa_w)));
Beta_w = sign((M(1,2))/(cos(Alfa_w)))*acos((M(1,1))/(cos(Alfa_w)));

% Velocity with respect to moving airmass
VV = VV - sqrt((xw_hat_2).^2 + (xw_hat_1).^2 + (xw_hat_3).^2 );

% Aerodynamic forces and coefficients of small glider (l = 2.6m, 
% AR = 11.08, Sw = 0.6106m^2) 
rho = 1.225-1.134e-4*XYZ(3);    % Air density (sea level = 1.225: Zi = 0)
Sw = 0.6106;                    % Wing area
q0 = 0.5*rho*VV*VV*Sw;
CL = 4.883*(Alfa_w + 0.0436);
CC = 0.303*Beta_w;
CD = 0.016 + 0.05*(CL-0.4)^2 + CL^2/33.05 + 0.8485*CC^2;
D_w = q0*CD;
C_w = q0*CC;
L_w = q0*CL;

% Move Forces from wind-relative fram through to inertial
% frame to the final velocity frame
Fa_w= [-D_w; -C_w; -L_w];
Rwi = [ca_w -sa_w 0;sa_w ca_w 0;0 0 1]*[ce_w 0 se_w;0 1 0;-se_w 0 ce_w]*[1 0 0;0 cb_w -sb_w;0 sb_w cb_w];
Fav = Rvi\(Rwi*Fa_w);

D = Fav(1);
C = Fav(2);
L = Fav(3);

Vtrim = sqrt((Mass*9.81)/(0.5*rho*Sw*CL));
XYZdot = VV*[cos(AZ)*cos(EL);sin(AZ)*cos(EL);-sin(EL)];
VVdot = -D/Mass - 9.81*sin(EL);
AZdot = (L*sin(BA) - C*cos(BA))/(Mass*VV*cos(EL));
ELdot = (L*cos(BA) + C*sin(BA))/(Mass*VV) - 9.81*cos(EL)/VV;
% Derivatives to integrate
sys = [XYZdot;VVdot;AZdot;ELdot]