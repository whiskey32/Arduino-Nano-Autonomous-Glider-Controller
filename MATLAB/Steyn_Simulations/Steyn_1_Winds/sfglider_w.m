function [sys,x0,str,ts] = sfglider_w(t,x,u,flag,Vo,ho,mo,AEo)
%  AIRSHIP - Full trajectory dynamics of a small glider with wind. 
%  WH Steyn - 31/07/19
%  BJGW du Plessis - 05/08/19
%  Vo   = Initial velocity
%  ho   = Initial height
%  mo   = Glider mass
%  AEo = Initial Az and El angle vector
%  State:  x = [XYZ;VV;AZ;EL];
%  Input:  u = [Alfa,Beta,BA,w_x, w_y, w_z] = [AoA,Sideslip and Bank angles (deg), Wind-relative frame inputs (m/s)]
%  Output: y = [x]

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %s
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(Vo,ho,mo,AEo);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys = [];

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

% end sfairship

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(Vo,ho,mo,AEo)

sizes = simsizes;
sizes.NumContStates  = 6;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);

% Initialise the state vector
global VV XYZ AZ EL Mass Vtrim 

VV = Vo;            % Initial velocity magnitude
XYZ = zeros(3,1);   % Initial position vector = [0,0,0]
XYZ(3) = -ho;
Mass = mo;          % Glider mass

d2r = pi/180;
AZ = AEo(1)*d2r; EL = AEo(2)*d2r;                % Initial AEB angles

x0  = [XYZ;VV;AZ;EL];
Vtrim = VV;
str = [];       % Empty string
ts  = [0 0];    % Continuous time

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

global VV Rvi XYZ AZ EL BA Mass 

% Connect States to variables
XYZ = x(1:3); VV = x(4); AZ = x(5); EL = x(6);

% Connect Inputs to variables
d2r = pi/180;
Alfa = u(1)*d2r; Beta = u(2)*d2r; BA = u(3)*d2r;

% Wind-relative frame input variables
w_x = u(4);   % x_wind (m/s)
w_y = u(5);   % y_wind (m/s)
w_z = u(6);   % z_wind (m/s)

% Velocity with respect to moving airmass
XYZdot = VV*[cos(AZ)*cos(EL);sin(AZ)*cos(EL);-sin(EL)];

VV_w_x = XYZdot(1) - w_x;
VV_w_y = XYZdot(2) - w_y;
VV_w_z = XYZdot(3) - w_z;
VV_w_abs = sqrt(VV_w_x.^2 + VV_w_y.^2 + VV_w_z.^2);

xw_hat_1 = VV_w_x/(VV_w_abs);
xw_hat_2 = VV_w_y/(VV_w_abs);
xw_hat_3 = VV_w_z/(VV_w_abs);

% Inertial Frame Wind angles
EL_w = asin(-xw_hat_3);                                           % Elevation Angle
AZ_w = sign((xw_hat_2)/(cos(EL_w)))*acos((xw_hat_1)/(cos(EL_w))); % Azimuth Angle
BA_w = 0;

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

% Aerodynamic forces and coefficients of small glider (l = 2.6m, 
% AR = 11.08, Sw = 0.6106m^2) 
rho = 1.225-1.134e-4*XYZ(3);    % Air density (sea level = 1.225: Zi = 0)
Sw = 0.6106;                    % Wing area
q0 = 0.5*rho*VV_w_abs*VV_w_abs*Sw;
CL = 4.883*(Alfa_w + 0.0436);
CC = 0.303*Beta_w;
CD = 0.016 + 0.05*(CL-0.4)^2 + CL^2/33.05 + 0.8485*CC^2;
D_w = q0*CD;
C_w = q0*CC;
L_w = q0*CL;

% Move Forces from wind-relative frame through to inertial frame
% Frame to the final velocity frame
Fa_w= [D_w; C_w; L_w];
Rwi = [ca_w -sa_w 0;sa_w ca_w 0;0 0 1]*[ce_w 0 se_w;0 1 0;-se_w 0 ce_w]*[1 0 0;0 cb_w -sb_w;0 sb_w cb_w];
Fav = Rvi\(Rwi*Fa_w);

D = Fav(1);
C = Fav(2);
L = Fav(3);

%Vtrim = sqrt((Mass*9.81)/(0.5*rho*Sw*CL));
VVdot = -D/Mass - 9.81*sin(EL);
AZdot = (L*sin(BA) - C*cos(BA))/(Mass*VV*cos(EL));
ELdot = (L*cos(BA) + C*sin(BA))/(Mass*VV) - 9.81*cos(EL)/VV;
% Derivatives to integrate
sys = [XYZdot;VVdot;AZdot;ELdot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
%global Vtrim

sys = [x];


% end mdlOutputs

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)


sys = [];

% end mdlTerminate
