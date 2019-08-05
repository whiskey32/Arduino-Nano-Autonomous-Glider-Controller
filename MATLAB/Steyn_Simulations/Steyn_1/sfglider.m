function [sys,x0,str,ts] = sfglider(t,x,u,flag,Vo,ho,mo,AEBo)
%  AIRSHIP - Full trajectory dynamics of a small glider
%  WH Steyn - 31/07/19
%  Vo   = Initial velocity
%  ho   = Initial height
%  mo   = Glider mass
%  AEBo = Initial Az, El, Bank angle vector
%  State:  x = [Xi,Yi,Zi,V,Az,El]
%  Input:  u = [Alfa,Beta] = AoA and Sideslip angles (deg)]
%  Output: y = [x,Vtrim]

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(Vo,ho,mo,AEBo);

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
function [sys,x0,str,ts]=mdlInitializeSizes(Vo,ho,mo,AEBo)

sizes = simsizes;
sizes.NumContStates  = 6;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 7;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);

% Initialise the state vector
global VV Rvi XYZ AZ EL BA Mass Vtrim
VV = Vo;            % Initial velocity magnitude
XYZ = zeros(3,1);   % Initial position vector = [0,0,0]
XYZ(3) = -ho;
Mass = mo;          % Glider mass
d2r = pi/180;
AZ = AEBo(1)*d2r; EL = AEBo(2)*d2r; BA = AEBo(3)*d2r;   % Initial AEB angles
cb = cos(BA); sb = sin(BA);
ce = cos(EL); sp = sin(EL);
ca = cos(AZ); sr = sin(AZ);                             % Initial Euler321 rotation matrix
Rvi = [cb*ce sb*ce -sp;cb*sp*sr-sb*ca sb*sp*sr+cb*ca ce*sr;cb*sp*ca+sb*sr sb*sp*ca-cb*sr ce*ca];
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

global VV Rvi XYZ AZ EL BA Mass Vtrim
XYZ = x(1:3); VV = x(4); AZ = x(5); EL = x(6);
d2r = pi/180;

% Aerodynamic forces and coefficients of small glider (l = 2.6m, 
% AR = 11.08, Sw = 0.6106m^2) 
rho = 1.225-1.134e-4*XYZ(3);    % Air density (sea level = 1.225: Zi = 0)
Sw = 0.6106;                    % Wing area
q0 = 0.5*rho*VV*VV*Sw;
Alfa = u(1)*d2r; Beta = u(2)*d2r;
CL = 4.883*(Alfa + 0.0436);
CC = 0.303*Beta;
CD = 0.016 + 0.05*(CL-0.4)^2 + CL^2/33.05 + 0.8485*CC^2;
D = q0*CD;
C = q0*CC;
L = q0*CL;
Vtrim = sqrt((Mass*9.81)/(0.5*rho*Sw*CL));
XYZdot = VV*[cos(AZ)*cos(EL);sin(AZ)*cos(EL);-sin(EL)];
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
global Vtrim

sys = [x;Vtrim];

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
