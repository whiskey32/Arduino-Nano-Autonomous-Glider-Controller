function [sys,x0,str,ts] = sfglider6(t,x,u,flag,ho,mo,UVWo,RPYo)
%  GLIDER6 - Full 6-DOF dynamics of the Radian XL2.6 glider
%  WH Steyn - 2/08/19
%  Vo   = Initial velocity vector
%  RPYo = Initial RPY attitude vector
%  State:  x = [U,V,W,P,Q,R,N,E,D,Q1,Q2,Q3,Q4]
%  Input:  u = [Ar,Ae] = [Rudder & Elevator angle (deg)]
%  Output: y = [U,V,W,P,Q,R,N,E,D,RR,PP,YY] = [Vel, Rot_rate, Pos, RPY]

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(ho,mo,UVWo,RPYo);

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
function [sys,x0,str,ts]=mdlInitializeSizes(ho,mo,UVWo,RPYo)

sizes = simsizes;
sizes.NumContStates  = 13;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 15;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);

% Initialise the state vector
global VV WW AA NED QQ
VV = UVWo';             % Initial velocity vector
NED = zeros(3,1);       % Initial position vector = [0,0,-ho]
NED(3) = -ho;
WW  = zeros(3,1);       % Initial angular rate vector = [0,0,0]
d2r = pi/180;
RR = RPYo(1)*d2r; PP = RPYo(2)*d2r; YY = RPYo(3)*d2r;   % Initial RPY angles
cy = cos(YY); sy = sin(YY);
cp = cos(PP); sp = sin(PP);
cr = cos(RR); sr = sin(RR);                             % Initial Euler321 rotation matrix
AA = [cy*cp sy*cp -sp;cy*sp*sr-sy*cr sy*sp*sr+cy*cr cp*sr;cy*sp*cr+sy*sr sy*sp*cr-cy*sr cp*cr];
QQ = zeros(4,1);
QQ(4) = 0.5*sqrt(1.0+AA(1,1)+AA(2,2)+AA(3,3));          % Initial Quaternion  
if QQ(4) > 0.5
  QQ(1) = 0.25*(AA(2,3)-AA(3,2))/QQ(4);
  QQ(2) = 0.25*(AA(3,1)-AA(1,3))/QQ(4);
  QQ(3) = 0.25*(AA(1,2)-AA(2,1))/QQ(4);
else
  QQ(3) = 0.5*sqrt(1.0-AA(1,1)-AA(2,2)+AA(3,3));
  QQ(1) = 0.25*(AA(1,3)+AA(3,1))/QQ(3);
  QQ(2) = 0.25*(AA(2,3)+AA(3,2))/QQ(3);
  QQ(4) = 0.25*(AA(1,2)-AA(2,1))/QQ(3);
end
x0  = [VV;WW;NED;QQ];
str = [];       % Empty string
ts  = [0 0];    % Continuous time

% Initialise other simulation constants for derivative calculations
global mass Ixyz Alpha Beta Init
mass = mo;                              % Total mass of glider in kg
Ixyz = [0.54; 0.23; 0.61];              % Estimated MOI parameters of glider in kgm2
Alpha = 2.5*d2r;                        % Initial AoA = Alpha_zero
Beta = YY;                              % Initial Sideslip = Yaw(0)                              
Init = 1;

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(~,x,u)

global VV WW NED QQ AA mass Ixyz Alpha Beta Head
VV = x(1:3); WW = x(4:6); NED = x(7:9); QQ = x(10:13);
d2r = pi/180;

% Gravity force vector
FG = mass*9.81*AA(:,3);

% Aerodynamic forces and coefficients of small glider (bw = 2.6m, 
% cbar = 0.24m, AR = 11.08, Sw = 0.6106m^2) 
rho = 1.225+1.134e-4*NED(3);    % Air density (sea level = 1.225: Zi = 0)
Sw = 0.6106;                    % Wing area
Vmag = norm(VV);
q0 = 0.5*rho*Vmag*Vmag*Sw;
Alpha = atan(VV(3)/VV(1)); Beta = atan(VV(2)/norm(VV));
CL = 4.883*(Alpha + 0.0436);
CC = 0.44*Beta;
CD = 0.016 + 0.05*(CL-0.4)^2 + CL^2/33.05 + 0.59*CC^2;
D = q0*CD;
C = q0*CC;
L = q0*CL;
% Aerodynamic force vector
FA = [-D*cos(Alpha)+L*sin(Alpha);-C;-L*cos(Alpha)-D*sin(Alpha)];

XYZ = FG + FA;

% Aerodynamic moments and coefficients of small glider (bw = 2.6m, 
% cbar = 0.24m, AR = 11.08, Sw = 0.6106m^2)
bw = 2.6;
cbar = 0.24;
Vm2 = 2.0*Vmag;
Cl = -0.0331*Beta - (0.4248*WW(1) - 0.045*WW(3))*bw/Vm2 - 0.008*u(1)*d2r;  % u(1) = delR
Cm = -0.2954*Alpha - 10.281*WW(2)*cbar/Vm2 - 1.5852*u(2)*d2r;              % u(2) = delE  
Cn = 0.086*Beta - (0.0251*WW(1) + 0.125*WW(3))*bw/Vm2 - 0.1129*u(1)*d2r;
% Aerodynamic moment vector
LMN = q0*[bw*Cl;cbar*Cm;bw*Cn];

% Kinetics
UVWdot = XYZ/mass + cross(VV,WW);
PQRdot = [(LMN(1) - WW(3)*WW(2)*(Ixyz(3)-Ixyz(2)))/Ixyz(1);
          (LMN(2) - WW(1)*WW(3)*(Ixyz(1)-Ixyz(3)))/Ixyz(2);
          (LMN(3) - WW(2)*WW(1)*(Ixyz(2)-Ixyz(1)))/Ixyz(3)];
% PQRdot = zeros(3,1);
NEDdot = AA'*VV;
Head = atan2(NEDdot(2),NEDdot(1));

% Kinematics
Qdot = 0.5*[0 WW(3) -WW(2) WW(1);-WW(3) 0 WW(1) WW(2);WW(2) -WW(1) 0 WW(3);-WW(1) -WW(2) -WW(3) 0]*QQ;

% Derivatives to integrate
sys = [UVWdot;PQRdot;NEDdot;Qdot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(~,~,~)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(~,x,~)

global VV WW NED QQ AA Alpha Beta Head Init
VV = x(1:3); WW = x(4:6); NED = x(7:9); QQ = x(10:13);
AA(1,1) = QQ(1)^2 - QQ(2)^2 - QQ(3)^2 + QQ(4)^2;
AA(1,2) = 2*(QQ(1)*QQ(2) + QQ(3)*QQ(4));
AA(1,3) = 2*(QQ(1)*QQ(3) - QQ(2)*QQ(4));
AA(2,1) = 2*(QQ(1)*QQ(2) - QQ(3)*QQ(4));
AA(2,2) =-QQ(1)^2 + QQ(2)^2 - QQ(3)^2 + QQ(4)^2;
AA(2,3) = 2*(QQ(2)*QQ(3) + QQ(1)*QQ(4));
AA(3,1) = 2*(QQ(1)*QQ(3) + QQ(2)*QQ(4));
AA(3,2) = 2*(QQ(2)*QQ(3) - QQ(1)*QQ(4));
AA(3,3) =-QQ(1)^2 - QQ(2)^2 + QQ(3)^2 + QQ(4)^2;
YY = atan2(AA(1,2),AA(1,1));
PP = -asin(AA(1,3));
RR = atan2(AA(2,3),AA(3,3));
% PQRdot = zeros(3,1);
NEDdot = AA'*VV;
Head = atan2(NEDdot(2),NEDdot(1));

r2d = 180/pi;
if abs(VV(3)) > abs(VV(1))
    Alpha = sign(VV(3))*pi/4;               % Limit Alpha to +/-45 deg
else
    if abs(VV(1)) > 0.0
        Alpha = atan(VV(3)/VV(1));
    else
        Alpha = 0.0;
    end
end
if norm(VV) < 1                             % Zero Beta when velocity < 1 m/s
    Beta = 0.0;
else
    Beta = atan(VV(2)/norm(VV));
end

if Init == 1
    sys = zeros(15,1);
    Init = 0;
else
    sys = [VV;WW;NED;r2d*RR;r2d*PP;r2d*YY;r2d*Alpha;r2d*Beta;r2d*Head];
end

% end mdlOutputs

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(~,~,~)

sys = [];

% end mdlTerminate
