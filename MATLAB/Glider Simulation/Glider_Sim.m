%  Dynamic Trajectory Control of Glider
% 
%  This code simulates the dynamic trajectory control of a glider. The
%  aerodynamic coefficient model is similar to that of the simulation-based
%  performance study of R/C model sailplanes by Beron-Rawdon. The model
%  here uses values of parameters from the Beron-Rawdon's paper and
%  formulas for the construction of aerodynamic coefficients include most
%  of the constitutive elements of that model. The drag and lift forces are
%  functions of angle of attack and certain design parameters. 
%  A new dynamic control alogorithm from the Rui Dilao paper runs
%  iteratively and the approach to the target point is self-correcting.
%  
%  
% // Created: BJGW DU PLESSIS
% // Student Number: 18989780 
% // Modified: 2019/07/30
% // Version: 0.1


%/////////////////////////////////////////////////////////////////////////
%// Aerodynamic Force Coefficient Parameters
%/////////////////////////////////////////////////////////////////////////

% Glider Lift Coefficient Variables
a_0 = 0.1 * (180/(pi));      % Lift Curve Slope
e = 0.95;                    % Oswald Efficiency Factor
AR = 11.08;                  % Aspect Ratio
alpha_0 = -2.5 * (pi/(180)); % Zero Point

% Glider Drag Coefficient Variables
l = 102.4;                   % Wingspan Length
l_t = 0.28 * l;              % Fuselage Moment Arm Length
V_H = 0.4;                   % Horizontal Tail Volume Ratio
V_V = 0.02;                  % Vertical Tail Volume Ratio
c_bar = (1.03*l)/(AR);       % Mean Aerodynamic Chord
CD_F = 0.008;                % Fuselage Drag Coefficient
CD_T = 0.01;                 % Tail Drag Coefficient
CD_E = 0.002;                % Miscellaneous "Extra Drag" Coefficient
S_F = 216;                   % Fuselage Area
S = (l^2)/(AR);              % Wing Surface Area 
S_T = (V_H*c_bar*S)/(l_t);   % Horizontal Tail Surface Area
S_V = (V_V*l*S)/(l_t);       % Vertical Tail Surface Area
Cd_0 = 0.01;                 % Wing Profile Drag Variable 1
Cd_L = 0.05;                 % Wing Profile Drag Variable 2
CL_min = 0.4;                % Wing Profile Drag Variable 3


%/////////////////////////////////////////////////////////////////////////
%// Force Formulas to Construct Aerodynamic Coefficients 
%/////////////////////////////////////////////////////////////////////////

% Glider Lift Formula
 CL_alpha = (a_0)/(1+(a_0)/(pi*e*AR));                          % Finite Wing Slope

 
 d_alpha= 0.01;
 alpha = -45:d_alpha:45;  % Plot in radians up until 45 degrees

 % Drag Coefficient function of alpha
 CD_0 = (CD_F*S_F)/(S) + (CD_T*(S_T + S_V))/(S) + CD_E + Cd_0;  % Constant Part of Total Drag Formula
 
 CD = CD_0 + Cd_L*( LC(alpha*(pi)/(180),CL_alpha,alpha_0) - CL_min).^2 + ((LC(alpha*(pi)/(180),CL_alpha,alpha_0)).^2)/(pi*e*AR);
  
 CL_div_CD = LC(alpha*(pi)/(180),CL_alpha,alpha_0)./CD;

%/////////////////////////////////////////////////////////////////////////
%// Plot Graphs
%/////////////////////////////////////////////////////////////////////////


 
% figure(1)
% plot(alpha,LC(alpha*(pi)/(180),CL_alpha,alpha_0));
% xlabel('alpha (deg)');
% ylabel('CL');
% figure(2)
% plot(alpha,CD);
% xlabel('alpha (deg)');
% ylabel('CD');
% figure(3)
 plot(alpha,CL_div_CD);
 grid on
 title('CL/CD Ratio vs Angle of Attack ');
 xlabel('alpha (deg)');
 ylabel('CL/CD');

%/////////////////////////////////////////////////////////////////////////
%// Functions
%/////////////////////////////////////////////////////////////////////////

% Lift Coefficient which is a function of alpha
function CL = LC(alpha,CL_alpha,alpha_0)
    CL = CL_alpha*(alpha - alpha_0);  
end


