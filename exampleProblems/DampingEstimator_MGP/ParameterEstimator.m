function [problem,guess] = DoubleIntegratorTracking
%DoubleIntergratorTracking - Double Integrator Tracking Problem
%
% Syntax:  [problem,guess] = DoubleIntergratorTracking
%
% Outputs:
%    problem - Structure with information on the optimal control problem
%    guess   - Guess for state, control and multipliers.
%
% Other m-files required: none
% MAT-files required: none
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk

%------------- BEGIN CODE --------------

% Load Track data
filename = '\\Barf1.com\prf\DLS\PreEvent\2023\04_30Bak\E_6_H4A_AeroHysteresis\Output\H4A_RRH_RHSU_Race\H4A_RRH_RHSU_Race_0087.mat';
trackData = load(filename);

% Choose channels from track data 
% HF content
auxData.HF.tLap = trackData.dsResult.dsControl.dsV7P_out.data.inputs.T;
auxData.P.ms = mean(trackData.dsResult.dsState.chassis.sprung.mass_R);
auxData.P.mu = mean(trackData.dsResult.dsState.chassis.total.mass_R) - mean(trackData.dsResult.dsState.chassis.sprung.mass_R);
auxData.P.ct = 70e3;
auxData.P.kt = 400e3;
dT = diff(auxData.HF.tLap);
auxData.HF.dT = dT;
auxData.HF.zRL = trackData.dsResult.dsControl.dsV7P_out.data.inputs.moveRLx;
auxData.HF.zRR = trackData.dsResult.dsControl.dsV7P_out.data.inputs.moveRRx;
auxData.HF.zRHeave = (auxData.HF.zRL + auxData.HF.zRR)/2;
dzRHeave = diff(auxData.HF.zRHeave);
auxData.HF.zRHeaveDot = [0; dzRHeave./dT];
auxData.HF.zRHeaveDDot = [0; diff(auxData.HF.zRHeaveDot)./dT];
auxData.HF.gHubRL = trackData.dsResult.dsControl.dsV7P_out.data.outputs.car.rearSuspension.left.wheelHub.gUprightVert;
auxData.HF.gHubRR = trackData.dsResult.dsControl.dsV7P_out.data.outputs.car.rearSuspension.right.wheelHub.gUprightVert;
auxData.HF.gHubRear = (auxData.HF.gHubRL+auxData.HF.gHubRR)/2;
auxData.HF.RRH = trackData.dsResult.dsControl.dsV7P_out.data.outputs.car.chassis.rideHeight.hRideR;
auxData.HF.RRHDot = [0; diff(auxData.HF.RRH)./dT];
auxData.HF.c = trackData.dsResult.dsControl.dsV7P_out.data.outputs.car.rearSuspension.heaveDamper.F./trackData.dsResult.dsControl.dsV7P_out.data.outputs.car.rearSuspension.heaveDamper.vCompression;
auxData.HF.vDamper = trackData.dsResult.dsControl.dsV7P_out.data.outputs.car.rearSuspension.heaveDamper.vCompression;
auxData.HF.kStiff = trackData.dsResult.dsControl.dsV7P_out.data.outputs.car.rearSuspension.heaveSpring.kStiff;
auxData.HF.vCar = trackData.dsResult.dsControl.dsV7P_out.data.inputs.vCar;
fPass = 50; %50Hz lowpass freq
fields = fieldnames(auxData.HF);
tVec = 0:0.02:auxData.HF.tLap(end);
for iField = 1:length(fields)
    if strcmp(fields{iField}, 'tLap') || strcmp(fields{iField}, 'dT')
    else
        dataLP = lowpass(auxData.HF.(fields{iField}), fPass, 1000);
        auxData.HF.(fields{iField}) = interp1(auxData.HF.tLap, dataLP, tVec, 'linear', 'extrap');
    end
end
auxData.HF.tLap = tVec;

% Finding straight conditions
[~, n]=max(auxData.HF.vCar);
auxData.HF.interval = n-500:n-100;
auxData.HF.intervalTime = 0:1/fPass:(length(auxData.HF.interval)/fPass-1/fPass);
auxData.HF.zRHeaveInterval = auxData.HF.zRHeave(auxData.HF.interval);
auxData.HF.zRHeaveDotInterval = auxData.HF.zRHeaveDot(auxData.HF.interval);
auxData.HF.RRHInterval = auxData.HF.RRH(auxData.HF.interval)-mean(auxData.HF.RRH(auxData.HF.interval));
auxData.HF.RRHDotInterval = auxData.HF.RRHDot(auxData.HF.interval);

% LF content
auxData.LF.tLap = interp1(trackData.dsResult.dsState.time, trackData.dsResult.dsState.time, auxData.HF.tLap, 'linear', 'extrap');
auxData.LF.tLapInterval = auxData.LF.tLap(auxData.HF.interval);
auxData.LF.FzRear =  interp1(trackData.dsResult.dsState.time, trackData.dsResult.dsState.aero.body.Fzr, auxData.HF.tLap, 'linear', 'extrap');
auxData.LF.FzRearInterval = auxData.LF.FzRear(auxData.HF.interval);
auxData.LF.zRearHub = interp1(trackData.dsResult.dsState.time, (trackData.dsResult.dsState.wheel.RL.z + trackData.dsResult.dsState.wheel.RR.z)/2, auxData.HF.tLap, 'linear', 'extrap');
auxData.LF.zRearHubInterval = auxData.LF.zRearHub(auxData.HF.interval) - mean(auxData.LF.zRearHub(auxData.HF.interval));
auxData.LF.zDotRearHub = interp1(trackData.dsResult.dsState.time, (trackData.dsResult.dsState.wheel.RL.vz + trackData.dsResult.dsState.wheel.RR.vz)/2, auxData.HF.tLap, 'linear', 'extrap');
auxData.LF.zDotRearHubInterval = auxData.LF.zDotRearHub(auxData.HF.interval);
% Plant model name, used for Adigator
InternalDynamics=@Dynamics_Internal;
SimDynamics=@Dynamics_Sim;

% Analytic derivative files (optional)
problem.analyticDeriv.gradCost=[];
problem.analyticDeriv.hessianLagrangian=[];
problem.analyticDeriv.jacConst=[];

% Settings file
problem.settings=@settings_Optimization;

%Initial Time. t0<tf
problem.time.t0_min=0;
problem.time.t0_max=0;
guess.t0=0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=auxData.HF.intervalTime(end);     
problem.time.tf_max=auxData.HF.intervalTime(end); 
guess.tf=auxData.HF.intervalTime(end);

%% Parameters - Detailed Model
% Parameters bounds. pl=< p <=pu
% p(1) = ms;
% p(2) = mu;
% p(3) = kt;
% p(4) = ct;
% p(5) = Ps;
% p(6) = Pt;
% p(7) = Pa;
% problem.parameters.pl=[0, 0, 0, 0];
% problem.parameters.pu=[Inf, Inf, Inf, Inf];
% guess.parameters=[400, 30, 300e3, 400];

% p(3) = kt;
% p(4) = ct;
% problem.parameters.pl=[0, 0];
% problem.parameters.pu=[Inf, Inf];
% guess.parameters=[300e5, 400e3];

problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];
%% Parameters - Simple Model
% % Parameters bounds. pl=< p <=pu SIMPLE MODEL
% % p(1) = ms;
% 
% problem.parameters.pl=[0];
% problem.parameters.pu=[700];
% guess.parameters=[400];

%% States - Detailed model
% Initial conditions for system.
problem.states.x0=[auxData.HF.RRHInterval(1)-auxData.HF.zRHeaveInterval(1), auxData.HF.zRHeaveDot(auxData.HF.interval(1)), auxData.LF.zRearHubInterval(1)-auxData.HF.zRHeave(auxData.HF.interval(1)), auxData.LF.zDotRearHub(auxData.HF.interval(1))];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=[-10 -10, -10, -10];
problem.states.x0u=[10, 10, 10, 10];  

% State bounds. xl=< x <=xu
problem.states.xl=[-10 -10, -10, -10];
problem.states.xu=[10, 10, 10, 10]; 

% State error bounds
problem.states.xErrorTol_local=[1e-3 1e-3 1e-3 1e-3];
problem.states.xErrorTol_integral=[1e-3 1e-3 1e-3 1e-3];


% State constraint error bounds
problem.states.xConstraintTol=[1e-3 1e-3 1e-3 1e-3];

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=[auxData.HF.RRHInterval(end)-auxData.HF.zRHeaveInterval(end), auxData.HF.zRHeaveDot(auxData.HF.interval(end)), auxData.LF.zRearHubInterval(end)-auxData.HF.zRHeave(auxData.HF.interval(end)), auxData.LF.zDotRearHub(auxData.HF.interval(end))];
problem.states.xfu= [auxData.HF.RRHInterval(end)-auxData.HF.zRHeaveInterval(end), auxData.HF.zRHeaveDot(auxData.HF.interval(end)), auxData.LF.zRearHubInterval(end)-auxData.HF.zRHeave(auxData.HF.interval(end)), auxData.LF.zDotRearHub(auxData.HF.interval(end))];

% Guesses using HF data
x1Guess = auxData.HF.RRHInterval - auxData.HF.zRHeaveInterval;
x2Guess = auxData.HF.RRHDot(auxData.HF.interval);
x3Guess = auxData.LF.zRearHubInterval - auxData.HF.zRHeaveInterval;
x4Guess = auxData.LF.zDotRearHubInterval;

guess.time= auxData.HF.intervalTime;
guess.states(:,1)=x1Guess;
guess.states(:,2)=x2Guess;
guess.states(:,3)=x3Guess;
guess.states(:,4)=x4Guess;

%% States - Simple model
% % Initial conditions for system.
% problem.states.x0=[auxData.HF.RRH(auxData.HF.interval(1))-auxData.HF.zRHeave(auxData.HF.interval(1)), auxData.HF.zRHeaveDot(auxData.HF.interval(1))];
% 
% % Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
% problem.states.x0l=[0 -1]; 
% problem.states.x0u=[1, 1]; 
% 
% % State bounds. xl=< x <=xu
% problem.states.xl=[0 -1];
% problem.states.xu=[1, 1]; 
% 
% % State error bounds
% problem.states.xErrorTol_local=[1e-3 1e-3];
% problem.states.xErrorTol_integral=[1e-3 1e-3];
% 
% 
% % State constraint error bounds
% problem.states.xConstraintTol=[1e-3 1e-3];
% 
% % Terminal state bounds. xfl=< xf <=xfu
% problem.states.xfl=[auxData.HF.RRH(auxData.HF.interval(end))-auxData.HF.zRHeave(auxData.HF.interval(end)), auxData.HF.zRHeaveDot(auxData.HF.interval(end))]-0.01;
% problem.states.xfu= [auxData.HF.RRH(auxData.HF.interval(end))-auxData.HF.zRHeave(auxData.HF.interval(end)), auxData.HF.zRHeaveDot(auxData.HF.interval(end))]+0.01;
% 
% % Guesses using HF data
% x1Guess = auxData.HF.RRH(auxData.HF.interval) - auxData.HF.zRHeave(auxData.HF.interval);
% x2Guess = auxData.HF.RRHDot(auxData.HF.interval);
% 
% guess.time= auxData.HF.tLap(auxData.HF.interval);
% guess.states(:,1)=x1Guess;
% guess.states(:,2)=x2Guess;

%% Residual Error
% Residual Error Scale
% problem.states.ResErrorScale
% problem.states.resCusWeight

%% CONTROL - DETAILED MODEL
% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N=0;       
      
% Input bounds
% u1 - ks
% u2 - ka;
% u3 - ca;
% u4 - cs;
problem.inputs.ul=[0, -inf -inf 0];
problem.inputs.uu=[inf inf inf inf];

% Bounds on the first control action
problem.inputs.u0l=[0 -inf -inf, 0];
problem.inputs.u0u=[inf inf inf, inf];

% Input rate constraint
% problem.inputs.url=[-1e4, -1e4, -1e4, -1e4];
% problem.inputs.uru=[1e4, 1e4, 1e4, 1e4]; 

% Input constraint error bounds
problem.inputs.uConstraintTol=[10, 10, 10, 10];

% Guess the input sequences with [u0 uf]
guess.inputs(:, 1) = 300e3*ones(length(auxData.HF.intervalTime), 1);
guess.inputs(:, 2) = zeros(length(auxData.HF.intervalTime), 1);
guess.inputs(:, 3) = zeros(length(auxData.HF.intervalTime), 1);
guess.inputs(:, 4) = 20e3*ones(length(auxData.HF.intervalTime), 1);

%% CONTROL - SIMPLE MODEL
% % Number of control actions N 
% % Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% % Note that the number of integration steps defined in settings.m has to be divisible 
% % by the  number of control actions N whenever it is not zero.
% problem.inputs.N=0;       
%       
% % Input bounds
% % u1 - ka;
% % u2 - ca;
% % u3 - cs;
% problem.inputs.ul=[-inf -inf];
% problem.inputs.uu=[inf inf];
% 
% % Bounds on the first control action
% problem.inputs.u0l=[-inf -inf];
% problem.inputs.u0u=[inf inf];
% 
% % Input rate constraint
% problem.inputs.url=[-1e4, -1e4];
% problem.inputs.uru=[1e4, 1e4]; 
% 
% % Input constraint error bounds
% problem.inputs.uConstraintTol=[1, 1];
% 
% % Guess the input sequences with [u0 uf]
% guess.inputs(:,1) = zeros(length(auxData.HF.intervalTime), 1);
% guess.inputs(:,2) = zeros(length(auxData.HF.intervalTime), 1);

%% Set points
% Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.ng_eq=0;
problem.constraints.gTol_eq=[];

problem.constraints.gl=[];
problem.constraints.gu=[];
problem.constraints.gTol_neq=[];

% Bounds for boundary constraints bl =< b(x0,xf,u0,uf,p,t0,tf) =< bu
problem.constraints.bl=[];
problem.constraints.bu=[];
problem.constraints.bTol=[];

% store the necessary problem parameters used in the functions
problem.data.auxData=auxData;


% Get function handles and return to Main.m
problem.data.InternalDynamics=InternalDynamics;
problem.data.functionfg=@fg;
problem.data.plantmodel = func2str(InternalDynamics);
problem.functions={@L,@E,@f,@g,@avrc,@b};
problem.sim.functions=SimDynamics;
problem.sim.inputX=[];
problem.sim.inputU=1:length(problem.inputs.ul);
problem.functions_unscaled={@L_unscaled,@E_unscaled,@f_unscaled,@g_unscaled,@avrc,@b_unscaled};
problem.data.functions_unscaled=problem.functions_unscaled;
problem.data.ng_eq=problem.constraints.ng_eq;
problem.constraintErrorTol=[problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.states.xConstraintTol,problem.states.xConstraintTol,problem.inputs.uConstraintTol,problem.inputs.uConstraintTol];

%------------- END OF CODE --------------

function stageCost=L_unscaled(x,xr,u,ur,p,t,vdat)

% L_unscaled - Returns the stage cost.
% The function must be vectorized and
% xi, ui are column vectors taken as x(:,i) and u(:,i) (i denotes the i-th
% variable)
% 
% Syntax:  stageCost = L(x,xr,u,ur,p,t,data)
%
% Inputs:
%    x  - state vector
%    xr - state reference
%    u  - input
%    ur - input reference
%    p  - parameter
%    t  - time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    stageCost - Scalar or vectorized stage cost
%
%  Remark: If the stagecost does not depend on variables it is necessary to multiply
%          the assigned value by t in order to have right vector dimesion when called for the optimization. 
%          Example: stageCost = 0*t;

%------------- BEGIN CODE --------------
dataX1 = vdat.auxData.HF.RRHInterval- vdat.auxData.HF.zRHeaveInterval;
dataX2 = vdat.auxData.HF.RRHDotInterval;
dataTime = vdat.auxData.HF.intervalTime;

sampledTrackX1 = interp1(dataTime, dataX1, t, 'linear', 'extrap');
sampledTrackX2 = interp1(dataTime, dataX2, t, 'linear', 'extrap');
e1=x(:,1)-sampledTrackX1;
e2 = x(:, 2) - sampledTrackX2;
% u1 = u(:,1);
% u2 = u(:,2);
q1 = 300;
q2 = 1;

% stageCost = e1.*e1+e2.*e2 + u1.*u1*0.0001 + u2.*u2*0.0001;
stageCost = q1*e1.*e1 + q2*e2.*e2;
%------------- END OF CODE --------------


function boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,data) 

% E_unscaled - Returns the boundary value cost
%
% Syntax:  boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,data) 
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    boundaryCost - Scalar boundary cost
%
%------------- BEGIN CODE --------------

boundaryCost= 0;

%------------- END OF CODE --------------


function bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)

% b_unscaled - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
%
% Syntax:  bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    bc - column vector containing the evaluation of the boundary function 
%
%------------- BEGIN CODE --------------
varargin=varargin{1};
bc=[];
%------------- END OF CODE --------------
% When adpative time interval add constraint on time
%------------- BEGIN CODE --------------
if length(varargin)==2
    options=varargin{1};
    t_segment=varargin{2};
    if ((strcmp(options.discretization,'hpLGR')) || (strcmp(options.discretization,'globalLGR')))  && options.adaptseg==1 
        if size(t_segment,1)>size(t_segment,2)
            bc=[bc;diff(t_segment)];
        else
            bc=[bc,diff(t_segment)];
        end
    end
end

%------------- END OF CODE --------------
