function [problem,guess] = AeroDampingEstimator_Track_Free(bLS, trackData)
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
% filename = '\\Barf1.com\prf\DLS\PreEvent\2023\06_04Bcn\A_4_Baseline_V7P\Output\Bsl_Race\Bsl_Race_0001.mat';
% trackData = load(filename);

motionRatio = 1.0422;

% Instantaneous stiffness

% Instantaneous damping
% FDam = trackData.FDamperRC_vts;
% vDam = trackData.vDamperRC_vts/1000;
% [vDam, unId] = unique(vDam);
% FDam = FDam(unId);

% dFDam = diff(FDam);
% dvDam = diff(vDam);
% cDam = dFDam./dvDam;
% cDam = [cDam; cDam(end)];

FDam = trackData.FDamperRC_vts;
vDam = trackData.vDamperRC_vts/1000;

vDamVec = -1:0.01:1;
% cDamVec = spline(vDam, cDam, vDamVec);
% cDamLinearVec = interp1(vDam, cDam, vDamVec, 'linear');
cutOffSpeed = 0.2;
pDam = polyfit(vDam, FDam, 5);
FDamPolyVec = polyval(pDam, vDamVec);
pDamDer = polyder(pDam);
cDamPolyVec = polyval(pDamDer, vDamVec);
[~, NegSpeedIndex] = min(abs(vDamVec+cutOffSpeed));
[~, PosSpeedIndex] = min(abs(vDamVec-cutOffSpeed));
cDamCap = [cDamPolyVec(NegSpeedIndex)*ones(1, NegSpeedIndex-1), cDamPolyVec(NegSpeedIndex:PosSpeedIndex), cDamPolyVec(PosSpeedIndex)*ones(1, length(vDamVec)-PosSpeedIndex)];

auxData.P.csVec = cDamCap*motionRatio^2;
auxData.P.vDamVec = vDamVec;

% kStiffness from SVA

% xHeaveVec = trackData.dsResult.dsControl.dsPolarisSVA_out.data.car.rearSuspension.left.wheelHub.xBump;
% ksVec = trackData.dsResult.dsControl.dsPolarisSVA_out.data.car.rearSuspension.KWheelRateDynamicRL;

ksVec = trackData.KHeaveAxleR_VAP;

xHeaveVec = (trackData.xWheelSetupRR_VAP + trackData.xWheelSetupRL_VAP)/2/1000;
% ksVec = trackData.dsResult.dsControl.dsPolarisSVA_out.data.car.rearSuspension.KWheelRateDynamicRL;


auxData.P.ksVec = ksVec*motionRatio;
dx = 0.0005;
xBumpVec = 0.0035:dx:0.03;
kFit = interp1(xHeaveVec, ksVec, xBumpVec, 'spline');
auxData.P.xBumpVec = xBumpVec;
auxData.P.ksVec = kFit;

% Choose channels from track data 
% HF content
auxData.HF.tLap = (trackData.Timestamp - trackData.Timestamp(1))/1e9;
auxData.P.ms = mean((1-trackData.rCarMass_VAP).*trackData.mCarSprung_VAP);
auxData.P.mTotal = mean((1-trackData.rCarMass_VAP).*trackData.mCar_VAP);
auxData.P.mu = auxData.P.mTotal - auxData.P.ms;
auxData.P.ct = 2*70e3;
auxData.P.kt = 2*400e3;
dT = diff(auxData.HF.tLap);
auxData.HF.dT = dT;

auxData.HF.RRH = trackData.hRideR_VAP/1000;
auxData.HF.xBumpR = (trackData.xWheelSetupRL_VAP + trackData.xWheelSetupRR_VAP)/2/1000;

auxData.HF.vCar = trackData.vCar;
auxData.HF.FzRear = -trackData.FAeroAxleRWTCalc_VAP;

auxData.HF.xHubR0 = (trackData.zSetupWheelCenterRR0_PLR + trackData.zSetupWheelCenterRL0_PLR)/2/1000;
auxData.HF.RRHSetup = trackData.hRideSetupR_VAP/1000;

fPass = 20; %50Hz lowpass freq
fields = fieldnames(auxData.HF);
deltaTime = 1/100;
tVec = 0:deltaTime:auxData.HF.tLap(end);
Fs = 1/mean(dT);
for iField = 1:length(fields)
    if strcmp(fields{iField}, 'tLap') || strcmp(fields{iField}, 'dT')
    else
        dataLP = lowpass(auxData.HF.(fields{iField}), fPass, Fs);
        auxData.HF.(fields{iField}) = interp1(auxData.HF.tLap, dataLP, tVec, 'linear', 'extrap');
    end
end
auxData.HF.tLap = tVec;
fHP = 3;

auxData.HF.xSetup = auxData.HF.xHubR0 - auxData.HF.RRHSetup;

auxData.HF.dRRH = auxData.HF.RRH - auxData.HF.RRHSetup;
auxData.HF.RRH_HP = highpass(auxData.HF.RRH, fHP, 1/deltaTime);
auxData.HF.RRHDot_HP = [diff(auxData.HF.RRH_HP)/deltaTime, 0];
auxData.HF.xBumpR_HP = highpass(auxData.HF.xBumpR, fHP, 1/deltaTime);

auxData.HF.dRRHDot = [diff(auxData.HF.dRRH)/deltaTime, 0];
auxData.HF.xBumpRDot = [diff(auxData.HF.xBumpR)/deltaTime, 0];

auxData.HF.dxHubR = auxData.HF.xBumpR + auxData.HF.dRRH;
auxData.HF.dxHubRDot = [diff(auxData.HF.dxHubR)/deltaTime, 0];
auxData.HF.xHubR_HP = highpass(auxData.HF.dxHubR, fHP, 1/deltaTime);
auxData.HF.xHubRDot_HP = [diff(auxData.HF.xHubR_HP)/deltaTime, 0];

%% Finding index to look into

if bLS
    [~, n]=min(auxData.HF.vCar);
    
    if n < 250
        iStart = 50;
        iEnd = 300;
    elseif n > length(auxData.HF.vCar) - 120
        iStart = n - 250;
        iEnd = length(auxData.HF.vCar);
    else
        iStart = n-100;
        iEnd = n+100;
    end
    zr0 = 0.03;
else
    [~, n]=max(auxData.HF.vCar);
    
    if n < 550
        iStart = 50;
        iEnd = 300;
    elseif n > length(auxData.HF.vCar) - 50
        iStart = n - 400;
        iEnd = length(auxData.HF.vCar);
    else
        iStart = n-250;
        iEnd = n;
    end
    zr0 = -0.01;
end
auxData.HF.interval = iStart:iEnd;
auxData.HF.intervalTime = 0:deltaTime:(length(auxData.HF.interval)*deltaTime-deltaTime);
% auxData.HF.zRHeaveInterval = auxData.HF.zRHeave(auxData.HF.interval);
% auxData.HF.zRHeaveDotInterval = auxData.HF.zRHeaveDot(auxData.HF.interval);
auxData.HF.RRHInterval = auxData.HF.dRRH(auxData.HF.interval);
auxData.HF.RRHDotInterval = auxData.HF.dRRHDot(auxData.HF.interval);

auxData.HF.xHubRInterval = auxData.HF.dxHubR(auxData.HF.interval);
auxData.HF.xHubRDotInterval = auxData.HF.dxHubRDot(auxData.HF.interval);

auxData.HF.xBumpRInterval = auxData.HF.xBumpR(auxData.HF.interval);
auxData.HF.xBumpRDotInterval = auxData.HF.xBumpRDot(auxData.HF.interval);
auxData.HF.FzRearInterval = auxData.HF.FzRear(auxData.HF.interval);

auxData.HF.vCarInterval = auxData.HF.vCar(auxData.HF.interval);

%% Plant model name, used for Adigator
InternalDynamics=@Dynamics_Bouncing_Internal_RE;
SimDynamics=@Dynamics_Sim;

% Analytic derivative files (optional)
problem.analyticDeriv.gradCost=[];
problem.analyticDeriv.hessianLagrangian=[];
problem.analyticDeriv.jacConst=[];

% Settings file
problem.settings=@settings_AeroDamping_Track;

%Initial Time. t0<tf
problem.time.t0_min=0;
problem.time.t0_max=0;
guess.t0=0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=auxData.HF.intervalTime(end);     
problem.time.tf_max=auxData.HF.intervalTime(end); 
guess.tf=auxData.HF.intervalTime(end);

%% Parameters - Detailed Model - Assuming all can be found from data
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];
%% States - Detailed model
% Initial conditions for system.
problem.states.x0=[auxData.HF.RRHInterval(1), auxData.HF.RRHDotInterval(1), auxData.HF.xHubRInterval(1), auxData.HF.xHubRDotInterval(1)];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=problem.states.x0-1;
problem.states.x0u=problem.states.x0+1;  

% State bounds. xl=< x <=xu
problem.states.xl=[-0.5 -1, -0.5, -1];
problem.states.xu=[0.5, 1, 0.5, 1];   

% State error bounds
problem.states.xErrorTol_local=[1e-3 1e-3 1e-3 1e-3];
problem.states.xErrorTol_integral=[1e-3 1e-3 1e-3 1e-3];


% State constraint error bounds
problem.states.xConstraintTol=[1e-3 1e-3 1e-3 1e-3];

% Terminal state bounds. xfl=< xf <=xfu
% problem.states.xfl=[auxData.HF.RRHInterval(end)-auxData.HF.zRHeaveInterval(end), auxData.HF.zRHeaveDot(auxData.HF.interval(end)), auxData.LF.zRearHubInterval(end)-auxData.HF.zRHeave(auxData.HF.interval(end)), auxData.LF.zDotRearHub(auxData.HF.interval(end))] + 0.1;
% problem.states.xfu= [auxData.HF.RRHInterval(end)-auxData.HF.zRHeaveInterval(end), auxData.HF.zRHeaveDot(auxData.HF.interval(end)), auxData.LF.zRearHubInterval(end)-auxData.HF.zRHeave(auxData.HF.interval(end)), auxData.LF.zDotRearHub(auxData.HF.interval(end))] + 0.1;

problem.states.xfl=[-0.5, -1, -0.5, -1];
problem.states.xfu= [0.5, 1 0.5, 1];  

% Guesses using HF data
x1Guess = auxData.HF.RRHInterval;
x2Guess = auxData.HF.RRHDotInterval;
x3Guess = auxData.HF.xHubRInterval;
x4Guess = auxData.HF.xHubRDotInterval;

guess.time= auxData.HF.intervalTime;
guess.states(:,1)=x1Guess;
guess.states(:,2)=x2Guess;
guess.states(:,3)=x3Guess;
guess.states(:,4)=x4Guess;

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
% u1 - ca;
% u2 - ka;
% u3 - zr;


problem.inputs.ul= [-5e6, -5e6,-10];
problem.inputs.uu= [5e6, 5e6, 10];

% problem.inputs.ul= -5e6;
% problem.inputs.uu= 5e6;

% Bounds on the first control action
problem.inputs.u0l= [-5e5, -5e6, -10];
problem.inputs.u0u= [5e5, 5e6, 10];

% problem.inputs.u0l= -5e6;
% problem.inputs.u0u= 5e6;

% Input rate constraint
% problem.inputs.url=[-1e4, -1e4, -1e4, -1e4];
% problem.inputs.uru=[1e4, 1e4, 1e4, 1e4]; 

% Input constraint error bounds
problem.inputs.uConstraintTol=[10, 10, 0.01];

% problem.inputs.uConstraintTol= 10;

% Guess ks and cs
% ksDef = interp1(auxData.P.xBumpVec, auxData.P.ksVec, x1Guess-x3Guess, 'linear', 'extrap');
% csDef = interp1(auxData.P.vDamVec, auxData.P.csVec, x2Guess-x4Guess, 'linear', 'extrap');

% Guess the input sequences with [u0 uf]
guess.inputs(:, 1) = zeros(length(auxData.HF.intervalTime), 1);
guess.inputs(:, 2) = zeros(length(auxData.HF.intervalTime), 1);
% guess.inputs(:, 3) = csDef;
% guess.inputs(:, 4) = ksDef;
guess.inputs(:,3) =  zeros(length(auxData.HF.intervalTime), 1);
%% Set points
% Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.ng_eq=0;
problem.constraints.gTol_eq=[];

% problem.constraints.gl=[0, 0, 0, 0];
% problem.constraints.gu=[inf, inf, inf, inf];
% problem.constraints.gTol_neq=[0.001 0.001 0.001 0.001];

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
dataX1 = vdat.auxData.HF.RRHInterval;
dataX2 = vdat.auxData.HF.RRHDotInterval;
dataX3 = vdat.auxData.HF.xHubRInterval;
dataX4 = vdat.auxData.HF.xHubRDotInterval;
dataX5 = vdat.auxData.HF.xBumpRInterval;
dataX6 = vdat.auxData.HF.xBumpRDotInterval;

ksDef = interp1(vdat.auxData.P.xBumpVec, vdat.auxData.P.ksVec, x(:,3)-x(:,1), 'linear', 'extrap');
csDef = interp1(vdat.auxData.P.vDamVec, vdat.auxData.P.csVec, x(:,2) - x(:,4), 'linear', 'extrap');

FDam = csDef.*(x(:,2)-x(:,4));
FSpring = ksDef.*(x(:,1)-x(:,3));
Ftotal = FDam + FSpring;

kt = vdat.auxData.P.kt;

dataTime = vdat.auxData.HF.intervalTime;

zr = cumtrapz(t, u(:,3));

sampledTrackX1 = interp1(dataTime, dataX1, t, 'linear', 'extrap');
sampledTrackX2 = interp1(dataTime, dataX2, t, 'linear', 'extrap');
sampledTrackX3 = interp1(dataTime, dataX3, t, 'linear', 'extrap');
sampledTrackX4 = interp1(dataTime, dataX4, t, 'linear', 'extrap');
sampledTrackX5 = interp1(dataTime, dataX5, t, 'linear', 'extrap');
sampledTrackX6 = interp1(dataTime, dataX6, t, 'linear', 'extrap');

% e1 = x(:,1) - sampledTrackX1 + zr;
% e2 = x(:,2) - sampledTrackX2;
% e3 = x(:,3) - sampledTrackX3 + zr;
% e4 = x(:,4) - sampledTrackX4; 

e1 = x(:,1) - sampledTrackX1;
e2 = x(:,2) - sampledTrackX2 - u(:,3);
e3 = x(:,3) - sampledTrackX3;
e4 = x(:,4) - sampledTrackX4 - u(:,3); 
e5 = (x(:,3)-x(:,1)) - sampledTrackX5;
e6 = (x(:,4)-x(:,2)) - sampledTrackX6;

% e1 = x(:,1) - sampledTrackX1;
% e2 = x(:,2) - sampledTrackX2;
% e3 = x(:,3) - sampledTrackX3;
% e4 = x(:,4) - sampledTrackX4;

% dX4 = [diff(x(:,4))./diff(t), 0];
% e5 = Ftotal/kt - (x(:,3));
% e5 = Ftotal/kt - (x(:,3) - u(:,5));
% u1 = u(:,1);
% u2 = u(:,2);
q1 = 5e3;
q2 = 1;
q3 = 1e4;
q4 = 1;
q5 = 1e4;
q6 = 1;
% q5 = 1e2;

% stageCost = q1*e1.*e1 + q2*e2.*e2 + q3*e3.*e3 + q4*e4.*e4 + q5*e5.*e5;
% stageCost = q1*e1.*e1 + q2*e2.*e2 + q3*e3.*e3 + q4*e4.*e4;
stageCost = q1*e1.*e1 + q2*e2.*e2 + q5*e5.*e5 + q6*e6.*e6;

% stageCost = q1*e1.*e1;
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
