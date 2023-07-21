function [problem,guess] = AeroDampingEstimator_sLap_PSD(auxData)
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

%% Plant model name, used for Adigator
InternalDynamics=@Dynamics_Internal_sLap_PSD_for_ipopt;
SimDynamics=@Dynamics_Sim;

% Analytic derivative files (optional)
problem.analyticDeriv.gradCost=[];
problem.analyticDeriv.hessianLagrangian=[];
problem.analyticDeriv.jacConst=[];

% Settings file
problem.settings=@settings_AeroDamping_Track_PSD;

%Initial Time. t0<tf
problem.time.t0_min=0;
problem.time.t0_max=0;
guess.t0=0;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=auxData.HF.intervalTime(end);     
problem.time.tf_max=auxData.HF.intervalTime(end); 
guess.tf=auxData.HF.intervalTime(end);

%% Parameters - Detailed Model - Assuming all can be found from data
% p(1) = kt, p(2) = ct, p(3) = ka, p(4) = ca, p(5) = alpha
% problem.parameters.pl=[auxData.P.kt-0.5*auxData.P.kt, 0, -1e7, -1e7, 0];
% problem.parameters.pu=[auxData.P.kt+auxData.P.kt, 2*auxData.P.ct, 1e7, 1e7, 1];
% guess.parameters=[auxData.P.kt, auxData.P.ct, 0, 0, 0.001];

problem.parameters.pl=[auxData.P.kt-0.5*auxData.P.kt, 0, 0];
problem.parameters.pu=[auxData.P.kt+auxData.P.kt, 2*auxData.P.ct, 1];
guess.parameters=[auxData.P.kt, auxData.P.ct, 0.001];
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
% No inputs

problem.inputs.ul= [-5e7, -5e7];
problem.inputs.uu= [5e7, 5e7];

% Bounds on the first control action
problem.inputs.u0l= [-5e5, -5e6];
problem.inputs.u0u= [5e5, 5e6];

% Input constraint error bounds
problem.inputs.uConstraintTol=[10, 10];

% problem.inputs.uConstraintTol= 10;

% Guess the input sequences with [u0 uf]
guess.inputs(:, 1) = zeros(length(auxData.HF.intervalTime), 1);
guess.inputs(:, 2) = zeros(length(auxData.HF.intervalTime), 1);
% %% Set points
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
auxData.P.roadVec = randn(length(auxData.HF.intervalTime),1);
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

dataTime = vdat.auxData.HF.intervalTime;

sampledTrackX1 = interp1(dataTime, dataX1, t, 'linear', 'extrap');
sampledTrackX2 = interp1(dataTime, dataX2, t, 'linear', 'extrap');
sampledTrackX3 = interp1(dataTime, dataX3, t, 'linear', 'extrap');
sampledTrackX4 = interp1(dataTime, dataX4, t, 'linear', 'extrap');
sampledTrackX5 = interp1(dataTime, dataX5, t, 'linear', 'extrap');
sampledTrackX6 = interp1(dataTime, dataX6, t, 'linear', 'extrap');

[track.x1_psd, track.y1_psd] = fnPSD(t, sampledTrackX1, 'HighPassFreq', 2, 'HighPassOrder', 3);
[track.x2_psd, track.y2_psd] = fnPSD(t, sampledTrackX2, 'HighPassFreq', 2, 'HighPassOrder', 3);
[track.x3_psd, track.y3_psd] = fnPSD(t, sampledTrackX5, 'HighPassFreq', 2, 'HighPassOrder', 3);
[track.x4_psd, track.y4_psd] = fnPSD(t, sampledTrackX6, 'HighPassFreq', 2, 'HighPassOrder', 3);

alpha = p(1,3);
zrDot = alpha*interp1(vdat.auxData.HF.intervalTime, vdat.auxData.P.roadVec, t, 'linear', 'extrap');

if size(x,1) == length(t)
    [model.x1_psd, model.y1_psd] = fnPSD(t, x(:,1), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model.x2_psd, model.y2_psd] = fnPSD(t, (x(:,2) - zrDot), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model.x3_psd, model.y3_psd] = fnPSD(t, (x(:,3) - x(:,1)), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model.x4_psd, model.y4_psd] = fnPSD(t, (x(:,4) - x(:,2)), 'HighPassFreq', 2, 'HighPassOrder', 3);
    
    % Resample to the same length in frequency domain
    f = 0:0.01:10;
    
    %Resample track
    TrackX1 = interp1(track.x1_psd, track.y1_psd, f, 'linear');
    TrackX2 = interp1(track.x2_psd, track.y2_psd, f, 'linear');
    TrackX3 = interp1(track.x3_psd, track.y3_psd, f, 'linear');
    TrackX4 = interp1(track.x4_psd, track.y4_psd, f, 'linear');
    
    % Resample Model
    ModelX1 = interp1(model.x1_psd, model.y1_psd, f, 'linear');
    ModelX2 = interp1(model.x2_psd, model.y2_psd, f, 'linear');
    ModelX3 = interp1(model.x3_psd, model.y3_psd, f, 'linear');
    ModelX4 = interp1(model.x4_psd, model.y4_psd, f, 'linear');
    
    % Weightings
    q1 = 1e6;
    q2 = 1e-2;
    q3 = 2e7;
    q4 = 1e1;
    
    % errors
    e1 = abs(TrackX1 - ModelX1);
    e2 = abs(TrackX2 - ModelX2);
    e3 = abs(TrackX3 - ModelX3);
    e4 = abs(TrackX4 - ModelX4);

    Cost = q1*e1*(e1') + q2*e2*(e2') + q3*e3*(e3') + q4*e4*(e4');
%     Cost = q1*e1*(e1');
%     Cost = q1*e1*(e1') + q3*e3*(e3');
else
    Cost = inf;
end
dX1 = abs(mean(sampledTrackX1) - mean(x(:,1)));
dX3 = abs(mean(sampledTrackX5) - mean(x(:,3)-x(:,1)));
Q1 = 1;
Q3 = 1;
Cost = Cost + (Q1*dX1 + Q3*dX3);
stageCost = Cost*ones(length(t),1);
%------------- END OF CODE --------------


function boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,vdat) 

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
