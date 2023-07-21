% Main script to solve the Optimal Control Problem 
%
% Double Integrator Tracking Problem
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk


%--------------------------------------------------------

clear; close all; format compact;
% Load Track data
dal = DET.Access.Layer;
% Channel names
channels = {
    'vCar:Chassis'; % vCar - for reference
    'hRideR_VAP'; % hRideR = z_s
    'KHeaveAxleR_VAP'; % stiffness = ks 
    'xHeaveLinkRC_VAP'; % spring compression
    'FDamperRC_vts:vTAGOLM';
    'vDamperRC_vts:vTAGOLM';
    'mCar_VAP';
    'rCarMass_VAP';
    'tLap_VAP';
    'xWheelSetupRL_VAP';
    'xWheelSetupRR_VAP';
    'FAeroAxleRWTCalc_VAP';
    'mCarSprung_VAP';
    'zSetupWheelCenterRL0_PLR';
    'zSetupWheelCenterRR0_PLR';
    'hRideSetupR_VAP'};
% Load data
trackData = dal.read('filepath', '\\data-server-factory\server\Race\2023\06_04Bcn\2023-03\Data_Parquet\2023-03 HAM Race GP 04-06 180448.prq', 'parameters', channels, 'Laps', 34, 'frequency', 200);
% EoS Condition
[problemEoS,guessEoS]=AeroDampingEstimator_Track_Free(0, trackData);          % Fetch the problem definition

optionsEoS= problemEoS.settings(length(problemEoS.data.auxData.HF.intervalTime));                  % Get options and solver settings 
[solutionEoS,MRHistoryEoS]=solveMyProblem(problemEoS,guessEoS,optionsEoS);

% LS Condition
[problemLS,guessLS]=AeroDampingEstimator_Track_Free(1, trackData);          % Fetch the problem definition

optionsLS= problemLS.settings(length(problemLS.data.auxData.HF.intervalTime));                  % Get options and solver settings 
[solutionLS,MRHistoryLS]=solveMyProblem( problemLS,guessLS,optionsLS);
% [ tv, xv, uv ] = simulateSolution( problem, solution, 'ode113', 0.02);

fPlotFigures
%% Eigenvalue plotting

% Eigenvalues EoS
[lambdaEoS, eigVecEoS] = fGetEigenvalues(problemEoS, solutionEoS);
% sprintf('ct: %.0f, kt: %.0f', solution.p(1), solution.p(2))
% %% figure
% xx=linspace(solution.T(1,1),solution.tf,1000);
% 
% 
% figure
% hold on
% plot(xx,speval(solution,'X',1,xx),'r-' )
% plot(xx,speval(solution,'X',2,xx),'b-' )
% plot(tv,xv(:,1),'k-.' )
% plot(tv,xv(:,2),'k-.' )
% xlabel('Time [s]')
% ylabel('States')
% legend('Position [m]','Velocity [m/s]')
% grid on
% 
% figure
% hold on
% plot(xx,speval(solution,'U',1,xx),'b-' )
% plot(tv,uv(:,1),'k-.' )
% plot([solution.T(1,1); solution.tf],[problem.inputs.ul, problem.inputs.ul],'r-' )
% plot([solution.T(1,1); solution.tf],[problem.inputs.uu, problem.inputs.uu],'r-' )
% xlim([0 solution.tf])
% xlabel('Time [s]')
% grid on
% ylabel('Control Input')
% legend('u [N]')
% 
% figure
% plot(xx,(speval(solution,'X',1,xx)-5.*sin(xx')))
% hold on
% plot(tv,(xv(:,1)-5.*sin(tv)),'k-.' )
% xlabel('Time [s]')
% grid on
% ylabel('Tracking error')
% legend('Position error [m]')
% 
