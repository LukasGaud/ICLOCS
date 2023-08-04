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
    'hRideSetupR_VAP';
    'sLap:Chassis';
    'NLap:Chassis';
    'Driver'};
% Load data
dataList = {
    '\\data-server\server\Test\2023\Sil04_15\2023-01\Data_Parquet\2023-01 RUS Test D2 16-04 161307.prq', 30, 1600, 1680;
    '\\data-server\server\Test\2023\Sil04_15\2023-01\Data_Parquet\2023-01 RUS Test D2 16-04 155938.prq', 28, 1600, 1680};

for iData = 1:size(dataList,1)
    [trackData, sessionDetails] = dal.read('filepath', dataList{iData,1}, 'parameters', channels, 'Laps', dataList{iData,2}, 'frequency', 200);
    % prepare the problem
    [problem{iData},guess{iData}]=AeroDampingEstimator_Track_stochRoad(trackData, sessionDetails, dataList{iData, 3}, dataList{iData,4});          % Fetch the problem definition
    options{iData}= problem{iData}.settings(length(problem{iData}.data.auxData.HF.intervalTime));                  % Get options and solver settings 
    [solution{iData},MRHistory{iData}]=solveMyProblem(problem{iData},guess{iData},options{iData});

    [problemConst{iData},guessConst{iData}]=AeroDampingEstimator_Param_Only_PSD(problem{iData}.data.auxData);
    optionsConst{iData} = problemConst{iData}.settings(length(problemConst{iData}.data.auxData.HF.intervalTime));                  % Get options and solver settings 
    [solutionConst{iData},MRHistoryConst{iData}]=solveMyProblem(problemConst{iData},guessConst{iData},optionsConst{iData});

    [problemPSD{iData},guessPSD{iData}]=AeroDampingEstimator_sLap_PSD(problem{iData}.data.auxData);          % Fetch the problem definition
    optionsPSD{iData} = problemPSD{iData}.settings(length(problemPSD{iData}.data.auxData.HF.intervalTime));                  % Get options and solver settings 
    [solutionPSD{iData},MRHistoryPSD{iData}]=solveMyProblem(problemPSD{iData},guessPSD{iData},optionsPSD{iData});
    
end
fPlotFigures_sLap_v2
%% Eigenvalue plotting

% % Eigenvalues EoS
% [lambdaEoS, eigVecEoS] = fGetEigenvalues(problemEoS, solutionEoS);
% 
