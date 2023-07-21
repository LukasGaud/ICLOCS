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
    'mCarSprung_VAP'};
% Load data
trackData = dal.read('filepath', '\\data-server-factory\server\Race\2023\06_04Bcn\2023-03\Data_Parquet\2023-03 HAM Race GP 04-06 180448.prq', 'parameters', channels, 'Laps', 34, 'frequency', 200);
% EoS Condition
[problemEoS,guessEoS]=AeroDampingEstimator_Track_Free(0, trackData);          % Fetch the problem definition

optionsEoS= problemEoS.settings(length(problemEoS.data.auxData.HF.intervalTime));                  % Get options and solver settings 
[solutionEoS,MRHistoryEoS]=solveMyProblem( problemEoS,guessEoS,optionsEoS);

% LS Condition
[problemLS,guessLS]=AeroDampingEstimator_Track_Free(1, trackData);          % Fetch the problem definition

optionsLS= problemLS.settings(length(problemLS.data.auxData.HF.intervalTime));                  % Get options and solver settings 
[solutionLS,MRHistoryLS]=solveMyProblem( problemLS,guessLS,optionsLS);
% [ tv, xv, uv ] = simulateSolution( problem, solution, 'ode113', 0.02);
%% Display outputs

% General Plot
figure
subplot(4,2,[1 3])
hold on
plot(solutionEoS.T, solutionEoS.X(:,1), 'b')
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.RRHInterval, '--r')

plot(solutionLS.T, solutionLS.X(:,1), 'k')
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.RRHInterval, '--g')

legend({'Model - EoS', 'Track (V7P) - EoS'})
title('z_s - z_r')

subplot(4,2,[5 7])
hold on
plot(solutionEoS.T, solutionEoS.X(:,2), 'b')
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.RRHDotInterval, '--r')

plot(solutionLS.T, solutionLS.X(:,2), 'k')
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.RRHDotInterval, '--g')
legend({'Model', 'Track (V7P)'})
title('z_s dot')

subplot(4,2, 2)
hold on
plot(solutionEoS.T, solutionEoS.U(:,1), 'b')
plot(solutionEoS.T, lowpass(solutionEoS.U(:,1), 10, 100), '--r')

plot(solutionLS.T, solutionLS.U(:,1), 'k')
plot(solutionLS.T, lowpass(solutionLS.U(:,1), 10, 100), '--g')

legend({'ca - EoS', 'ca - lowpass', 'cs'})
title('ca & cs')

subplot(4,2, 4)
hold on
plot(solutionEoS.T, solutionEoS.U(:,2), 'b')
plot(solutionEoS.T, solutionEoS.U(:,4), 'r')

plot(solutionLS.T, solutionLS.U(:,2), 'k')
plot(solutionLS.T, solutionLS.U(:,4), 'g')
legend({'ka', 'ks'})
title('ka & ks')

subplot(4,2, 6)
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.FzRearInterval, 'b')

plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.FzRearInterval, 'k')
title('Fz Rear')

subplot(4,2, 8)
plot(solutionEoS.T, solutionEoS.U(:,5), 'b')

plot(solutionLS.T, solutionLS.U(:,5), 'k')
title('Road vertical velocity')

% Aero parameter estimation
figure
subplot(2,2, 1)
hold on
plot(solutionEoS.T, solutionEoS.U(:,1), 'b', 'LineWidth', 2)
plot(solutionEoS.T, lowpass(solutionEoS.U(:,1), 10, 100), '--r', 'LineWidth', 2)

legend({'ca - EoS', 'ca - lowpass - LS'})
title('ca - HS'); xlabel('Time, s')

subplot(2,2, 2)
hold on
plot(solutionLS.T, solutionLS.U(:,1), 'k', 'LineWidth', 2)
plot(solutionLS.T, lowpass(solutionLS.U(:,1), 10, 100), '--g', 'LineWidth', 2)

legend({'ca - LS', 'ca - lowpass - LS'})
title('ca - LS'); xlabel('Time, s')

subplot(2,2, 3)
hold on
plot(solutionEoS.T, solutionEoS.U(:,2), 'b', 'LineWidth', 2)
plot(solutionEoS.T, solutionEoS.U(:,4), 'r', 'LineWidth', 2)
legend({'ka', 'ks'})
title('ka & ks - EoS')

subplot(2,2,4)
hold on
plot(solutionLS.T, solutionLS.U(:,2), 'k', 'LineWidth', 2)
plot(solutionLS.T, solutionLS.U(:,4), 'g', 'LineWidth', 2)
legend({'ka', 'ks'})
title('ka & ks - LS')

figure
subplot(2,2,1)
hold on
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.vCar(problemEoS.data.auxData.HF.interval), 'b', 'LineWidth', 2)
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.vCar(problemLS.data.auxData.HF.interval), 'k', 'LineWidth', 2)
xlabel('Time, s')
title('vCar')

subplot(2,2,3)
hold on
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.RRHInterval, 'b', 'LineWidth', 2)
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.RRHInterval, 'k', 'LineWidth', 2)
xlabel('Time, s')
title('RRH')

subplot(2,2,2)
hold on
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.FzRearInterval, 'b', 'LineWidth', 2)
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.FzRearInterval, 'k', 'LineWidth', 2)
legend({'EoS', 'LS'})
xlabel('Time, s')
title('Fz Rear')

subplot(2,2,4)
hold on
plot(solutionEoS.T, cumtrapz(solutionEoS.T, solutionEoS.U(:,5)), 'b', 'LineWidth', 2)
plot(solutionLS.T, cumtrapz(solutionLS.T, solutionLS.U(:,5)), 'k', 'LineWidth', 2)

title('Road vertical displacement')
ylabel('Road displacement, m')
xlabel('Time, s')

% Solution Compare
figure
subplot(2,2,1)
hold on
plot(solutionEoS.T, solutionEoS.X(:,1), 'b', 'LineWidth', 2)
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.RRHInterval, '--r', 'LineWidth', 2)
legend({'Model - EoS', 'Track (V7P) - EoS'})
title('EoS: z_s - z_r'); ylabel('z_s - z_r'); xlabel('Time, s')

subplot(2,2,2)
hold on
plot(solutionLS.T, solutionLS.X(:,1), 'k', 'LineWidth', 2)
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.RRHInterval, '--g', 'LineWidth', 2)
legend({'Model - LS', 'Track (V7P) - LS'})
title('LS: z_s - z_r'); ylabel('z_s - z_r'); xlabel('Time, s')

subplot(2,2,3)
hold on
plot(solutionEoS.T, solutionEoS.X(:,2), 'b', 'LineWidth', 2)
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.RRHDotInterval, '--r', 'LineWidth', 2)
legend({'Model - EoS', 'Track (V7P) - EoS'})
title('EoS: DOT(z_s)'); ylabel('DOT(z_s) - velocity, m/s'); xlabel('Time, s')

subplot(2,2,4)
hold on
plot(solutionLS.T, solutionLS.X(:,2), 'k', 'LineWidth', 2)
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.RRHDotInterval, '--g', 'LineWidth', 2)
legend({'Model - LS', 'Track (V7P) - LS'})
title('LS: DOT(z_s)'); ylabel('DOT(z_s) - velocity, m/s'); xlabel('Time, s')

figure
subplot(2,2,1)
hold on
plot((solutionEoS.X(:,1))*1000, solutionEoS.U(:,2), '.b', 'LineWidth', 2);

plot((solutionLS.X(:,1))*1000, solutionLS.U(:,2), '.k', 'LineWidth', 2);
xlabel('RRH, m'); ylabel('Aero Stiffness, ka'); title('Aero Stiffness vs RRH')
legend({'EoS', 'LS'})

subplot(2,2,2)
hold on
plot(solutionEoS.X(:,2), solutionEoS.U(:,2), '.b', 'LineWidth', 2);

plot(solutionLS.X(:,2), solutionLS.U(:,2), '.k', 'LineWidth', 2);
xlabel('RRH_{dot}, m'); ylabel('Aero Stiffness, ka'); title('Aero Stiffness vs Dot(RRH)')

subplot(2,2,3)
hold on
% plot(solution.X(:,1) + problem.data.auxData.HF.zRHeaveInterval' + problem.data.auxData.LF.RRHInterval', lowpass(solution.U(:,1), 5, 100), '.');
plot((solutionEoS.X(:,1))*1000, solutionEoS.U(:,1), '.b', 'LineWidth', 2);
plot((solutionLS.X(:,1) )*1000, solutionLS.U(:,1), '.k', 'LineWidth', 2);
xlabel('RRH, m'); ylabel('Aero Damping, ca'); title('Aero Damping vs RRH')

subplot(2,2,4)
hold on
% plot(solution.X(:,2), lowpass(solution.U(:,1), 5, 100), '.');
plot(solutionEoS.X(:,2), solutionEoS.U(:,1), '.b', 'LineWidth', 2);

plot(solutionLS.X(:,2), solutionLS.U(:,1), '.k', 'LineWidth', 2);
xlabel('RRH_{dot}, m'); ylabel('Aero Damping, ca'); title('Aero Damping vs Dot(RRH)')
legend({'EoS', 'LS'})

ksDefEoS = interp1(problemEoS.data.auxData.P.xBumpVec, problemEoS.data.auxData.P.ksVec, solutionEoS.X(:,1)-solutionEoS.X(:,3), 'linear', 'extrap');
csDefEoS = interp1(problemEoS.data.auxData.P.vDamVec, problemEoS.data.auxData.P.csVec, solutionEoS.X(:,2)-solutionEoS.X(:,4), 'linear', 'extrap');

ksDefLS = interp1(problemLS.data.auxData.P.xBumpVec, problemLS.data.auxData.P.ksVec, solutionLS.X(:,1)-solutionLS.X(:,3), 'linear', 'extrap');
csDefLS = interp1(problemLS.data.auxData.P.vDamVec, problemLS.data.auxData.P.csVec, solutionLS.X(:,2)-solutionLS.X(:,4), 'linear', 'extrap');

figure
subplot(2,2,1)
hold on
p1 = plot(solutionEoS.T, ksDefEoS, 'r', 'LineWidth', 2);
p2 = plot(solutionEoS.T, ksDefEoS+0.05*ksDefEoS, '--k', 'LineWidth', 2);
p3 = plot(solutionEoS.T, ksDefEoS-0.05*ksDefEoS, '--k', 'LineWidth', 2);
p4 = plot(solutionEoS.T, solutionEoS.U(:,4), 'b', 'LineWidth', 2);
title('EoS Spring Stiffness')
xlabel('Time, s')
ylabel('Spring Stiffness, N/m')
legend([p1, p2, p4],{'Interpolated (default) value', '5% Bounds', 'Optimised value'})

subplot(2,2,3)
hold on
plot(solutionEoS.T, csDefEoS, 'r', 'LineWidth', 2)
plot(solutionEoS.T, csDefEoS+0.05*csDefEoS, '--k', 'LineWidth', 2)
plot(solutionEoS.T, csDefEoS-0.05*csDefEoS, '--k', 'LineWidth', 2)
plot(solutionEoS.T, solutionEoS.U(:,3), 'b', 'LineWidth', 2)
title('EoS Damper Viscosity')
xlabel('Time, s')
ylabel('Damper Viscosity, Ns/m')

subplot(2,2,2)
hold on
plot(solutionLS.T, ksDefLS, 'r', 'LineWidth', 2)
plot(solutionLS.T, ksDefLS+0.05*ksDefLS, '--k', 'LineWidth', 2)
plot(solutionLS.T, ksDefLS-0.05*ksDefLS, '--k', 'LineWidth', 2)
plot(solutionLS.T, solutionLS.U(:,4), 'b', 'LineWidth', 2)
title('LS Spring Stiffness')
xlabel('Time, s')
ylabel('Spring Stiffness, N/m')

subplot(2,2,4)
hold on
plot(solutionLS.T, csDefLS, 'r', 'LineWidth', 2)
plot(solutionLS.T, csDefLS+0.05*csDefLS, '--k', 'LineWidth', 2)
plot(solutionLS.T, csDefLS-0.05*csDefLS, '--k', 'LineWidth', 2)
plot(solutionLS.T, solutionLS.U(:,3), 'b', 'LineWidth', 2)
title('LS Damper Viscosity')
xlabel('Time, s')
ylabel('Damper Viscosity, Ns/m')
%%
% All states compare
figure
subplot(2,2,1)
hold on
plot(solutionEoS.T, solutionEoS.X(:,1), 'b', 'LineWidth', 2)
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.RRHInterval, '--r', 'LineWidth', 2)
legend({'Model - EoS', 'Track (V7P) - EoS'})
title('EoS: z_s - z_r'); ylabel('z_s - z_r'); xlabel('Time, s')

subplot(2,2,2)
hold on
plot(solutionEoS.T, solutionEoS.X(:,2), 'k', 'LineWidth', 2)
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.RRHDotInterval, '--g', 'LineWidth', 2)
legend({'Model - LS', 'Track (V7P) - LS'})
title('z_s dot'); ylabel('z_s dot'); xlabel('Time, s')

subplot(2,2,3)
hold on
plot(solutionEoS.T, solutionEoS.X(:,3), 'b', 'LineWidth', 2)
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.xHubRInterval, '--r', 'LineWidth', 2)
legend({'Model - EoS', 'Track (V7P) - EoS'})
title('EoS: DOT(z_s)'); ylabel('DOT(z_s) - velocity, m/s'); xlabel('Time, s')

subplot(2,2,4)
hold on
plot(solutionEoS.T, solutionEoS.X(:,4), 'k', 'LineWidth', 2)
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.xHubRDotInterval, '--g', 'LineWidth', 2)
legend({'Model - LS', 'Track (V7P) - LS'})
title('LS: DOT(z_s)'); ylabel('DOT(z_s) - velocity, m/s'); xlabel('Time, s')

figure
hold on
subplot(5,1,1)
plot(solutionEoS.T, solutionEoS.U(:,1))

hold on
subplot(5,1,2)
plot(solutionEoS.T, solutionEoS.U(:,2))

hold on
subplot(5,1,3)
plot(solutionEoS.T, solutionEoS.U(:,3))

hold on
subplot(5,1,4)
plot(solutionEoS.T, solutionEoS.U(:,4))

hold on
subplot(5,1,5)
plot(solutionEoS.T, solutionEoS.U(:,5))
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
