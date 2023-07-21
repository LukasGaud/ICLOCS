%% Display outputs
% Aero parameter estimation
ksDefEoS = interp1(problemEoS.data.auxData.P.xBumpVec, problemEoS.data.auxData.P.ksVec, solutionEoS.X(:,3)-solutionEoS.X(:,1), 'linear', 'extrap');
csDefEoS = interp1(problemEoS.data.auxData.P.vDamVec, problemEoS.data.auxData.P.csVec, solutionEoS.X(:,2)-solutionEoS.X(:,4), 'linear', 'extrap');

ksDefLS = interp1(problemLS.data.auxData.P.xBumpVec, problemLS.data.auxData.P.ksVec, solutionLS.X(:,3)-solutionLS.X(:,1), 'linear', 'extrap');
csDefLS = interp1(problemLS.data.auxData.P.vDamVec, problemLS.data.auxData.P.csVec, solutionLS.X(:,2)-solutionLS.X(:,4), 'linear', 'extrap');

figure
subplot(2,2, 1)
hold on
plot(solutionEoS.T, solutionEoS.U(:,1), 'k', 'LineWidth', 2)
plot(solutionEoS.T, csDefEoS, 'b', 'LineWidth', 2)

legend({'ca - EoS', 'cs - EoS'})
title('ca - HS'); xlabel('Time, s')

subplot(2,2, 2)
hold on
plot(solutionLS.T, solutionLS.U(:,1), 'k', 'LineWidth', 2)
plot(solutionLS.T, csDefLS, 'b', 'LineWidth', 2)

legend({'ca - LS', 'cs - LS'})
title('ca - LS'); xlabel('Time, s')

subplot(2,2, 3)
hold on
plot(solutionEoS.T, solutionEoS.U(:,2), 'k', 'LineWidth', 2)
plot(solutionEoS.T, ksDefEoS, 'b', 'LineWidth', 2)
legend({'ka', 'ks'})
title('ka & ks - EoS')

subplot(2,2,4)
hold on
plot(solutionLS.T, solutionLS.U(:,2), 'k', 'LineWidth', 2)
plot(solutionLS.T, ksDefLS, 'b', 'LineWidth', 2)
legend({'ka', 'ks'})
title('ka - LS')

% External Inputs
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

zrEoS = cumtrapz(solutionEoS.T, solutionEoS.U(:,3));
zrLS = cumtrapz(solutionLS.T, solutionLS.U(:,3));

subplot(2,2,4)
hold on
plot(solutionEoS.T, cumtrapz(solutionEoS.T, solutionEoS.U(:,3)), 'b', 'LineWidth', 2)
plot(solutionLS.T, cumtrapz(solutionLS.T, solutionLS.U(:,3)), 'k', 'LineWidth', 2)

title('Road vertical displacement')
ylabel('Road displacement, m')
xlabel('Time, s')

figure
subplot(2,2,1)
hold on
plot((solutionEoS.X(:,1)+solutionEoS.U(:,3))*1000 + mean(problemEoS.data.auxData.HF.RRHSetup), solutionEoS.U(:,2), '.b', 'LineWidth', 2);

plot((solutionLS.X(:,1)+solutionLS.U(:,3))*1000 + mean(problemLS.data.auxData.HF.RRHSetup), solutionLS.U(:,2), '.k', 'LineWidth', 2);
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
plot((solutionEoS.X(:,1)+solutionEoS.U(:,3))*1000 + mean(problemEoS.data.auxData.HF.RRHSetup), solutionEoS.U(:,1), '.b', 'LineWidth', 2);
plot((solutionLS.X(:,1) +solutionLS.U(:,3))*1000 + mean(problemLS.data.auxData.HF.RRHSetup), solutionLS.U(:,1), '.k', 'LineWidth', 2);
xlabel('RRH, m'); ylabel('Aero Damping, ca'); title('Aero Damping vs RRH')

subplot(2,2,4)
hold on
% plot(solution.X(:,2), lowpass(solution.U(:,1), 5, 100), '.');
plot(solutionEoS.X(:,2), solutionEoS.U(:,1), '.b', 'LineWidth', 2);

plot(solutionLS.X(:,2), solutionLS.U(:,1), '.k', 'LineWidth', 2);
xlabel('RRH_{dot}, m/s'); ylabel('Aero Damping, ca'); title('Aero Damping vs Dot(RRH)')
legend({'EoS', 'LS'})


% All states compare - EoS
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
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.RRHDotInterval + solutionEoS.U(:,3)', '--g', 'LineWidth', 2)
legend({'Model - LS', 'Track (V7P) - LS'})
title('z_s dot'); ylabel('z_s dot'); xlabel('Time, s')

subplot(2,2,3)
hold on
plot(solutionEoS.T, solutionEoS.X(:,3), 'b', 'LineWidth', 2)
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.xHubRInterval, '--r', 'LineWidth', 2)
legend({'Model - EoS', 'Track (V7P) - EoS'})
title('EoS: z_u-z_r'); ylabel('EoS: z_u-z_r'); xlabel('Time, s')

subplot(2,2,4)
hold on
plot(solutionEoS.T, solutionEoS.X(:,4), 'k', 'LineWidth', 2)
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.xHubRDotInterval + solutionEoS.U(:,3)', '--g', 'LineWidth', 2)
legend({'Model - LS', 'Track (V7P) - LS'})
title('EoS: z_u dot'); ylabel('EoS: z_u dot, m/s'); xlabel('Time, s')

% All states compare - EoS
figure
subplot(2,1,1)
hold on
plot(solutionEoS.T, (solutionEoS.X(:,3) - solutionEoS.X(:,1)), 'b', 'LineWidth', 2)
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.xBumpRInterval, '--r', 'LineWidth', 2)
legend({'Model - EoS', 'Track (V7P) - EoS'})
title('EoS: z_u - z_s'); ylabel('z_u - z_s'); xlabel('Time, s')

subplot(2,1,2)
hold on
plot(solutionEoS.T, (solutionEoS.X(:,4) - solutionEoS.X(:,2)), 'k', 'LineWidth', 2)
plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.xBumpRDotInterval, '--g', 'LineWidth', 2)
legend({'Model - LS', 'Track (V7P) - LS'})
title('z_u dot - z_s dot'); ylabel('z_u dot - z_s dot'); xlabel('Time, s')


% All states compare - LS
figure
subplot(2,2,1)
hold on
plot(solutionLS.T, solutionLS.X(:,1), 'b', 'LineWidth', 2)
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.RRHInterval, '--r', 'LineWidth', 2)
legend({'Model - EoS', 'Track (V7P) - EoS'})
title('EoS: z_s'); ylabel('LS: z_s'); xlabel('Time, s')

subplot(2,2,2)
hold on
plot(solutionLS.T, solutionLS.X(:,2), 'k', 'LineWidth', 2)
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.RRHDotInterval + solutionLS.U(:,3)', '--g', 'LineWidth', 2)
legend({'Model - LS', 'Track - LS'})
title('LS: z_s dot'); ylabel('z_s dot'); xlabel('Time, s')

subplot(2,2,3)
hold on
plot(solutionLS.T, solutionLS.X(:,3), 'b', 'LineWidth', 2)
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.xHubRInterval, '--r', 'LineWidth', 2)
legend({'Model - EoS', 'Track - EoS'})
title('LS: z_u'); ylabel('LS: z_u'); xlabel('Time, s')

subplot(2,2,4)
hold on
plot(solutionLS.T, solutionLS.X(:,4), 'k', 'LineWidth', 2)
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.xHubRDotInterval + solutionLS.U(:,3)', '--g', 'LineWidth', 2)
legend({'Model - LS', 'Track - LS'})
title('LS: z_u dot'); ylabel('LS: z_u dot, m/s'); xlabel('Time, s')

% All states compare - LS
figure
subplot(2,1,1)
hold on
plot(solutionLS.T, (solutionLS.X(:,3) - solutionLS.X(:,1)), 'b', 'LineWidth', 2)
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.xBumpRInterval, '--r', 'LineWidth', 2)
legend({'Model - EoS', 'Track (V7P) - EoS'})
title('LS: z_u - z_s'); ylabel('z_u - z_s'); xlabel('Time, s')

subplot(2,1,2)
hold on
plot(solutionLS.T, (solutionLS.X(:,4) - solutionLS.X(:,2)), 'k', 'LineWidth', 2)
plot(problemLS.data.auxData.HF.intervalTime, problemLS.data.auxData.HF.xBumpRDotInterval, '--g', 'LineWidth', 2)
legend({'Model - LS', 'Track (V7P) - LS'})
title('z_u dot - z_s dot'); ylabel('z_u dot - z_s dot'); xlabel('Time, s')
