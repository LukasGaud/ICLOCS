%% Display outputs
% Create figures for plotting
% Aero Damping and Stiffness Compare
close all
clear pScatter
% External Inputs
f2 = figure;
subplot(4, 1,1)
hold on
Axf21 = gca;
ylabel(Axf21, 'vCar, kph')
title(Axf21, 'External Inputs compare')
subplot(4, 1,2);
hold on
Axf22 = gca;
ylabel(Axf22, 'Fz on Rear of Car, N')
ylim([-inf, 0])
subplot(4, 1,3);
hold on
Axf23 = gca;
ylabel(Axf23, 'RRH: (z_s - z_r), mm')

subplot(4, 1,4);
hold on
Axf24 = gca;
ylabel(Axf24, 'Estimated Road height (z_r), mm')
xlabel(Axf24, 'Time')
% Aero Parameters Scatters
f3 = figure;
subplot(2,2,1)
hold on
Axf31 = gca;
ylabel(Axf31, 'k_a')
title(Axf31, 'Estimated k_a & c_a in RRH domain')

subplot(2,2,2);
hold on
Axf32 = gca;
ylabel(Axf32, 'k_a')
title(Axf32, 'Estimated k_a & c_a in dot(RRH) domain')

subplot(2,2,3);
hold on
Axf33 = gca;
ylabel(Axf33, 'c_a')
xlabel(Axf33, 'RRH: (z_s - z_r)')
subplot(2,2,4);
hold on
Axf34 = gca;
ylabel(Axf34, 'c_a')
xlabel(Axf34, 'RRH Velocity: dot(z_s - z_r)')

colors = ['g', 'k', 'b', 'r'];
colorsRGB = [
    0, 1, 0;
    0, 0, 0;
    0, 0, 1;
    1, 0, 0];
linestyle = {'-', '--', '-', '--'};
leg1 = {};
leg2 = {};
leg3 = {};
leg4 = {};
leg5 = {};

index1 = 1;
index2 = 1;
index3 = 1;
index4 = 1;
index5 = 1;

i = 1;
% for i = 1:size(problem,2)
    
    % Aero Damping and Stiffness Compare
    ksDef = interp1(problem{i}.data.auxData.P.xBumpVec, problem{i}.data.auxData.P.ksVec, solution{i}.X(:,3)-solution{i}.X(:,1), 'spline', 'extrap');
    csDef = interp1(problem{i}.data.auxData.P.vDamVec, problem{i}.data.auxData.P.csVec, solution{i}.X(:,4)-solution{i}.X(:,2), 'spline', 'extrap');
    
    % Outing Parameters
    Driver  = problem{i}.data.auxData.P.driver;
    Lap = mean(problem{i}.data.auxData.P.NLap);
    sLapStart = problem{i}.data.auxData.P.sLapStart;
    sLapEnd = problem{i}.data.auxData.P.sLapEnd;
    index1 = 1;
    index2 = 1;
    % Figure 1
    figure
    subplot(2,1,1)
    hold on
    Ax = gca;
    % ca
    plot(Ax, solution{i}.T, solution{i}.U(:,1), 'LineWidth', 2, 'LineStyle', linestyle(index1), 'Color', colors(index1))
    leg1{index1} = ['ca - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index1 = index1 + 1;

    % ca - PSD
    plot(Ax, solutionPSD{i}.T, solutionPSD{i}.U(:,1), 'LineWidth', 2, 'LineStyle', linestyle(index1), 'Color', colors(index1))
    leg1{index1} = ['ca PSD - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index1 = index1 + 1;

    % cs
    plot(Ax, solution{i}.T, csDef, 'LineWidth', 2, 'LineStyle', linestyle(index1), 'Color', colors(index1))
    leg1{index1} = ['cs - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index1 = index1 + 1;
    legend(Ax,leg1);
    title(Ax, ['Estimated Stiffness and Damping Compare for: ', Driver, ' L', num2str(Lap)])
    ylabel(Ax, 'Damping Terms')
    subplot(2,1,2)
    hold on
    Ax = gca;
    % ka
    plot(Ax, solution{i}.T, solution{i}.U(:,2), 'LineWidth', 2, 'LineStyle', linestyle(index2), 'Color', colors(index2))
    leg2{index2} = ['ka - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index2 = index2 + 1;

    % ka - PSD
    plot(Ax, solutionPSD{i}.T, solutionPSD{i}.U(:,2), 'LineWidth', 2, 'LineStyle', linestyle(index2), 'Color', colors(index2))
    leg2{index2} = ['ka - PSD - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index2 = index2 + 1;

    % ks
    plot(Ax, solution{i}.T, ksDef, 'LineWidth', 2, 'LineStyle', linestyle(index2), 'Color', colors(index2))
    leg2{index2} = ['ks - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index2 = index2 + 1;
    legend(Ax,leg2);
    xlabel(Ax, 'Time, s')
    ylabel(Ax, 'Stiffness Terms')
    % Figure 2
    % vCar
    plot(Axf21, problem{i}.data.auxData.HF.intervalTime, problem{i}.data.auxData.HF.vCar(problem{i}.data.auxData.HF.interval), 'LineWidth', 2, 'LineStyle', linestyle(index3), 'Color', colors(index3))
    
    % RRH
    plot(Axf23, problem{i}.data.auxData.HF.intervalTime, (problem{i}.data.auxData.HF.RRHInterval + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, 'LineWidth', 2, 'LineStyle', linestyle(index3), 'Color', colors(index3))
    
    % Fz
    plot(Axf22, problem{i}.data.auxData.HF.intervalTime, problem{i}.data.auxData.HF.FzRearInterval, 'LineWidth', 2, 'LineStyle', linestyle(index3), 'Color', colors(index3))

    % Road Displacement
    zr = cumtrapz(solution{i}.T, solution{i}.U(:,3));
    plot(Axf24, problem{i}.data.auxData.HF.intervalTime, zr*1000, 'LineWidth', 2, 'LineStyle', linestyle(index3), 'Color', colors(index3))
    
    leg3{index3} = [Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
    index3 = index3 + 1;
    legend(Axf24, leg3)

    % Figure 3
    % ka vs RRH
    plot(Axf31, (solution{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solution{i}.U(:,2), '.', 'MarkerSize', 5, 'Color', colors(index4))
    plot(Axf31, (solution{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solution{i}.U(:,2), 'LineWidth', 1, 'Color', [colorsRGB(index4, :), 0.1]);
    % PSD parameters
    plot(Axf31, (solutionPSD{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solutionPSD{i}.U(:,2), '.', 'MarkerSize', 5, 'Color', colors(index4+1))
    plot(Axf31, (solutionPSD{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solutionPSD{i}.U(:,2), 'LineWidth', 1, 'Color', [colorsRGB(index4+1, :), 0.1]);
    
    % ca vs RRH
    plot(Axf33, (solution{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solution{i}.U(:,1), '.', 'MarkerSize', 5,  'Color', colors(index4))
    plot(Axf33, (solution{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solution{i}.U(:,1), 'LineWidth', 1, 'Color', [colorsRGB(index4, :), 0.1]);
    
    % PSD parameters
    plot(Axf33, (solutionPSD{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solutionPSD{i}.U(:,1), '.', 'MarkerSize', 5,  'Color', colors(index4+1))
    plot(Axf33, (solutionPSD{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solutionPSD{i}.U(:,1), 'LineWidth', 1, 'Color', [colorsRGB(index4+1, :), 0.1]);
 
    % ka vs RRH_Dot
    plot(Axf32, solution{i}.X(:,2) - solution{i}.U(:,3) , solution{i}.U(:,2), '.', 'MarkerSize', 5, 'Color', colors(index4))
    plot(Axf32, solution{i}.X(:,2) - solution{i}.U(:,3) , solution{i}.U(:,2), 'LineWidth', 1, 'Color', [colorsRGB(index4, :), 0.1]);
    
    % PSD parameters
    plot(Axf32, solutionPSD{i}.X(:,2), solutionPSD{i}.U(:,2), '.', 'MarkerSize', 5, 'Color', colors(index4+1))
    plot(Axf32, solutionPSD{i}.X(:,2), solutionPSD{i}.U(:,2), 'LineWidth', 1, 'Color', [colorsRGB(index4+1, :), 0.1]);
   
    % ka vs RRH_Dot
    pScatter(index4) = plot(Axf34, solution{i}.X(:,2), solution{i}.U(:,1), '.', 'MarkerSize', 5, 'Color', colors(index4));
    plot(Axf34, solution{i}.X(:,2), solution{i}.U(:,1), 'LineWidth', 1, 'Color', [colorsRGB(index4, :), 0.1]);
    leg4{index4} = [Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
    index4 = index4 + 1;

    pScatter(index4) = plot(Axf34, solutionPSD{i}.X(:,2), solutionPSD{i}.U(:,1), '.', 'MarkerSize', 5, 'Color', colors(index4));
    plot(Axf34, solution{i}.X(:,2), solutionPSD{i}.U(:,1), 'LineWidth', 1, 'Color', [colorsRGB(index4, :), 0.1]);
    leg4{index4} = [Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd), ' PSD'];
    index4 = index4 + 1;

    legend(Axf34, pScatter, leg4)

    % Figure 5
    index5 = 1;
    figure
    subplot(4,1,1)
    hold on
    Axf41 = gca;
    title(Axf41, 'Model and Track Reponse Compare')
    ylabel(Axf41, 'RRH: z_s - z_r, mm')
    subplot(4,1,2);
    hold on
    Axf42 = gca;
    ylabel(Axf42, 'dot(RRH) - dot(z_r): dot(z_s), mm/s')

    subplot(4,1,3);
    hold on
    Axf43 = gca;
    ylabel(Axf43, 'Bump Motion: (z_u - z_s), mm')

    subplot(4,1,4);
    hold on
    Axf44 = gca;
    ylabel(Axf44, 'Bump Velocity: dot(z_u - z_s), mm/s')
    % Model
    % z_s-z_r
    plot(Axf41, solution{i}.T, (solution{i}.X(:,1)+ mean(problem{i}.data.auxData.HF.RRHSetup))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    % z_s dot
    plot(Axf42, solution{i}.T, solution{i}.X(:,2)*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    % z_u - z_s
    plot(Axf43, solution{i}.T, (solution{i}.X(:,3) - solution{i}.X(:,1))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    % (z_u - z_s) dot
    plot(Axf44, solution{i}.T, (solution{i}.X(:,4) - solution{i}.X(:,2))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    
    leg5{index5} = ['Model - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
    index5 = index5 + 1;

%     % Model Constant Values
%     % z_s-z_r
%     plot(Axf41, solutionConst{i}.T, (solutionConst{i}.X(:,1)+ mean(problemConst{i}.data.auxData.HF.RRHSetup))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
%     % z_s dot
%     plot(Axf42, solutionConst{i}.T, solutionConst{i}.X(:,2)*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
%     % z_u - z_s
%     plot(Axf43, solutionConst{i}.T, (solutionConst{i}.X(:,3) - solutionConst{i}.X(:,1))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
%     % (z_u - z_s) dot
%     plot(Axf44, solutionConst{i}.T, (solutionConst{i}.X(:,4) - solutionConst{i}.X(:,2))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
%     
%     leg5{index5} = ['Model Const - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
%     index5 = index5 + 1;
%     
    % Model PSD
    % z_s-z_r
    plot(Axf41, solutionPSD{i}.T, (solutionPSD{i}.X(:,1)+ mean(problem{i}.data.auxData.HF.RRHSetup))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    % z_s dot
    plot(Axf42, solutionPSD{i}.T, solutionPSD{i}.X(:,2)*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    % z_u - z_s
    plot(Axf43, solutionPSD{i}.T, (solutionPSD{i}.X(:,3) - solutionPSD{i}.X(:,1))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    % (z_u - z_s) dot
    plot(Axf44, solutionPSD{i}.T, (solutionPSD{i}.X(:,4) - solutionPSD{i}.X(:,2))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    
    leg5{index5} = ['Model PSD - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
    index5 = index5 + 1;

    %Track
    % z_s-z_r
    plot(Axf41, problem{i}.data.auxData.HF.intervalTime, (problem{i}.data.auxData.HF.RRHInterval+ mean(problem{i}.data.auxData.HF.RRHSetup))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    % z_s dot
    plot(Axf42, problem{i}.data.auxData.HF.intervalTime, (problem{i}.data.auxData.HF.RRHDotInterval + solution{i}.U(:,3)')*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    % z_u - z_s
    plot(Axf43, problem{i}.data.auxData.HF.intervalTime, (problem{i}.data.auxData.HF.xBumpRInterval*1000), 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    % (z_u - z_s) dot
    plot(Axf44, problem{i}.data.auxData.HF.intervalTime, (problem{i}.data.auxData.HF.xBumpRDotInterval*1000), 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colors(index5))
    
    leg5{index5} = ['Track - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
    index5 = index5 + 1;
    legend(Axf44, leg5)

% end

% All states compare - EoS

% 
% subplot(2,2,3)
% hold on
% plot(solutionEoS.T, solutionEoS.X(:,3), 'b', 'LineWidth', 2)
% plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.xHubRInterval, '--r', 'LineWidth', 2)
% legend({'Model - EoS', 'Track (V7P) - EoS'})
% title('EoS: z_u-z_r'); ylabel('EoS: z_u-z_r'); xlabel('Time, s')
% 
% subplot(2,2,4)
% hold on
% plot(solutionEoS.T, solutionEoS.X(:,4), 'k', 'LineWidth', 2)
% plot(problemEoS.data.auxData.HF.intervalTime, problemEoS.data.auxData.HF.xHubRDotInterval + solutionEoS.U(:,3)', '--g', 'LineWidth', 2)
% legend({'Model - LS', 'Track (V7P) - LS'})
% title('EoS: z_u dot'); ylabel('EoS: z_u dot, m/s'); xlabel('Time, s')

