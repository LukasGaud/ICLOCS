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
% f3 = figure;
% subplot(2,2,1)
% hold on
% Axf31 = gca;
% ylabel(Axf31, 'k_a')
% title(Axf31, 'Estimated k_a & c_a in RRH domain')
% 
% subplot(2,2,2);
% hold on
% Axf32 = gca;
% ylabel(Axf32, 'k_a')
% title(Axf32, 'Estimated k_a & c_a in dot(RRH) domain')
% 
% subplot(2,2,3);
% hold on
% Axf33 = gca;
% ylabel(Axf33, 'c_a')
% xlabel(Axf33, 'RRH: (z_s - z_r)')
% subplot(2,2,4);
% hold on
% Axf34 = gca;
% ylabel(Axf34, 'c_a')
% xlabel(Axf34, 'RRH Velocity: dot(z_s - z_r)')

colors = ['g', 'b', 'r', 'm', 'c', "#7E2F8E"];
colorsRGB = [
    0.9290, 0.6940, 0.1250;
    0.4940, 0.1840, 0.5560;
    0.4660, 0.6740, 0.1880;
    0.3010, 0.7450, 0.9330;
    0.6350, 0.0780, 0.1840;
    0, 0.4470, 0.7410;
    0.8, 0.8, 0.8;
    0.3, 0.8, 0.7];
linestyle = {'-', '--', ':', '-.', '-', '--', ':', '-.'};
leg1 = {};
leg2 = {};
leg3 = {};
leg4 = {};
leg5 = {};

index1 = 1;
index2 = 1;
index3 = 1;
index4 = 1;
index4leg = 1;
index5 = 1;
index6 = 1;

i = 1;
for i = 1:size(problem,2)
    
    % Aero Damping and Stiffness Compare
    ksDef = interp1(problem{i}.data.auxData.P.xBumpVec, problem{i}.data.auxData.P.ksVec, solution{i}.X(:,3)-solution{i}.X(:,1), 'spline', 'extrap');
    csDef = interp1(problem{i}.data.auxData.P.vDamVec, problem{i}.data.auxData.P.csVec, solution{i}.X(:,4)-solution{i}.X(:,2), 'spline', 'extrap');
    
    % Outing Parameters
    Driver  = problem{i}.data.auxData.P.driver;
    Lap = mean(problem{i}.data.auxData.P.NLap);
    sLapStart = problem{i}.data.auxData.P.sLapStart;
    sLapEnd = problem{i}.data.auxData.P.sLapEnd;
%     index1 = 1;
%     index2 = 1;
    indexleg1 = 1;
    indexleg2 = 1;
    % Figure 1
    figure
    subplot(2,1,1)
    hold on
    Ax = gca;
    % ca
    plot(Ax, solution{i}.T, solution{i}.U(:,2), 'LineWidth', 2, 'LineStyle', linestyle(index1), 'Color', colorsRGB(index1, :))
    leg1{indexleg1} = ['ca - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index1 = index1 + 1;
    indexleg1 = indexleg1 + 1;

    % ca - PSD
    plot(Ax, solutionPSD{i}.T, solutionPSD{i}.U(:,2), 'LineWidth', 2, 'LineStyle', linestyle(index1), 'Color', colorsRGB(index1, :))
    leg1{indexleg1} = ['ca PSD - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index1 = index1 + 1;
    indexleg1 = indexleg1 + 1;

    % ca - Const
    plot(Ax, solutionConst{i}.T, solutionConst{i}.p(5)*ones(length(solutionConst{i}.T),1), 'LineWidth', 2, 'LineStyle', linestyle(index1), 'Color', colorsRGB(index1, :))
    leg1{indexleg1} = ['ca Const - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index1 = index1 + 1;
    indexleg1 = indexleg1 + 1;

    % cs
    plot(Ax, solution{i}.T, csDef, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'k')
    leg1{indexleg1} = ['cs - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index1 = index1 + 1;
    indexleg1 = indexleg1 + 1;
    legend(Ax,leg1);
    title(Ax, ['Estimated Stiffness and Damping Compare for: ', Driver, ' L', num2str(Lap)])
    ylabel(Ax, 'Damping Terms')
    
    subplot(2,1,2)
    hold on
    Ax = gca;
    % ka
    plot(Ax, solution{i}.T, solution{i}.U(:,1), 'LineWidth', 2, 'LineStyle', linestyle(index2), 'Color', colorsRGB(index2,:))
    leg2{indexleg2} = ['ka - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index2 = index2 + 1;
    indexleg2 = indexleg2 + 1;

    % ka - PSD
    plot(Ax, solutionPSD{i}.T, solutionPSD{i}.U(:,1), 'LineWidth', 2, 'LineStyle', linestyle(index2), 'Color', colorsRGB(index2,:))
    leg2{indexleg2} = ['ka - PSD - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index2 = index2 + 1;
    indexleg2 = indexleg2 + 1;

    % ka - Const
    plot(Ax, solutionConst{i}.T, solutionConst{i}.p(4)*ones(length(solutionConst{i}.T),1), 'LineWidth', 2, 'LineStyle', linestyle(index2), 'Color', colorsRGB(index2,:))
    leg2{indexleg2} = ['ka Const - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    index2 = index2 + 1;
    indexleg2 = indexleg2 + 1;

    % ks
    plot(Ax, solution{i}.T, ksDef, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'k')
    leg2{indexleg2} = ['ks - ', Driver, ' L', num2str(Lap), ' sLap:', num2str(sLapStart), ':', num2str(sLapEnd)];
    legend(Ax,leg2);
    xlabel(Ax, 'Time, s')
    ylabel(Ax, 'Stiffness Terms')


    % Figure 2
    % vCar
    plot(Axf21, problem{i}.data.auxData.HF.intervalTime, problem{i}.data.auxData.HF.vCar(problem{i}.data.auxData.HF.interval), 'LineWidth', 2, 'LineStyle', linestyle(index3), 'Color', colorsRGB(index3, :))
    
    % RRH
    plot(Axf23, problem{i}.data.auxData.HF.intervalTime, (problem{i}.data.auxData.HF.RRHInterval + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, 'LineWidth', 2, 'LineStyle', linestyle(index3), 'Color', colorsRGB(index3, :))
    
    % Fz
    plot(Axf22, problem{i}.data.auxData.HF.intervalTime, problem{i}.data.auxData.HF.FzRearInterval, 'LineWidth', 2, 'LineStyle', linestyle(index3), 'Color', colorsRGB(index3, :))

    % Road Displacement
    zr = cumtrapz(problem{i}.data.auxData.HF.intervalTime, problem{i}.data.auxData.HF.roadVecInterval);
    plot(Axf24, problem{i}.data.auxData.HF.intervalTime, zr*1000, 'LineWidth', 2, 'LineStyle', linestyle(index3), 'Color', colorsRGB(index3, :))
    
    leg3{index3} = [Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
    index3 = index3 + 1;
    legend(Axf24, leg3)


    % Figure 3
    index4leg = 1;
    figure
    subplot(2,2,1)
    hold on
    % ka vs RRH
    plot((solution{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solution{i}.U(:,1), '.', 'MarkerSize', 5, 'Color', colorsRGB(index4, :))
    plot((solution{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solution{i}.U(:,1), 'LineWidth', 1, 'Color', [colorsRGB(index4, :), 0.1]);
    % PSD parameters
    plot((solutionPSD{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solutionPSD{i}.U(:,1), '.', 'MarkerSize', 5, 'Color', colorsRGB(index4+1, :))
    plot((solutionPSD{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solutionPSD{i}.U(:,1), 'LineWidth', 1, 'Color', [colorsRGB(index4+1, :), 0.1]);
    
    rangeRRH = 14:1:26;
    rangeRRHDot = -0.3:0.1:0.3;
    plot(rangeRRH, solutionConst{i}.p(4)*ones(length(rangeRRH),1), 'LineStyle', '--', 'LineWidth', 0.5, 'Color', 'k');
   
    ylabel('k_a')
    title('Estimated k_a & c_a in RRH domain')

    subplot(2,2,3)
    hold on
    % ca vs RRH
    plot((solution{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solution{i}.U(:,2), '.', 'MarkerSize', 5,  'Color', colorsRGB(index4, :))
    plot((solution{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solution{i}.U(:,2), 'LineWidth', 1, 'Color', [colorsRGB(index4, :), 0.1]);
    
    % PSD parameters
    plot((solutionPSD{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solutionPSD{i}.U(:,2), '.', 'MarkerSize', 5,  'Color', colorsRGB(index4+1, :))
    plot((solutionPSD{i}.X(:,1) + mean(problem{i}.data.auxData.HF.RRHSetup))*1000, solutionPSD{i}.U(:,2), 'LineWidth', 1, 'Color', [colorsRGB(index4+1, :), 0.1]);

    plot(rangeRRH, solutionConst{i}.p(5)*ones(length(rangeRRH),1), 'LineStyle', '--', 'LineWidth', 0.5, 'Color', 'k');
    ylabel('c_a')
    xlabel('RRH: (z_s - z_r)')
    
   
    subplot(2,2,2)
    hold on
    % ka vs RRH_Dot
    plot(solution{i}.X(:,2), solution{i}.U(:,1), '.', 'MarkerSize', 5, 'Color', colorsRGB(index4, :))
    plot(solution{i}.X(:,2), solution{i}.U(:,1), 'LineWidth', 1, 'Color', [colorsRGB(index4, :), 0.1]);
    
    % PSD parameters
    plot(solutionPSD{i}.X(:,2), solutionPSD{i}.U(:,1), '.', 'MarkerSize', 5, 'Color', colorsRGB(index4+1, :))
    plot(solutionPSD{i}.X(:,2), solutionPSD{i}.U(:,1), 'LineWidth', 1, 'Color', [colorsRGB(index4+1, :), 0.1]);

    plot(rangeRRHDot, solutionConst{i}.p(4)*ones(length(rangeRRHDot),1), 'LineStyle', '--', 'LineWidth', 0.5, 'Color', 'k');
    ylabel('k_a')
    title('Estimated k_a & c_a in dot(RRH) domain')

    subplot(2,2,4)
    hold on
    % ca vs RRH_Dot
    pScatter(index4leg) = plot(solution{i}.X(:,2), solution{i}.U(:,2), '.', 'MarkerSize', 5, 'Color', colorsRGB(index4, :));
    plot(solution{i}.X(:,2), solution{i}.U(:,2), 'LineWidth', 1, 'Color', [colorsRGB(index4, :), 0.1]);
    leg4{index4leg} = [Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
    index4 = index4 + 1;
    index4leg = index4leg + 1;

    pScatter(index4leg) = plot(solutionPSD{i}.X(:,2), solutionPSD{i}.U(:,2), '.', 'MarkerSize', 5, 'Color', colorsRGB(index4, :));
    plot(solution{i}.X(:,2), solutionPSD{i}.U(:,2), 'LineWidth', 1, 'Color', [colorsRGB(index4, :), 0.1]);
    leg4{index4leg} = [Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd), ' PSD'];
    index4leg = index4leg + 1;

    pScatter(index4leg) = plot(rangeRRHDot, solutionConst{i}.p(5)*ones(length(rangeRRHDot),1), 'LineStyle', '--', 'LineWidth', 0.5, 'Color', 'k');
    leg4{index4leg} = [Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd), ' Const'];
    index4 = index4 + 1;
    index4leg = index4leg + 1;
    ylabel('c_a')
    xlabel('Bump Velocity: Dot(z_s - z_r)')
    legend(pScatter, leg4)

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
    plot(Axf41, solution{i}.T, (solution{i}.X(:,1)+ mean(problem{i}.data.auxData.HF.RRHSetup))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5,:))
    % z_s dot
    plot(Axf42, solution{i}.T, solution{i}.X(:,2)*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5,:))
    % z_u - z_s
    plot(Axf43, solution{i}.T, (solution{i}.X(:,3) - solution{i}.X(:,1))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5,:))
    % (z_u - z_s) dot
    plot(Axf44, solution{i}.T, (solution{i}.X(:,4) - solution{i}.X(:,2))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5,:))
    
    leg5{index5} = ['Model Free - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
    index5 = index5 + 1;
%     
    % Model PSD
    % z_s-z_r
    plot(Axf41, solutionPSD{i}.T, (solutionPSD{i}.X(:,1)+ mean(problem{i}.data.auxData.HF.RRHSetup))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5,:))
    % z_s dot
    plot(Axf42, solutionPSD{i}.T, solutionPSD{i}.X(:,2)*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5, :))
    % z_u - z_s
    plot(Axf43, solutionPSD{i}.T, (solutionPSD{i}.X(:,3) - solutionPSD{i}.X(:,1))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5,:))
    % (z_u - z_s) dot
    plot(Axf44, solutionPSD{i}.T, (solutionPSD{i}.X(:,4) - solutionPSD{i}.X(:,2))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5,:))
    
    leg5{index5} = ['Model PSD - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
    index5 = index5 + 1;

    % Model Constant Values
    % z_s-z_r
    plot(Axf41, solutionConst{i}.T, (solutionConst{i}.X(:,1)+ mean(problemConst{i}.data.auxData.HF.RRHSetup))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5,:))
    % z_s dot
    plot(Axf42, solutionConst{i}.T, solutionConst{i}.X(:,2)*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5,:))
    % z_u - z_s
    plot(Axf43, solutionConst{i}.T, (solutionConst{i}.X(:,3) - solutionConst{i}.X(:,1))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5,:))
    % (z_u - z_s) dot
    plot(Axf44, solutionConst{i}.T, (solutionConst{i}.X(:,4) - solutionConst{i}.X(:,2))*1000, 'LineWidth', 2, 'LineWidth', 2, 'LineStyle', linestyle(index5), 'Color', colorsRGB(index5,:))
    
    leg5{index5} = ['Model Const - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
    index5 = index5 + 1;
    %Track
    % z_s-z_r
    plot(Axf41, problem{i}.data.auxData.HF.intervalTime, (problem{i}.data.auxData.HF.RRHInterval+ mean(problem{i}.data.auxData.HF.RRHSetup))*1000, 'LineWidth', 2, 'LineStyle', '-', 'Color', 'k')
    % z_s dot
    plot(Axf42, problem{i}.data.auxData.HF.intervalTime, (problem{i}.data.auxData.HF.RRHDotInterval + solution{i}.p(3)*problem{i}.data.auxData.HF.roadVecInterval')*1000, 'LineWidth', 2, 'LineStyle', '-', 'Color', 'k')
    % z_u - z_s
    plot(Axf43, problem{i}.data.auxData.HF.intervalTime, (problem{i}.data.auxData.HF.xBumpRInterval*1000), 'LineWidth', 2, 'LineStyle', '-', 'Color', 'k')
    % (z_u - z_s) dot
    plot(Axf44, problem{i}.data.auxData.HF.intervalTime, (problem{i}.data.auxData.HF.xBumpRDotInterval*1000), 'LineWidth', 2, 'LineStyle', '-', 'Color', 'k')
    
    leg5{index5} = ['Track - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)];
    index5 = index5 + 1;
    legend(Axf44, leg5)
    
    % Fz Figure
    figure
    hold on
    plot(solution{i}.T, solution{i}.U(:,3), 'LineWidth', 2, 'LineStyle', linestyle(index3), 'Color', colorsRGB(1, :))
    plot(solutionPSD{i}.T, solutionConst{i}.U(:,1), 'LineWidth', 2, 'LineStyle', linestyle(1), 'Color', colorsRGB(2, :))
    plot(solutionConst{i}.T, solutionPSD{i}.U(:,3), 'LineWidth', 2, 'LineStyle', linestyle(1), 'Color', colorsRGB(3, :))
    plot(problem{i}.data.auxData.HF.intervalTime, problem{i}.data.auxData.HF.FzRearInterval, 'LineWidth', 2, 'LineStyle', linestyle(1), 'Color', 'k')
    xlabel('Time, s')
    ylabel('Fz, N')
    title('Aerodynamic Force')
    legend({ ...
        ['Model Free - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)],...
        ['Model Const - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)],...
        ['Model PSD - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)],...
        ['Track - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)], ...
        })
    
    % PSD plots
    figure
    index6 = 1;
    subplot(4,1,1)
    hold on
    Axf61 = gca;
    title(Axf61, 'PSD Compare')
    ylabel(Axf61, 'RRH: z_s - z_r, mm')
    xlim([0,10])
    subplot(4,1,2)
    hold on
    Axf62 = gca;
    ylabel(Axf62, 'dot(RRH) - dot(z_r): dot(z_s), mm/s')
    xlim([0,10])
    subplot(4,1,3);
    hold on
    Axf63 = gca;
    ylabel(Axf63, 'Bump Motion: (z_u - z_s), mm')
    xlim([0,10])
    subplot(4,1,4);
    hold on
    Axf64 = gca;
    ylabel(Axf64, 'Bump Velocity: dot(z_u - z_s), mm/s')
    xlabel('Frequency, Hz')
    xlim([0,10])
    % Model - Free
    [model_free.x1_psd, model_free.y1_psd] = fnPSD(solution{i}.T, solution{i}.X(:,1), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model_free.x2_psd, model_free.y2_psd] = fnPSD(solution{i}.T, solution{i}.X(:,2), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model_free.x3_psd, model_free.y3_psd] = fnPSD(solution{i}.T, solution{i}.X(:,3) - solution{i}.X(:,1), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model_free.x4_psd, model_free.y4_psd] = fnPSD(solution{i}.T, solution{i}.X(:,4) - solution{i}.X(:,2), 'HighPassFreq', 2, 'HighPassOrder', 3);
    
    % Model - Const
    [model_const.x1_psd, model_const.y1_psd] = fnPSD(solutionConst{i}.T, solutionConst{i}.X(:,1), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model_const.x2_psd, model_const.y2_psd] = fnPSD(solutionConst{i}.T, solutionConst{i}.X(:,2), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model_const.x3_psd, model_const.y3_psd] = fnPSD(solutionConst{i}.T, solutionConst{i}.X(:,3) - solutionConst{i}.X(:,1), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model_const.x4_psd, model_const.y4_psd] = fnPSD(solutionConst{i}.T, solutionConst{i}.X(:,4) - solutionConst{i}.X(:,2), 'HighPassFreq', 2, 'HighPassOrder', 3);
    
    % Model - PSD
    [model_psd.x1_psd, model_psd.y1_psd] = fnPSD(solutionPSD{i}.T, solutionPSD{i}.X(:,1), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model_psd.x2_psd, model_psd.y2_psd] = fnPSD(solutionPSD{i}.T, solutionPSD{i}.X(:,2), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model_psd.x3_psd, model_psd.y3_psd] = fnPSD(solutionPSD{i}.T, solutionPSD{i}.X(:,3) - solutionPSD{i}.X(:,1), 'HighPassFreq', 2, 'HighPassOrder', 3);
    [model_psd.x4_psd, model_psd.y4_psd] = fnPSD(solutionPSD{i}.T, solutionPSD{i}.X(:,4) - solutionPSD{i}.X(:,2), 'HighPassFreq', 2, 'HighPassOrder', 3);

    % Track
    [track.x1_psd, track.y1_psd] = fnPSD(problem{i}.data.auxData.HF.intervalTime, problem{i}.data.auxData.HF.RRHInterval, 'HighPassFreq', 2, 'HighPassOrder', 3);
    [track.x2_psd, track.y2_psd] = fnPSD(problem{i}.data.auxData.HF.intervalTime, problem{i}.data.auxData.HF.RRHDotInterval + solution{i}.p(3)*problem{i}.data.auxData.HF.roadVecInterval', 'HighPassFreq', 2, 'HighPassOrder', 3);
    [track.x3_psd, track.y3_psd] = fnPSD(problem{i}.data.auxData.HF.intervalTime, problem{i}.data.auxData.HF.xBumpRInterval, 'HighPassFreq', 2, 'HighPassOrder', 3);
    [track.x4_psd, track.y4_psd] = fnPSD(problem{i}.data.auxData.HF.intervalTime, problem{i}.data.auxData.HF.xBumpRDotInterval, 'HighPassFreq', 2, 'HighPassOrder', 3);

    % Plots
    plot(Axf61, model_free.x1_psd, model_free.y1_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6, :))
    plot(Axf62, model_free.x2_psd, model_free.y2_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6, :))
    plot(Axf63, model_free.x3_psd, model_free.y3_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6, :))
    plot(Axf64, model_free.x4_psd, model_free.y4_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6, :))

    plot(Axf61, model_const.x1_psd, model_const.y1_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6+1, :))
    plot(Axf62, model_const.x2_psd, model_const.y2_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6+1, :))
    plot(Axf63, model_const.x3_psd, model_const.y3_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6+1, :))
    plot(Axf64, model_const.x4_psd, model_const.y4_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6+1, :))

    plot(Axf61, model_psd.x1_psd, model_psd.y1_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6+2, :))
    plot(Axf62, model_psd.x2_psd, model_psd.y2_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6+2, :))
    plot(Axf63, model_psd.x3_psd, model_psd.y3_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6+2, :))
    plot(Axf64, model_psd.x4_psd, model_psd.y4_psd, 'LineWidth', 2, 'LineStyle', linestyle(index6), 'Color', colorsRGB(index6+2, :))

    plot(Axf61, track.x1_psd, track.y1_psd, 'LineWidth', 2, 'LineStyle', '-', 'Color', 'k')
    plot(Axf62, track.x2_psd, track.y2_psd, 'LineWidth', 2, 'LineStyle', '-', 'Color', 'k')
    plot(Axf63, track.x3_psd, track.y3_psd, 'LineWidth', 2, 'LineStyle', '-', 'Color', 'k')
    plot(Axf64, track.x4_psd, track.y4_psd, 'LineWidth', 2, 'LineStyle', '-', 'Color', 'k')
    index6 = index6 + 3;
    legend(Axf64, { ...
        ['Model Free - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)],...
        ['Model Const - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)],...
        ['Model PSD - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)],...
        ['Track - ', Driver, ' L', num2str(Lap), ' sLap: ', num2str(sLapStart), ':', num2str(sLapEnd)], ...
        })
end

% All states compare - EoS


