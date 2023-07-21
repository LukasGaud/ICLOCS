function fDampingCheck_PSD(controlVar, auxData, P, zrDot, roadVec)
x1Guess = auxData.HF.RRHInterval;
x2Guess = auxData.HF.RRHDotInterval;
x3Guess = auxData.HF.xBumpRInterval;
x4Guess = auxData.HF.xBumpRDotInterval;
time= auxData.HF.intervalTime;
x0 = [x1Guess(1); x2Guess(1); x3Guess(1); x4Guess(1)];
% kt = P(1);
% ct = P(2);

kt = controlVar(1);
ct = controlVar(2);

[track.x1_psd, track.y1_psd] = fnPSD(time, x1Guess, 'HighPassFreq', 2);
[track.x2_psd, track.y2_psd] = fnPSD(time, x2Guess, 'HighPassFreq', 2);
[track.x3_psd, track.y3_psd] = fnPSD(time, x3Guess, 'HighPassFreq', 2);
[track.x4_psd, track.y4_psd] = fnPSD(time, x4Guess, 'HighPassFreq', 2);

alpha = controlVar(5);
zrDot = abs(alpha)*roadVec;
% zrDot = roadVec;

[t, y] = ode45(@(t, x)Dynamics_Bouncing_Internal_Param_PSD(t, x, controlVar, P, auxData, zrDot), time, x0);

if size(y,1) == length(time)
    [model.x1_psd, model.y1_psd] = fnPSD(time, y(:,1), 'HighPassFreq', 2);
    [model.x2_psd, model.y2_psd] = fnPSD(time, (y(:,2) - zrDot), 'HighPassFreq', 2);
    [model.x3_psd, model.y3_psd] = fnPSD(time, (y(:,3) - y(:,1)), 'HighPassFreq', 2);
    [model.x4_psd, model.y4_psd] = fnPSD(time, (y(:,4) - y(:,2)), 'HighPassFreq', 2);
end

figure
subplot(2,2,1)
hold on
title('x1')
plot(track.x1_psd, track.y1_psd)
plot(model.x1_psd, model.y1_psd)
xlim([0, 10])

subplot(2,2,2)
hold on
title('x2')
plot(track.x2_psd, track.y2_psd)
plot(model.x2_psd, model.y2_psd)
xlim([0, 10])

subplot(2,2,3)
hold on
title('x3')
plot(track.x3_psd, track.y3_psd)
plot(model.x3_psd, model.y3_psd)
xlim([0, 10])

subplot(2,2,4)
hold on
title('x4')
plot(track.x4_psd, track.y4_psd)
plot(model.x4_psd, model.y4_psd)
xlim([0, 10])