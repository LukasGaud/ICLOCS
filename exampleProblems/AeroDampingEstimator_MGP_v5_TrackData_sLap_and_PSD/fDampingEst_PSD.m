function Cost = fDampingEst_PSD(controlVar, auxData, P, roadVec)
x1Guess = auxData.HF.RRHInterval;
x2Guess = auxData.HF.RRHDotInterval;
x3Guess = auxData.HF.xBumpRInterval;
x4Guess = auxData.HF.xBumpRDotInterval;
time= auxData.HF.intervalTime;
x0 = [x1Guess(1); x2Guess(1); x3Guess(1); x4Guess(1)];
kt = P(1);
ct = P(2);

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
    q2 = 1e-1;
    q3 = 2e7;
    q4 = 1e1;
    
    % errors
    e1 = abs(TrackX1 - ModelX1);
    e2 = abs(TrackX2 - ModelX2);
    e3 = abs(TrackX3 - ModelX3);
    e4 = abs(TrackX4 - ModelX4);

    Cost = q1*e1*(e1') + q2*e2*(e2') + q3*e3*(e3') + q4*e4*(e4');
    dX1 = abs(mean(x1Guess) - mean(y(:,1)));
    dX3 = abs(mean(x3Guess-x1Guess) - mean(y(:,3)-y(:,1)));
    Q1 = 1;
    Q3 = 10;
    Cost = Cost + (Q1*dX1 + Q3*dX3);
%     Cost = q1*e1*(e1');
%     Cost = q1*e1*(e1') + q3*e3*(e3');
else
    Cost = inf;
end

% [ModelX1Upper, ModelX1Lower] = envelope(y(:,1));
% [ModelX2Upper, ModelX2Lower] = envelope(y(:,2));
% [ModelX3Upper, ModelX3Lower] = envelope(y(:,3));
% [ModelX4Upper, ModelX4Lower] = envelope(y(:,4));
% 
% [TrackX1Upper, TrackX1Lower] = envelope(x1Guess);
% [TrackX2Upper, TrackX2Lower] = envelope(x2Guess);
% [TrackX3Upper, TrackX3Lower] = envelope(x3Guess);
% [TrackX4Upper, TrackX4Lower] = envelope(x4Guess);
% 
% qA1 = 1;
% dAmp = abs(max(abs(y(:,1))) - max(abs(x1Guess)));
% Cost = Cost + qA1*dAmp;