function [dx] = Dynamics_Internal_stochRoad(x, u, p, t, vdat)
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%
%------------- BEGIN CODE --------------
% States
% x1 = zs-zr;
% x2 = zsDot;
% x3 = zu-zr;
% x4 = zuDot;
x1 = x(:,1);x2 = x(:,2); x3 = x(:,3); x4 = x(:,4);

% Parameters
% p(1) = ms;
% p(2) = mu;
% p(3) = kt;
% p(4) = ct;
ms = vdat.auxData.P.ms;
mu = vdat.auxData.P.mu;
% ct = vdat.auxData.P.ct;
% kt = vdat.auxData.P.kt;
% kt = p(1,1); ct = p(1,2);

% ms = p(1,1); mu = p(1,2); kt = p(1,3); ct = p(1,4); 
% Ps = p(1,5); Pt = p(1,6); Pa = p(1,7);
% kt = p(1); ct = p(2); ka = u(1); ca = u(2);
kt = p(1,1); ct = p(1,2); alpha = p(1,3);
ka = u(:,1); ca = u(:,2); Fs = u(:,3);
% Inputs
zrDot = alpha*interp1(vdat.auxData.HF.intervalTime, vdat.auxData.HF.roadVecInterval, t, 'linear', 'extrap');

% Data to get the external disturbances
% Fs = interp1(vdat.auxData.HF.intervalTime, vdat.auxData.HF.FzRearInterval, t, 'linear', 'extrap');
% Fs = 0;
% zr = interp1(vdat.auxData.HF.intervalTime, vdat.auxData.HF.zRHeaveDotInterval, t, 'linear', 'extrap');

% Getting ks and cs
ks = interp1(vdat.auxData.P.xBumpVec, vdat.auxData.P.ksVec, x3-x1, 'linear', 'extrap');
cs = interp1(vdat.auxData.P.vDamVec, vdat.auxData.P.csVec, x4-x2, 'linear', 'extrap');

dx(:,1) = x2 - zrDot;

dx(:,2) = (-ks./ms - ka./ms).*x1 + (-cs./ms - ca./ms).*x2 + (ks./ms).*x3 + (cs./ms).*x4 + Fs./ms + ca./ms.*zrDot;

dx(:,3) = x4 - zrDot;

dx(:,4) = (ks./mu).*x1 + (cs./mu).*x2 + (-ks./mu - kt./mu).*x3 + (-cs./mu - ct./mu).*x4 + (ct/mu)*zrDot;
%------------- END OF CODE --------------