function [dx] = Dynamics_Bouncing_Internal_Param(x,u,p,t,vdat)
% Double Integrator Dynamics - Internal
%
% Syntax:  
%          [dx] = Dynamics(x,u,p,t,vdat)	(Dynamics Only)
%          [dx,g_eq] = Dynamics(x,u,p,t,vdat)   (Dynamics and Eqaulity Path Constraints)
%          [dx,g_neq] = Dynamics(x,u,p,t,vdat)   (Dynamics and Inqaulity Path Constraints)
%          [dx,g_eq,g_neq] = Dynamics(x,u,p,t,vdat)   (Dynamics, Equality and Ineqaulity Path Constraints)
% 
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%    vdat - structured variable containing the values of additional data used inside
%          the function%      
% Output:
%    dx - time derivative of x
%    g_eq - constraint function for equality constraints
%    g_neq - constraint function for inequality constraints
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk

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
kt = p(1,1); ct = p(1,2); ka = p(1,3); ca = p(1,4);
% Inputs
zrDot = interp1(vdat.auxData.HF.intervalTime, vdat.auxData.Inputs.zrDot, t, 'linear', 'extrap');

% Data to get the external disturbances
Fs = interp1(vdat.auxData.HF.intervalTime, vdat.auxData.HF.FzRearInterval, t, 'linear', 'extrap');
% zr = interp1(vdat.auxData.HF.intervalTime, vdat.auxData.HF.zRHeaveDotInterval, t, 'linear', 'extrap');

% Getting ks and cs
ks = interp1(vdat.auxData.P.xBumpVec, vdat.auxData.P.ksVec, x3-x1, 'linear', 'extrap');
cs = interp1(vdat.auxData.P.vDamVec, vdat.auxData.P.csVec, x4-x2, 'linear', 'extrap');

dx(:,1) = x2 - zrDot;

dx(:,2) = (-ks./ms - ka./ms).*x1 + (-cs./ms - ca./ms).*x2 + (ks./ms).*x3 + (cs./ms).*x4 + Fs./ms + ca./ms.*zrDot;

dx(:,3) = x4 - zrDot;

dx(:,4) = (ks./mu).*x1 + (cs./mu).*x2 + (-ks./mu - kt./mu).*x3 + (-cs./mu - ct./mu).*x4 + (ct/mu)*zrDot;
%------------- END OF CODE --------------