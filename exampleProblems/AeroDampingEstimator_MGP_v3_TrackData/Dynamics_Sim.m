function dx = Dynamics_Sim(x,u,p,t,vdat)
%Double Integrator Dynamics for Simulation
%
% Syntax:  
%          [dx] = Dynamics(x,u,p,t,vdat)
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
ms = p(1,1); mu = p(1,2); kt = p(1,3); ct = p(1,4);

% Inputs
% u1 - ka;
% u2 - ca;
% u3 - ks;
% u4 - cs;
ka = u(:,1); ca = u(:,2); ks = u(:,3); cs = u(:,4); 

% Data to get the external disturbances
Fs = interp1(vdat.auxData.LF.tLap, vdat.auxData.LF.FzRear, t, 'linear', 'extrap');
zr = interp1(vdat.auxData.HF.tLap, vdat.auxData.HF.zRHeaveDot, t, 'linear', 'extrap');

dx(:,1) = x2 - zr;

dx(:,2) = (-ka./ms - ks./ms).*x1 + (-cs./ms - ca./ms).*x2 + (ks./ms).*x3 + (cs./ms).*x4 + Fs./ms;

dx(:,3) = x4 - zr;

dx(:,4) = (ks./mu).*x1 + (cs./mu).*x2 + (-ks./mu - kt./mu).*x3 + (-cs./mu - ct./mu).*x4 + (ct/mu)*zr;

%------------- END OF CODE --------------