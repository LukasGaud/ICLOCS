function [dx] = Dynamics_Simple_Internal(x,u,p,t,vdat)
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

%------------- BEGIN CODE --------------
% States
% x1 = zs-zr;
% x2 = zsDot;
% x3 = zu-zr;
% x4 = zuDot;
x1 = x(:,1);x2 = x(:,2);

% Parameters
% p(1) = ms;
% p(2) = mu;
% p(3) = kt;
% p(4) = ct;
ms = p(1,1);

% Inputs
% u1 - ka;
% u2 - ca;
% u3 - ks;
% u4 - cs;
ka = u(:,1); ca = u(:,2);

% Data to get the external disturbances
Fs = interp1(vdat.auxData.HF.intervalTime, vdat.auxData.LF.FzRearInterval, t, 'linear', 'extrap');
zr = interp1(vdat.auxData.HF.intervalTime, vdat.auxData.HF.zRHeaveDotInterval, t, 'linear', 'extrap');

dx(:,1) = x2 - zr;

dx(:,2) = (-ka./ms).*x1 + (-ca./ms).*x2 +ca./ms.*zr + Fs./ms;


%------------- END OF CODE --------------