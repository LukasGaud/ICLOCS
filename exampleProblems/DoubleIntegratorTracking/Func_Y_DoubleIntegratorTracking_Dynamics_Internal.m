% This code was generated using ADiGator version 1.4
% ©2010-2014 Matthew J. Weinstein and Anil V. Rao
% ADiGator may be obtained at https://sourceforge.net/projects/adigator/ 
% Contact: mweinstein@ufl.edu
% Bugs/suggestions may be reported to the sourceforge forums
%                    DISCLAIMER
% ADiGator is a general-purpose software distributed under the GNU General
% Public License version 3.0. While the software is distributed with the
% hope that it will be useful, both the software and generated code are
% provided 'AS IS' with NO WARRANTIES OF ANY KIND and no merchantability
% or fitness for any purpose or application.

function const = Func_Y_DoubleIntegratorTracking_Dynamics_Internal(X_in,U,p,t,data)
global ADiGator_Func_Y_DoubleIntegratorTracking_Dynamics_Internal
if isempty(ADiGator_Func_Y_DoubleIntegratorTracking_Dynamics_Internal); ADiGator_LoadData(); end
Gator1Data = ADiGator_Func_Y_DoubleIntegratorTracking_Dynamics_Internal.Func_Y_DoubleIntegratorTracking_Dynamics_Internal.Gator1Data;
% ADiGator Start Derivative Computations
%User Line: %userFunction_Adigator_noConst - Adigator template for user defined function (dynamics constraints) with direct collocation method (h-type)
%User Line: %
%User Line: % Syntax:  [ const ] = userFunction_Adigator_noConst( X_in,U,p,t,data)
%User Line: %
%User Line: % Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
%User Line: % The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
%User Line: % This code is published under the MIT License.
%User Line: % Department of Aeronautics and Department of Electrical and Electronic Engineering,
%User Line: % Imperial College London London  England, UK
%User Line: % ICLOCS (Imperial College London Optimal Control) Version 2.5
%User Line: % 1 Aug 2019
%User Line: % iclocs@imperial.ac.uk
%User Line: %------------- BEGIN CODE --------------
f=data.data.InternalDynamics;
%User Line: f=data.data.InternalDynamics;
vdat=data.data;
%User Line: vdat=data.data;
X.dY = X_in.dY; X.f = X_in.f;
%User Line: X=X_in;
cada1f1dY = X_in.dY(Gator1Data.Index1);
cada1f1 = X_in.f(1,:);
x0.dY = cada1f1dY;
x0.f = cada1f1.';
%User Line: x0=X_in(1,:)';
nt.f = data.sizes{1};
%User Line: nt=data.sizes{1};
n.f = data.sizes{3};
%User Line: n=data.sizes{3};
M.f = data.sizes{7};
%User Line: M=data.sizes{7};
ns.f = data.sizes{9};
%User Line: ns=data.sizes{9};
t0.dY = t.dY(1);
t0.f = t.f(1);
%User Line: t0=t(1);
cada1f1 = length(t.f);
tf.dY = t.dY(2);
tf.f = t.f(cada1f1);
%User Line: tf=t(end);
cada1td1 = zeros(2,1);
cada1td1(2) = tf.dY;
cada1td1(1) = cada1td1(1) + -t0.dY;
delta_t.dY = cada1td1;
delta_t.f = tf.f - t0.f;
%User Line: delta_t=tf-t0;
cada1f1 = [0;data.tau_inc];
cada1tempdY = delta_t.dY(Gator1Data.Index2);
cada1tf1 = cada1f1(Gator1Data.Index4);
cada1f2dY = cada1tf1(:).*cada1tempdY(Gator1Data.Index3);
cada1f2 = cada1f1*delta_t.f;
cada1f3dY = cada1f2dY./ns.f;
cada1f3 = cada1f2/ns.f;
cada1tempdY = t0.dY(Gator1Data.Index5);
cada1td1 = zeros(117,1);
cada1td1(Gator1Data.Index6) = cada1f3dY;
cada1td1(Gator1Data.Index7) = cada1td1(Gator1Data.Index7) + cada1tempdY;
T.dY = cada1td1;
T.f = cada1f3 + t0.f;
%User Line: T=[0;data.tau_inc]*delta_t/ns+t0;
P.f = repmat(p,59,1);
%User Line: P=repmat(p,M,1);
cada1f1dY = X.dY(Gator1Data.Index8);
cada1f1 = X.f(:,1);
cada1f2 = X.f(:,2);
cada1f3 = U.f(:,1);
cada1temp1 = Gator1Data.Data1;
cada1f4 = cada1temp1;
cada1f4(:,1) = cada1f2;
cada1td1 = zeros(118,1);
cada1td1(Gator1Data.Index9) = cada1f3dY;
cada1td1(Gator1Data.Index10) = cada1f4dY(Gator1Data.Index11);
dynF.dY = cada1td1;
dynF.f = cada1f4;
dynF.f(:,2) = cada1f3;
cada1f  = size(X_in.f);
cada1f  = size(U.f);
cada1f  = size(p);
cada1f  = size(t.f);
cada1f  = size(data);
cada1f  = size(vdat);
cada1f  = size(X.f);
cada1f  = size(x0.f);
cada1f  = size(nt.f);
cada1f  = size(n.f);
cada1f  = size(M.f);
cada1f  = size(ns.f);
cada1f  = size(t0.f);
cada1f  = size(tf.f);
cada1f  = size(delta_t.f);
cada1f  = size(T.f);
cada1f  = size(P.f);
cada1f  = size(X_in.f);
cada1f  = size(U.f);
cada1f  = size(p);
cada1f  = size(t.f);
cada1f  = size(data);
cada1f  = size(vdat);
cada1f  = size(X.f);
cada1f  = size(x0.f);
cada1f  = size(nt.f);
cada1f  = size(n.f);
cada1f  = size(M.f);
cada1f  = size(ns.f);
cada1f  = size(t0.f);
cada1f  = size(tf.f);
cada1f  = size(delta_t.f);
cada1f  = size(T.f);
cada1f  = size(P.f);
cada1f  = size(X_in.f);
cada1f1 = size(U.f);
cada1f2 = size(p);
cada1f3 = size(t.f);
cada1f4 = size(data);
cada1f5 = size(vdat);
cada1f6 = size(X.f);
cada1f7 = size(x0.f);
cada1f8 = size(nt.f);
cada1f9 = size(n.f);
cada1f10 = size(M.f);
cada1f11 = size(ns.f);
cada1f12 = size(t0.f);
cada1f13 = size(tf.f);
cada1f14 = size(delta_t.f);
cada1f15 = size(T.f);
cada1f16 = size(P.f);
%User Line: dynF=f(X,U,P,T,vdat);
cada1f17 = size(X_in.f);
cada1f18 = size(U.f);
cada1f19 = size(p);
cada1f20 = size(t.f);
X_vect.f = size(data);
cada1f1 = size(vdat);
cada1f2 = size(X.f);
cada1f3 = size(x0.f);
cada1f4 = size(nt.f);
cada1f5 = size(n.f);
cada1f6 = size(M.f);
cada1f7 = size(ns.f);
cada1f8 = size(t0.f);
cada1f9 = size(tf.f);
cada1f10 = size(delta_t.f);
cada1f11 = size(T.f);
cada1f12 = size(P.f);
cada1f13 = size(dynF.f);
cada1f14dY = X.dY(Gator1Data.Index12);
cada1f14 = X.f.';
cada1f15 = n.f*M.f;
cada1f16dY = cada1f14dY;
cada1f16 = reshape(cada1f14,cada1f15,1);
cada1f17 = size(X_in.f);
cada1f18 = size(U.f);
cada1f19 = size(p);
cada1f20 = size(t.f);
cada1f21 = size(data);
cada1f22 = size(vdat);
cada1f23 = size(X.f);
cada1f24 = size(x0.f);
cada1f25 = size(nt.f);
cada1f26 = size(n.f);
cada1f27 = size(M.f);
cada1f28 = size(ns.f);
const.f = size(t0.f);
%User Line: X_vect=reshape(X',n*M,1);
