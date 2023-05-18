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

function const = Func_Y_Dynamics_Simple_Internal(X_in,U,p,t,data)
global ADiGator_Func_Y_Dynamics_Simple_Internal
if isempty(ADiGator_Func_Y_Dynamics_Simple_Internal); ADiGator_LoadData(); end
Gator1Data = ADiGator_Func_Y_Dynamics_Simple_Internal.Func_Y_Dynamics_Simple_Internal.Gator1Data;
% ADiGator Start Derivative Computations
%User Line: %userFunction_Adigator_noConst_Scaling - Adigator template for user defined function (dynamics constraints) with direct collocation method (h-type) and automatic scaling
%User Line: %
%User Line: % Syntax:  [ const ] = userFunction_Adigator_noConst_Scaling( X_in,U,p,t,data)
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
dimX1.f = size(X.f,1);
%User Line: dimX1=size(X,1);
dimX2.f = size(X.f,2);
%User Line: dimX2=size(X,2);
dimU1.f = size(U.f,1);
%User Line: dimU1=size(U,1);
dimU2.f = size(U.f,2);
%User Line: dimU2=size(U,2);
XshiftMat = data.scaling.XshiftMat;
%User Line: XshiftMat=data.scaling.XshiftMat;
UshiftMat = data.scaling.UshiftMat;
%User Line: UshiftMat=data.scaling.UshiftMat;
cada1f1dY = X.dY;
cada1f1 = X.f(:);
cada1td1 = sparse(Gator1Data.Index2,Gator1Data.Index3,cada1f1dY,8138,8138);
cada1td1 = data.scaling.XunscaleMat*cada1td1;
cada1td1 = cada1td1(:);
cada1f2dY = full(cada1td1(Gator1Data.Index4));
cada1f2 = data.scaling.XunscaleMat*cada1f1;
cada1f3 = XshiftMat(:);
X.dY = cada1f2dY;
X.f = cada1f2 - cada1f3;
%User Line: X=data.scaling.XunscaleMat*X(:)-XshiftMat(:);
cada1f1dY = U.dY;
cada1f1 = U.f(:);
cada1td1 = sparse(Gator1Data.Index5,Gator1Data.Index6,cada1f1dY,12207,12207);
cada1td1 = data.scaling.UunscaleMat*cada1td1;
cada1td1 = cada1td1(:);
cada1f2dY = full(cada1td1(Gator1Data.Index7));
cada1f2 = data.scaling.UunscaleMat*cada1f1;
cada1f3 = UshiftMat(:);
U.dY = cada1f2dY;
U.f = cada1f2 - cada1f3;
%User Line: U=data.scaling.UunscaleMat*U(:)-UshiftMat(:);
%User Line: % X=data.scaling.XunscaleMat*(X(:)-XshiftMat(:));
%User Line: % U=data.scaling.UunscaleMat*(U(:)-UshiftMat(:));
X.dY = X.dY;
X.f = reshape(X.f,dimX1.f,dimX2.f);
%User Line: X=reshape(X,dimX1,dimX2);
U.dY = U.dY;
U.f = reshape(U.f,dimU1.f,dimU2.f);
%User Line: U=reshape(U,dimU1,dimU2);
cadaconditional1 = isfield(data.data,'Pscale');
%User Line: cadaconditional1 = isfield(data.data,'Pscale');
    %User Line: %     p=(p-data.data.Pshift)./data.data.Pscale;
    cada1f1dY = p.dY./data.data.Pscale;
    cada1f1 = p.f/data.data.Pscale;
    p.dY = cada1f1dY;
    p.f = cada1f1 - data.data.Pshift;
    %User Line: p=p./data.data.Pscale-data.data.Pshift;
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
cada1tempdY = delta_t.dY(Gator1Data.Index8);
cada1tf1 = cada1f1(Gator1Data.Index10);
cada1f2dY = cada1tf1(:).*cada1tempdY(Gator1Data.Index9);
cada1f2 = cada1f1*delta_t.f;
cada1f3dY = cada1f2dY./ns.f;
cada1f3 = cada1f2/ns.f;
cada1tempdY = t0.dY(Gator1Data.Index11);
cada1td1 = zeros(8137,1);
cada1td1(Gator1Data.Index12) = cada1f3dY;
cada1td1(Gator1Data.Index13) = cada1td1(Gator1Data.Index13) + cada1tempdY;
T.dY = cada1td1;
T.f = cada1f3 + t0.f;
%User Line: T=[0;data.tau_inc]*delta_t/ns+t0;
P.dY = p.dY(Gator1Data.Index14);
P.f = repmat(p.f,4069,1);
%User Line: P=repmat(p,M,1);
cada1f1dY = X.dY(Gator1Data.Index15);
cada1f1 = X.f(:,1);
cada1f2dY = X.dY(Gator1Data.Index16);
cada1f2 = X.f(:,2);
cada1f3dY = P.dY(1);
cada1f3 = P.f(1,1);
cada1f4dY = U.dY(Gator1Data.Index17);
cada1f4 = U.f(:,1);
cada1f5dY = U.dY(Gator1Data.Index18);
cada1f5 = U.f(:,2);
cada1tf1 = ppval(Gator1Data.Data2,T.f);
cada1tf2 = cada1tf1(Gator1Data.Index19);
cada1f6dY = cada1tf2(:).*T.dY;
cada1f6 = ppval(Gator1Data.Data1,T.f);
cada1tf1 = ppval(Gator1Data.Data4,T.f);
cada1tf2 = cada1tf1(Gator1Data.Index20);
cada1f7dY = cada1tf2(:).*T.dY;
cada1f7 = ppval(Gator1Data.Data3,T.f);
cada1td1 = zeros(12206,1);
cada1td1(Gator1Data.Index21) = cada1f2dY;
cada1td1(Gator1Data.Index22) = cada1td1(Gator1Data.Index22) + -cada1f7dY;
cada1f8dY = cada1td1;
cada1f8 = cada1f2 - cada1f7;
cada1temp1 = Gator1Data.Data5;
cada1f9dY = cada1f8dY;
cada1f9 = cada1temp1;
cada1f9(:,1) = cada1f8;
cada1f10dY = -cada1f4dY;
cada1f10 = uminus(cada1f4);
cada1tempdY = cada1f3dY(Gator1Data.Index23);
cada1td1 = zeros(8138,1);
cada1td1(Gator1Data.Index24) = cada1f10dY./cada1f3;
cada1td1(Gator1Data.Index25) = cada1td1(Gator1Data.Index25) + -cada1f10(:)./cada1f3.^2.*cada1tempdY;
cada1f11dY = cada1td1;
cada1f11 = cada1f10/cada1f3;
cada1tf1 = cada1f1(Gator1Data.Index26);
cada1td1 = zeros(12207,1);
cada1td1(Gator1Data.Index27) = cada1tf1(:).*cada1f11dY;
cada1td1(Gator1Data.Index28) = cada1td1(Gator1Data.Index28) + cada1f11(:).*cada1f1dY;
cada1f12dY = cada1td1;
cada1f12 = cada1f11.*cada1f1;
cada1f13dY = -cada1f5dY;
cada1f13 = uminus(cada1f5);
cada1tempdY = cada1f3dY(Gator1Data.Index29);
cada1td1 = zeros(8138,1);
cada1td1(Gator1Data.Index30) = cada1f13dY./cada1f3;
cada1td1(Gator1Data.Index31) = cada1td1(Gator1Data.Index31) + -cada1f13(:)./cada1f3.^2.*cada1tempdY;
cada1f14dY = cada1td1;
cada1f14 = cada1f13/cada1f3;
cada1tf1 = cada1f2(Gator1Data.Index32);
cada1td1 = zeros(12207,1);
cada1td1(Gator1Data.Index33) = cada1tf1(:).*cada1f14dY;
cada1td1(Gator1Data.Index34) = cada1td1(Gator1Data.Index34) + cada1f14(:).*cada1f2dY;
cada1f15dY = cada1td1;
cada1f15 = cada1f14.*cada1f2;
cada1td1 = zeros(20345,1);
cada1td1(Gator1Data.Index35) = cada1f12dY;
cada1td1(Gator1Data.Index36) = cada1td1(Gator1Data.Index36) + cada1f15dY;
cada1f16dY = cada1td1;
cada1f16 = cada1f12 + cada1f15;
cada1tempdY = cada1f3dY(Gator1Data.Index37);
cada1td1 = zeros(8138,1);
cada1td1(Gator1Data.Index38) = cada1f5dY./cada1f3;
cada1td1(Gator1Data.Index39) = cada1td1(Gator1Data.Index39) + -cada1f5(:)./cada1f3.^2.*cada1tempdY;
cada1f17dY = cada1td1;
cada1f17 = cada1f5/cada1f3;
cada1tf1 = cada1f7(Gator1Data.Index40);
cada1td1 = zeros(16275,1);
cada1td1(Gator1Data.Index41) = cada1tf1(:).*cada1f17dY;
cada1tf1 = cada1f17(Gator1Data.Index42);
cada1td1(Gator1Data.Index43) = cada1td1(Gator1Data.Index43) + cada1tf1(:).*cada1f7dY;
cada1f18dY = cada1td1;
cada1f18 = cada1f17.*cada1f7;
cada1td1 = zeros(28482,1);
cada1td1(Gator1Data.Index44) = cada1f16dY;
cada1td1(Gator1Data.Index45) = cada1td1(Gator1Data.Index45) + cada1f18dY;
cada1f19dY = cada1td1;
cada1f19 = cada1f16 + cada1f18;
cada1tempdY = cada1f3dY(Gator1Data.Index46);
cada1td1 = zeros(12206,1);
cada1td1(Gator1Data.Index47) = cada1f6dY./cada1f3;
cada1td1(Gator1Data.Index48) = cada1td1(Gator1Data.Index48) + -cada1f6(:)./cada1f3.^2.*cada1tempdY;
cada1f20dY = cada1td1;
cada1f20 = cada1f6/cada1f3;
cada1td1 = cada1f19dY;
cada1td1(Gator1Data.Index49) = cada1td1(Gator1Data.Index49) + cada1f20dY;
cada1f21dY = cada1td1;
cada1f21 = cada1f19 + cada1f20;
cada1td1 = zeros(40688,1);
cada1td1(Gator1Data.Index50) = cada1f21dY;
cada1td1(Gator1Data.Index51) = cada1f9dY(Gator1Data.Index52);
dynF_org.dY = cada1td1;
dynF_org.f = cada1f9;
dynF_org.f(:,2) = cada1f21;
%User Line: [dynF_org]=f(X,U,P,T,vdat);
cada1f1dY = dynF_org.dY;
cada1f1 = dynF_org.f(:);
cada1td1 = sparse(Gator1Data.Index53,Gator1Data.Index54,cada1f1dY,8138,16279);
cada1td1 = data.scaling.XscaleMat*cada1td1;
cada1td1 = cada1td1(:);
dynF.dY = full(cada1td1(Gator1Data.Index55));
dynF.f = data.scaling.XscaleMat*cada1f1;
%User Line: dynF=data.scaling.XscaleMat*dynF_org(:);
cada1f1dY = dynF.dY;
cada1f1 = reshape(dynF.f,dimX1.f,dimX2.f);
dynF.dY = cada1f1dY(Gator1Data.Index56);
dynF.f = cada1f1.';
%User Line: dynF=reshape(dynF,dimX1,dimX2)';
cada1f1dY = X_in.dY(Gator1Data.Index57);
cada1f1 = X_in.f.';
cada1f2 = n.f*M.f;
X_vect.dY = cada1f1dY;
X_vect.f = reshape(cada1f1,cada1f2,1);
%User Line: X_vect=reshape(X_in',n*M,1);
cada1f1dY = x0.dY;
cada1f1 = x0.f - data.x0t;
cada1f2dY = data.cx0.*cada1f1dY;
cada1f2 = cada1f1*data.cx0;
cada1td1 = sparse(Gator1Data.Index58,Gator1Data.Index59,X_vect.dY,8138,8138);
cada1td1 = data.map.A*cada1td1;
cada1td1 = cada1td1(:);
cada1f3dY = full(cada1td1(Gator1Data.Index60));
cada1f3 = data.map.A*X_vect.f;
cada1tempdY = delta_t.dY(Gator1Data.Index61);
cada1tf1 = dynF.f(Gator1Data.Index62);
cada1td1 = zeros(40690,1);
cada1td1(Gator1Data.Index63) = cada1tf1(:).*cada1tempdY;
cada1td1(Gator1Data.Index64) = cada1td1(Gator1Data.Index64) + delta_t.f.*dynF.dY;
cada1f4dY = cada1td1;
cada1f4 = delta_t.f*dynF.f;
cada1f5 = M.f*n.f;
cada1f6dY = cada1f4dY;
cada1f6 = reshape(cada1f4,cada1f5,1);
cada1td1 = sparse(Gator1Data.Index65,Gator1Data.Index66,cada1f6dY,8138,16279);
cada1td1 = data.map.B*cada1td1;
cada1td1 = cada1td1(:);
cada1f7dY = full(cada1td1(Gator1Data.Index67));
cada1f7 = data.map.B*cada1f6;
cada1td1 = zeros(83394,1);
cada1td1(Gator1Data.Index68) = cada1f3dY;
cada1td1(Gator1Data.Index69) = cada1td1(Gator1Data.Index69) + cada1f7dY;
cada1f8dY = cada1td1;
cada1f8 = cada1f3 + cada1f7;
cada1td1 = zeros(83396,1);
cada1td1(Gator1Data.Index70) = cada1f2dY;
cada1td1(Gator1Data.Index71) = cada1f8dY;
const.dY = cada1td1;
const.f = [cada1f2;cada1f8];
%User Line: const=[(x0-data.x0t)*data.cx0;data.map.A*X_vect+data.map.B*reshape(delta_t*dynF,M*n,1)];
const.dY_size = [8138,20348];
const.dY_location = Gator1Data.Index72;
end


function ADiGator_LoadData()
global ADiGator_Func_Y_Dynamics_Simple_Internal
ADiGator_Func_Y_Dynamics_Simple_Internal = load('Func_Y_Dynamics_Simple_Internal.mat');
return
end