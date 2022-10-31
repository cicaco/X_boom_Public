%% Script per creare la Geometria 3D
%Main accoppiato con XFOIL senza GA
clear all 
close all
addpath(genpath('BLACKBOX'));
%% Input Data goemtrici 3D

p_c=15; % numero di profili di "Transizione" nella parte centrale
l=0.35; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
Chord=0.288/4.14;
delta=45.5*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=5; %Numero di profili totale su ciascuna metà;
PARA=1.3; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
%% Profilo 2D e caratteristiche aerodinamiche
Profile2D=importdata('NACA4412.dat');
Xp = Profile2D.data(:,1).*Chord-Chord;
Zp = Profile2D.data(:,2).*Chord-max(Profile2D.data(:,2).*Chord)/2;
Xp_flip = -(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Xp_flip = [Xp_flip(numel(Xp_flip)/2+1:end); Xp_flip(1:numel(Xp_flip)/2)];
Zp_flip = [Zp(numel(Xp_flip)/2+1:end); Zp(1:numel(Xp_flip)/2)];
%% Creazione di BoomInfo.s
BoomInfo.Pianta.l=l;
BoomInfo.Pianta.freccia=delta;
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;
BoomInfo.Geom3D.p_c=p_c;
BoomInfo.Geom3D.num=num;
BoomInfo.Geom3D.PARA=PARA;
BoomInfo.Mecc.Dens=345;
BoomInfo.Profile.Chord=Chord;
BoomInfo.Profile.Xp_sx=Xp;
BoomInfo.Profile.Xp_dx=fliplr(Xp_flip')';
BoomInfo.Profile.Zp_sx=Zp;
BoomInfo.Profile.Zp_dx=fliplr(Zp_flip')';
CheckBoomInfo(BoomInfo)
%Creazione della geometria tridemensionale
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Create_Stl');
%% Calcolo coefficienti aerodinamici per il profilo scelto
coeff360  = f_polar_360('load', 'NACA4412.dat',linspace(-10,16, 27), 50000,2,5, 0.007, 1.6);
CL_t      = coeff360.CL;
CD_t      = coeff360.CD;
CM_t      = coeff360.CM;
alpha_cl  = coeff360.alpha;
alpha_cd  = coeff360.alpha;
alpha_cm  = coeff360.alpha;
BoomInfo.Aero.alpha_cl=alpha_cl;
BoomInfo.Aero.alpha_cd=alpha_cd;
BoomInfo.Aero.alpha_cm=alpha_cm;
BoomInfo.Aero.Cl=CL_t;
BoomInfo.Aero.Cd=CD_t;
BoomInfo.Aero.Cm=CM_t;
%%
CheckBoomInfo(BoomInfo,'Plot')
%% Initial condition
fprintf('Starting Genetic Alghoritm...\n');
D=45*pi/180;
theta=0;
Chi=0.80;
lb=[5*100 45*100];
ub=[15*100 90*100];
fitnessfcn=@(x) GA_para_Chi(x,BoomInfo,Chi,D,theta);
options = optimoptions('ga', 'MaxStallGenerations', 10, 'MaxGenerations', 10, 'NonlinearConstraintAlgorithm', 'penalty',...
    'PopulationSize', 30, 'PlotFcn', {'gaplotbestindiv', 'gaplotbestf'},...
    'Display', 'iter', 'UseParallel', true, 'UseVectorized', false,'FitnessLimit',5);
X_ini = ga(fitnessfcn,5,[],[],[],[],lb,ub,[],1:5,options);

%% Initial condition
r0=X_ini(1)*2*pi/100;
phi=X_ini(2)*pi/180/100;
R=norm(BoomInfo.Aero.P_Finish_Dx);
Vs=r0*R*(1/Chi-1);
z0= 1.8;
[quat,ustart] = HandInitial(r0,theta,D,phi,Vs,BoomInfo);
tfin=40;
options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
%%
T0=quatToAtt(quat);
InitialConditionPlot(T0,ustart',[0;0;r0],BoomInfo);
tic
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo),[0 tfin],Y0,options); %
toc
[YOUT] = Eul_Quat(YOUT_quat,TOUT);
Energy(TOUT,YOUT,BoomInfo);
PlotTipDxSx(TOUT,YOUT,BoomInfo)
PlotAeroForce(YOUT,TOUT,BoomInfo)
ChiAvan(BoomInfo,YOUT,TOUT)
FinalReport(YOUT,TOUT)
