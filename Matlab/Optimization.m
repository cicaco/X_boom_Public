%% Script per creare la Geometria 3D
clear all
close all
addpath(genpath('BLACKBOX'));
% Input Data
% Input Data
p_c=15; % numero di profili di "Transizione" nella parte centrale
l= 0.2930; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
Chord=l/4.62;
delta= 44.3*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=5; %Numero di profili totale su ciascuna metà;
PARA=1.2; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V

% Profilo 2D e caratteristiche aerodinamiche
Profile2D=importdata('fastcatch.dat');
Xp = Profile2D.data(:,1).*Chord-Chord;
Zp = Profile2D.data(:,2).*Chord-max(Profile2D.data(:,2).*Chord)/2;
Xp_flip = -(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Xp_flip = [Xp_flip(numel(Xp_flip)/2+1:end); Xp_flip(1:numel(Xp_flip)/2)];
Zp_flip = [Zp(numel(Xp_flip)/2+1:end); Zp(1:numel(Xp_flip)/2)];

% Creazione dell Info Box
BoomInfo.Pianta.l=l;
BoomInfo.Pianta.freccia=delta;
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;
BoomInfo.Geom3D.p_c=p_c;
BoomInfo.Geom3D.num=num;
BoomInfo.Geom3D.PARA=PARA;
BoomInfo.Mecc.Dens=6.327343750000000e+02;
BoomInfo.Profile.Chord=Chord;
BoomInfo.Profile.Xp_sx=fliplr(Xp')';
BoomInfo.Profile.Xp_dx=Xp_flip;
BoomInfo.Profile.Zp_sx=fliplr(Zp')';
BoomInfo.Profile.Zp_dx=Zp_flip;
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Create_Stl');
load fastcatch_360.mat
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

%
CheckBoomInfo(BoomInfo,'Plot','Text')
%% Initial Condition

theta=0*pi/180;
D=0*pi/180;
Chi=0.85;
%% Genetic Alghortim (1° Optimization)
% lb = [30*10,0.15*1000,4*100];
% ub = [60*10,0.3*1000,8*100];
% Fun_A= @(x) GA_Spot(x,BoomInfo,D,theta,Chi);
% options = optimoptions('ga', 'MaxStallGenerations', 10, 'MaxGenerations', 10, 'NonlinearConstraintAlgorithm', 'penalty',...
%     'PopulationSize', 30, 'PlotFcn', {'gaplotbestindiv', 'gaplotbestf'},...
%     'Display', 'iter', 'UseParallel', true, 'UseVectorized', false);
% X_fin_1 = ga(Fun_A,3,[],[],[],[],lb,ub,[],1:3,options);
%% Somplex Method (2° Optimization)
% x0=[43.7 0.286 4.73];
% lb = [30,0.15,4];
% ub = [60,0.3,8];
% Fun_A_constrained= @(x) GA_Spot_constrained(x,lb,ub,BoomInfo,D,theta,Chi);
% options = optimset('Display','iter','PlotFcns',@optimplotfval);
% X_fin_2 = fminsearch(Fun_A_constrained,x0,options)
%% Simplex Method (3° Optimization)
% x0=[650];
% lb = [200];
% ub = [1000];
% Fun_mass= @(x) GA_mass(x,lb,ub,BoomInfo,D,theta,Chi);
% options = optimset('Display','iter','PlotFcns',@optimplotfval);
% Dens_Opt = fminsearch(Fun_mass,x0,options)
%%
% X_fin_1 = [43.7000000000000	0.286000000000000	4.7300000000000]; %Result
% of GA (1° optimization)
X_fin = [45.573878982296918   0.280322920216553   4.904364778342789];
% Result of Simplex method (2° optimization)
% Dens_Opt=     6.327343750000000e+02;
BoomInfo.Profile.Xp_sx=BoomInfo.Profile.Xp_sx./BoomInfo.Profile.Chord;
BoomInfo.Profile.Xp_dx=BoomInfo.Profile.Xp_dx./BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_sx=BoomInfo.Profile.Zp_sx./BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_dx=BoomInfo.Profile.Zp_dx./BoomInfo.Profile.Chord;
BoomInfo.Pianta.freccia=X_fin(1)*pi/180;
BoomInfo.Pianta.l=X_fin(2);
BoomInfo.Profile.Chord=BoomInfo.Pianta.l/(X_fin(3));
%divido per la nuova corda
BoomInfo.Profile.Xp_sx=BoomInfo.Profile.Xp_sx.*BoomInfo.Profile.Chord;
BoomInfo.Profile.Xp_dx=BoomInfo.Profile.Xp_dx.*BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_sx=BoomInfo.Profile.Zp_sx.*BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_dx=BoomInfo.Profile.Zp_dx.*BoomInfo.Profile.Chord;
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Create_Stl');

[A] = StabilityCheck(BoomInfo,D,theta,Chi);
%%
tfin=5;
z0= 1.8; % initial altitude
r0=13.7914*2*pi;
phi=88.3779*pi/180;
R=norm(BoomInfo.Aero.P_Finish_Dx);

Vs=r0*R*(1/Chi-1);
[quat,ustart] = HandInitial(r0,theta,D,phi,Vs,BoomInfo);

options = odeset('Events', @EventsAntiSheronQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';

tic
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo),[0 tfin],Y0,options); %
toc


[YOUT] = Eul_Quat(YOUT_quat,TOUT);
%Energy(TOUT,YOUT,BoomInfo);
PlotTipDxSx(TOUT,YOUT,BoomInfo)
%PlotAeroForce(YOUT,TOUT,BoomInfo)
%ChiAvan(BoomInfo,YOUT,TOUT)
FinalReport(YOUT,TOUT);
%%
figure(10)
title('Condizioni di lancio: Forma Ottimizzata Finale','fontsize',12,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
xlabel('$r_{0}$ [Hz]','fontsize',11,'interpreter','latex');
ylabel('$\phi$','fontsize',11,'interpreter','latex');