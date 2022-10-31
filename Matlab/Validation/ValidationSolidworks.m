%% Script per creare la Geometria 3D
% Codice di Confronto tra il modello geometrico e solidworks (file
% BoomerangValidatio.sldprt)
clear all
close all
addpath(genpath('BLACKBOX'));

%% Input Data
p_c=20; % numero di profili di "Transizione" nella parte centrale
l= 0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
Chord=0.06;
delta= 30*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=1.2; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V

%% Profilo 2D e caratteristiche aerodinamiche
Profile2D=importdata('Naca0012.dat');
Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;
Xp_flip=-(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Zp_flip=(Zp);
[n,~]=size(Xp);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
%% Creazione dell Info Box
BoomInfo.Pianta.l=l;
BoomInfo.Pianta.freccia=delta;
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;
BoomInfo.Geom3D.p_c=p_c;
BoomInfo.Geom3D.num=num;
BoomInfo.Geom3D.PARA=PARA;
BoomInfo.Mecc.Dens=1000;
BoomInfo.Profile.Chord=Chord;
BoomInfo.Profile.Xp_dx=Xp;
BoomInfo.Profile.Xp_sx=Xp_flip;
BoomInfo.Profile.Zp_dx=Zp;
BoomInfo.Profile.Zp_sx=Zp_flip;
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Create_Stl');
CG_sw=[ -0.10494375; -0.00234504; 0 ];
I_sw=[0.00506118 ; -0.00011051 ;0;  -0.00011051;  0.00045103 ;0; 0 ;0; 0.00551108];
m= 0.19074257;

num_vect=[5 10 20 40 100];
pc_vect=[5 10 20 40 100];
for i=1:5
    
    BoomInfo.Geom3D.p_c=pc_vect(i);
    BoomInfo.Geom3D.num=num_vect(i);
    [BoomInfo] = Boom3DShape(BoomInfo,'Info');
    I_vect(i)=norm(BoomInfo.Mecc.I_rho);
    CG_vect(i)=norm(BoomInfo.Mecc.CG);
    m(i)=BoomInfo.Mecc.m;
end
%%
figure()
subplot(1,3,1);

plot(num_vect,I_vect,'r','linewidth',1.2);
grid on
ylabel('norm(I)','fontsize',11,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
xlabel('n','fontsize',11,'interpreter','latex');
subplot(1,3,2);

plot(num_vect,CG_vect,'g','linewidth',1.2);
grid on
ylabel('norm(CG)','fontsize',11,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
xlabel('n','fontsize',11,'interpreter','latex');
subplot(1,3,3)
plot(num_vect,m,'b','linewidth',1.2);
grid on
ylabel('m','fontsize',11,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
xlabel('n','fontsize',11,'interpreter','latex');
sgtitle('Convergenza del modello geometrico','fontsize',11,'interpreter','latex')
