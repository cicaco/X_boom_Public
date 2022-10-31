function [F,M,AoA1,AoA2,Re,Mn]=AeroDynamics_FAST(u,omega,BoomInfo)
%% AeroDynamics computes the force and momentum for the whole boomerang
% and the angle of attack and Reynolds for each section spanwise
%
% [F,M,AoA1,AoA2,Re,Mn]=AeroDynamics(u,omega,BoomInfo) computes the quantity described as
% function of u and omega
%
% INPUT:
% u velocity in the body frame
% omega angular speed in the body frame
%
% OUTPUT:
% F: vector of aerodynamic forces in body frame
% M: vector of aerodynamic moments in body frame
% AoAsx: Angles of Attack of each section of left blade
% AoAdx: Angles of Attack of each section of right blade
% Re: Reynolds number of a charateristic blade section at 3/4 Length of blade

%% import geometria
c=BoomInfo.Profile.Chord; %m
L=BoomInfo.Pianta.l;
freccia=BoomInfo.Pianta.freccia;
coning=BoomInfo.Pianta.diedro; %(rot asse x2)
pitch=BoomInfo.Pianta.pitch; %(rot asse y3)
%pala sx (denominata 1)
xac1=BoomInfo.Aero.P_origin_Sx(1); %posizione centro aerodinamico pala 1
start1=BoomInfo.Aero.Start_Sx; %inizio pala 1 con profilo costante
%pala2
xac2=BoomInfo.Aero.P_origin_Dx(1);%posizione centro aerodinamico pala 2
start2=BoomInfo.Aero.Start_Dx; %inizio pala 2 con profilo costante
alpha_cl=BoomInfo.Aero.alpha_cl;
alpha_cd=BoomInfo.Aero.alpha_cd;
alpha_cm=BoomInfo.Aero.alpha_cm;
CL_t=BoomInfo.Aero.Cl;
CD_t=BoomInfo.Aero.Cd;
CM_t=BoomInfo.Aero.Cm;
%% definizione sistemi di riferimento pala
%PALA 1
span1=linspace(start1,start1+L,10);
eta1=midspan(span1);
lambda=pi/2+freccia;
%svergolamento
twist1=zeros(1,length(eta1));
% twist1=linspace(0,2*pi/180,length(eta1));

%spanwise wing distance

%calcolo n° Re
spanRe=3/4*(start1+L);

%matrice di rotazione da body a blade pala 1
Tj1=[sin(lambda)*cos(pitch)+cos(lambda)*sin(coning)*sin(pitch), -cos(lambda)*cos(pitch)+sin(lambda)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(lambda)*cos(coning), sin(lambda)*cos(coning), +sin(coning)
    sin(lambda)*sin(pitch)-cos(lambda)*sin(coning)*cos(pitch), -cos(lambda)*sin(pitch)-sin(lambda)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];


%SECONDA PALA
%spanwise wing distance
span2=linspace(start2,start2+L,10);
eta2=midspan(span2);
lambda=2*pi-lambda;
%svergolamento
twist2=zeros(1,length(eta2));
% twist2=linspace(0,2*pi/180,length(eta1));
%matrice di rotazione da body a blade
Tj2=[sin(lambda)*cos(pitch)+cos(lambda)*sin(coning)*sin(pitch), -cos(lambda)*cos(pitch)+sin(lambda)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(lambda)*cos(coning), sin(lambda)*cos(coning), +sin(coning)
    sin(lambda)*sin(pitch)-cos(lambda)*sin(coning)*cos(pitch), -cos(lambda)*sin(pitch)-sin(lambda)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];
%% calcoli aerodinamici
%vel indotta
v_ind_old=0; %ipotesi iniziale

%% new Lore
ra1=([xac1;0;0]+(Tj1'*[zeros(1,numel(eta1));eta1;zeros(1,numel(eta1))]))';
ra2=([xac2;0;0]+(Tj2'*[zeros(1,numel(eta2));eta2;zeros(1,numel(eta2))]))';
%velocity of blade element
omega_m=[omega(1).*ones(numel(eta1),1) omega(2).*ones(numel(eta1),1) omega(3).*ones(numel(eta1),1)];
vel1=u'+cross(omega_m,ra1);
vel2=u'+cross(omega_m,ra2);
%relative velocity of blade in blade frame
w1= Tj1*(-vel1-[0 0 v_ind_old])';
w2= Tj2*(-vel2-[0 0 v_ind_old])';
%AoA
AoA1=wrapToPi(atan2(w1(3,:),w1(1,:))+twist1);
AoA2=wrapToPi(atan2(w2(3,:),w2(1,:))+twist2);
%BET
net=length(eta1);
CL_naca=interp1(alpha_cl, CL_t, AoA1*180/pi);
CD_naca=interp1(alpha_cd, CD_t, AoA1*180/pi);
CM_naca=interp1(alpha_cm, CM_t, AoA1*180/pi);
F1_i=(span1(2:net+1)-span1(1:net))'.*0.5.*1.225.*c.*(vecnorm(w1([1 3],:)).^2)'.*([-CL_naca.*sin(AoA1)+CD_naca.*cos(AoA1); zeros(size(AoA1)); CL_naca.*cos(AoA1)+CD_naca.*sin(AoA1)])';
M1_i=(span1(2:net+1)-span1(1:net))'.*0.5.*1.225.*c.*(vecnorm(w1([1 3],:)).^2)'.*([(CL_naca.*cos(AoA1)+CD_naca.*sin(AoA1)).*eta1; c.*CM_naca; (CL_naca.*sin(AoA1)-CD_naca.*cos(AoA1)).*eta1])';
CL_naca=interp1(alpha_cl, CL_t, AoA2*180/pi);
CD_naca=interp1(alpha_cd, CD_t, AoA2*180/pi);
CM_naca=interp1(alpha_cm, CM_t, AoA2*180/pi);
F2_i=(span2(2:net+1)-span2(1:net))'.*0.5.*1.225.*c.*(vecnorm(w2([1 3],:)).^2)'.*([-CL_naca.*sin(AoA2)+CD_naca.*cos(AoA2); zeros(size(AoA2)); CL_naca.*cos(AoA2)+CD_naca.*sin(AoA2)])';
M2_i=(span2(2:net+1)-span2(1:net))'.*0.5.*1.225.*c.*(vecnorm(w2([1 3],:)).^2)'.*([(CL_naca.*cos(AoA2)+CD_naca.*sin(AoA2)).*eta2; c.*CM_naca; (CL_naca.*sin(AoA2)-CD_naca.*cos(AoA2)).*eta2])';

F1_i=((Tj1')*F1_i');
M1_i=(Tj1')*M1_i'+cross([ones(1,net).*xac1;zeros(1,net);zeros(1,net)],F1_i);
F2_i=((Tj2')*F2_i');
M2_i=(Tj2')*M2_i'+cross([ones(1,net).*xac2;zeros(1,net);zeros(1,net)],F2_i);

F1=sum(F1_i,2);
M1=sum(M1_i,2);
F2=sum(F2_i,2);
M2=sum(M2_i,2);
F=F1+F2;
M=M1+M2;

%% calcolo Re
ra1_Re=[xac1;0;0]+Tj1'*[0;spanRe;0];
vel1_Re=u+cross(omega,ra1_Re);
wel1_Re= Tj1*(-vel1_Re-[0;0;v_ind_old]);

Re=1.225*norm([wel1_Re(1), 0, wel1_Re(3)])*c/(1.81*10^-5);
%% Calcolo momento Mn
vn_versore=-[u(1) u(2) 0]./norm([u(1) u(2) 0]);
Mn=vn_versore*M;
