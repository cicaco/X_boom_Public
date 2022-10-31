function [Are] = SpotArea(BoomInfo,D,theta,Chi)
%% STABILITYCHECK è una funzione che permette di stimare su N lanci con
% condizioni iniziali randomiche in un certo range quante volte effettivamente il
% boomerang torna indietro
% INPUT
% - BoomInfo: Struct dati del Boomerang
% - lb. Limite inferiore delle condizoni iniziali
% - ub: Limite superiore delle condizoni iniziali
% - N: numero di lanci N
% OUTPUT:
% - S: Percentuale di lanci riusciti
% - Time: Tempo dei lanci  (DIM:Nx1)
% - Dist: Distanza finale dal lanciatore (DIM:Nx1)
% - Xm: Condizioni iniziali degli N lanci (DIM:Nx5)
%%

tfin=40;
z0= 1.8; % initial altitude

S=0;

error=0;
%% griglia iniziale
n=5;
m=5;
A=zeros(n,m);

jmax=0;
imax=0;
imin=100;
jmin=100;
AREA_ini=  [];
%Ai=[reshape((linspace(1,n,n)'.*ones(1,m))',[1,m*n]);reshape((linspace(1,m,m)'.*ones(1,n)),[1,m*n])];

for i=1:n
    for j=1:m
        phi=(87.5-5*(i-1))*pi/180;
        r0=(6+2*(j-1))*2*pi;

        Vs=r0*norm(BoomInfo.Aero.P_Finish_Dx)*(1/Chi-1);
        
        [quat,ustart] = HandInitial(r0,theta,D,phi,Vs,BoomInfo);
    
    options = odeset('Events', @EventsAntiSheronQUAT,'RelTol',1e-4,'AbsTol',1e-6);
    Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
    [~,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo),[0 tfin],Y0,options); %
    Dist_i=norm(YOUT_quat(end,11:13));
  
     if max(vecnorm(YOUT_quat(:,11:13)'))/1.1<=Dist_i 
        Dist_i=1000;
     end
    if Dist_i<5 && YOUT_quat(end,end)<2.0 && max(vecnorm(YOUT_quat(:,11:12)'))>10
        A(i,j)=1;
        AREA_ini=[AREA_ini; r0 phi];

        if j>jmax
            jmax=j;
        end
        if i>imax
            imax=i;
        end
        if j<jmin
            jmin=j;
        end
        if i<imin
            imin=i;
        end 
    end
    end
end



lb_phi=0;
ub_phi=0;
lb_r0=0;
ub_r0=0;

if imax==0
    S=0;
    error=1;
end

if imin==1
    ub_phi=90*pi/180;
else
    ub_phi=(87.5-5*(imin-2))*pi/180; 
end

if jmin==1
    lb_r0=5*2*pi;
else
    lb_r0=(6+2*(jmin-2))*2*pi;
end

if imax==5
    lb_phi=65*pi/180;
else
    lb_phi=(87.5-5*(imax))*pi/180;
end

if jmax==5
    ub_r0=15*2*pi;
else
    ub_r0=(6+2*(jmax))*2*pi;
end
N=nnz(A);
N=30*N;

lb=[lb_phi lb_r0];
ub=[ub_phi ub_r0];
%% prove random su griglia più ristretta
if error
    Are=0;
else
Xm=[lb(1)+(ub(1)-lb(1))*rand(N,1) lb(2)+(ub(2)-lb(2))*rand(N,1)];
hh=BoomInfo.Aero.P_Finish_Dx;
AREA=zeros(N,2);
parfor i=1:N 
    r0=Xm(i,2);
    phi=Xm(i,1);

    Vs=r0*norm(hh)*(1/Chi-1);
    [quat,ustart] = HandInitial(r0,theta,D,phi,Vs,BoomInfo);
    
    options = odeset('Events', @EventsAntiSheronQUAT,'RelTol',1e-4,'AbsTol',1e-6);
    Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
    [TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo),[0 tfin],Y0,options); %
    Time_i=TOUT(end);
    Dist_i=norm(YOUT_quat(end,11:13));
    
    if max(vecnorm(YOUT_quat(:,11:13)'))/1.1<=Dist_i %(che è sta roba)
        Dist_i=1000;
    elseif Dist_i<5 && YOUT_quat(end,end)<2.0
        S=S+1;
        AREA(i,:)=[ r0 phi];

    end
    Dist(i)=Dist_i;
    Time(i)=Time_i;
    
end
S=S/N*100;
%%
AREA=[AREA;AREA_ini];
r_ok=nonzeros(AREA(:,1))/2/pi;
phi_ok=nonzeros(AREA(:,2))*180/pi;
[~,Are]=boundary(r_ok,phi_ok,1);

end
end