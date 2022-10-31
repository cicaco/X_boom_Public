function [PAR,varargout] = GA_para_Chi(x,BoomInfo,Chi,D,theta,varargin)
% GA_para_Chi è la fitness function utilizzata dall GA per calcolare le
% condizioni iniziali variando solo r0 e phi
% INPUT:
% - x: vettore dei parametri di progetto
% - BoomInfo: Struct dei dati del boomerang
% OUTPUT
% - PARA: vettore contente Tempo e Distanza finale della traiettoria del
%   boomerang, se il boomerang viene lanciato "male" PARA(2) = 1000
% Se varargin è presente:
% - T_min: Tempo del lancio finale
% - R_max: Distanza massima del lancio finale
r0=x(1)*2*pi/100;
phi=x(2)*pi/180/100;
R=norm(BoomInfo.Aero.P_Finish_Dx);
Vs=r0*R*(1/Chi-1);
z0= 1.8; % initial altitude


[quat,ustart] = HandInitial(r0,theta,D,phi,Vs,BoomInfo);
tfin=40;


%[V_dx_b,V_sx_b]=InitialConditionPlot(Tl_0,T0,ustart,[0;0;r0],BoomInfo);


options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
%%
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo),[0 tfin],Y0,options); %

% La distanza ed il tempo vengono minimizzati
Dist=norm(YOUT_quat(end,11:13));

if max(vecnorm(YOUT_quat(:,11:13)'))/1.1<=Dist
    Dist=1000;
end
PAR=Dist;
if isnan(Dist)
    fprintf('Lol \n');
end
%PAR=Dist;
if not(isempty(varargin))
    PAR(1)=Dist;
    PAR(2)=TOUT(end);
    fprintf ('SIMULAZIONE senza velocità indotta:\n');
    fprintf('Rotational Rate %.2f, Speed Tip %.2f \n',r0/2/pi,Vs);
    [YOUT] = Eul_Quat(YOUT_quat,TOUT);
    [t_min,r_max]=FinalReport(YOUT,TOUT);
    varargout{1}=t_min;
    varargout{2}=r_max;
end

