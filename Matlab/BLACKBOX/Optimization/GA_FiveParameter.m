function [PAR,varargout] = GA_FiveParameter(x,BoomInfo,varargin)
tic
r0=x(1)*2*pi/10;
theta=x(2)*pi/180/10;
D=x(3)*pi/180/10;
phi=x(4)*pi/180/10;
Vs=x(5)/10;
z0= 1.8; % initial altitude

[quat,ustart] = HandInitial(r0,theta,D,phi,Vs,BoomInfo);

tfin=40;


options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
%%
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo),[0 tfin],Y0,options); %
Dist=norm(YOUT_quat(end,11:13));

if max(vecnorm(YOUT_quat(:,11:13)'))/1.1<=Dist
    Dist=1000;
end
PAR=Dist;
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


