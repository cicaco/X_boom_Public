function PlotTipDxSx(TOUT,YOUT,BoomInfo)
%% PlotTipDxSx è una funzione che permette di plottare il comportamento del
% boomerang mostrando la posizione della tip Dx e Sx e della posizione del
% baricentro
% INPUT:
% - YOUT: Vettore d'uscita della ode con angoli di eulero
% - TOUT: Tempo  rispetto a YOUT
% - BoomInfo: Struct con le informazioni geometriche del boomerang
% OUTPUT:
% - Grafico
%%
Tl_0=eye(3);
R=BoomInfo.Pianta.l; %0.30; %m
xac=BoomInfo.Aero.P_origin_Sx(1); %va cambiato tra prima e seconda pala (DA FARE)

%sdr PALA j
sigma=BoomInfo.Pianta.freccia +90*pi/180; %120*pi/180;
coning=BoomInfo.Pianta.diedro; %tipo diedro (rot asse x2)
pitch=BoomInfo.Pianta.pitch; %5*pi/180; %pitch della pala (rot asse y3)

%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

x_tipsx_bodyframe=[xac;0;0]+Tj'*[0;R;0];

%sdr PALA dx
sigma= BoomInfo.Pianta.freccia + 210*pi/180;
coning=BoomInfo.Pianta.diedro; %tipo diedro (rot asse x2)
pitch=BoomInfo.Pianta.pitch; %5*pi/180; %pitch della pala (rot asse y3)
%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];
xac=BoomInfo.Aero.P_origin_Dx(1);
x_tipdx_bodyframe=[xac;0;0]+Tj'*[0;R;0];

x_tipdx=[];
x_tipsx=[];
for i=1:length(TOUT)
    theta=(YOUT(i,1));
    phi=(YOUT(i,2));
    psi=(YOUT(i,3));
    T0=[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
        -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
        sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
    x_tipdx=[x_tipdx  [YOUT(i,10); YOUT(i,11); YOUT(i,12)]+Tl_0'*T0'*x_tipdx_bodyframe];
    x_tipsx=[ x_tipsx  [YOUT(i,10); YOUT(i,11); YOUT(i,12)]+Tl_0'*T0'*x_tipsx_bodyframe];
end
figure()
plot3(YOUT(:,10),YOUT(:,11),YOUT(:,12),'b','linewidth',1.2)
hold on
plot3(x_tipsx(1,:),x_tipsx(2,:),x_tipsx(3,:),'g','linewidth',1.2)
plot3(x_tipdx(1,:),x_tipdx(2,:),x_tipdx(3,:),'r','linewidth',1.2)
legend('CG','tipSX','tipDX','fontsize',10,'interpreter','latex')

% legend('CG','fontsize',10,'interpreter','latex')
xlabel('X','fontsize',11,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
ylabel('Y','fontsize',11,'interpreter','latex');
zlabel('Z','fontsize',11,'interpreter','latex');
title('Boomerang Traiectory in 3D space','fontsize',12,'interpreter','latex');

grid on;
axis equal
end

