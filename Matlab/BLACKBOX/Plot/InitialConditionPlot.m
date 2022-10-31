function [V_dx_b,V_sx_b]=InitialConditionPlot(T0,ustart,Om,BoomInfo)
%% InitialConditionPlot permette di visualizzare il boomerang nel sistema di
% riferimento inerziale, rispetto alle condizioni iniziali, inoltre viene
% fornita la distribuzioni di velocità a t=0.
% INPUT
% - Tl_0: Matrice di attitude iniziale
% - T0: Matrice di attitude
% - ustart: Velocità iniziale del baricentro nel sistema di rifeirmento
%   BODY
% - Om: Velocità angolari nel sistema di riferimento BODY
% - BoomInfo: Struct con le informazioni geometriche del boomerang
% OUTPUT:
% - V_dx_b: Velocità dei profili della pala destra
% - V_sx_b: Velcità dei profili della pala sinistra
%%

Tl_0=eye(3);
% Ricavo da BoomInfo i dati necessari
num=BoomInfo.Geom3D.num;
p_c=BoomInfo.Geom3D.p_c;
CG=BoomInfo.Mecc.CG;
l=BoomInfo.Pianta.l;
P_tot=BoomInfo.Geom3D.Profile;

figure()
% Riscrivo i profili nel sistema di riferimento inerziale
for i=1:2*(num+p_c)-2
    P_tot(3*i-2:3*i,:)=P_tot(3*i-2:3*i,:)-CG';
    P_ruot=Tl_0'*T0'*P_tot(3*i-2:3*i,:);
    plot3(P_ruot(1,:),P_ruot(2,:),P_ruot(3,:),'k');
    hold on
    axis equal
    grid on
end
C_fin_rot=BoomInfo.Geom3D.C_aer;
% Calcolo velocità in questi punti
P_dx=C_fin_rot(:,1:num)';
P_sx=C_fin_rot(:,num+2*p_c:end)';
%Calcolo la velocità dei centri aerodinamici della pala destra e sinistra
%nel sistema di riferimento body
for i=1:num
    V_dx(i,:)=ustart+cross(Om',P_dx(i,:))';
    V_sx(i,:)=ustart+cross(Om',P_sx(i,:))';
end
% Riscrivo la posizione e la velocità dei centri aerodinamici nel sistema
% di riferimento body
P_dx=((T0*Tl_0)'*P_dx')';
P_sx=((T0*Tl_0)'*P_sx')';
V_dx=((T0*Tl_0)'*[V_dx(:,1) V_dx(:,2) V_dx(:,3)]')';
V_sx=((T0*Tl_0)'*[V_sx(:,1) V_sx(:,2) V_sx(:,3)]')';
V_dx_b=V_dx;
V_sx_b=V_sx;
% Creo la figura
h2=quiver3([P_dx(:,1); P_sx(:,1)],[P_dx(:,2) ;P_sx(:,2)],[P_dx(:,3) ;P_sx(:,3)],[V_dx_b(:,1) ;V_sx_b(:,1)],[V_dx_b(:,2); V_sx_b(:,2)],[V_dx_b(:,3); V_sx_b(:,3)],'r','linewidth',1);
h1=quiver3(0.0,0,0,1,0 ,0,'b','linewidth',1);
quiver3(0.0,0,0,0,1 ,0,'b','linewidth',1);
quiver3(0.0,0,0,0,0 ,1,'b','linewidth',1);
plot3([0 -1],[0 0], [0 0],'--b','linewidth',1);
plot3([0 0],[0 -1], [0 0],'--b','linewidth',1);
plot3([0 0],[0 0], [0 -1],'--b','linewidth',1);
xlabel('X','fontsize',10,'interpreter','latex');
ylabel('Y','fontsize',10,'interpreter','latex');
zlabel('Z','fontsize',10,'interpreter','latex');
title('Configurazione Iniziale','fontsize',11,'interpreter','latex');
xlim([-2*l,2*l]);
%Riscrivo la velocità e la velocità angolare nel sistema di riferimento
%body
Us=Tl_0'*T0'*ustart;
Us=Us/norm(Us);
Oms=Tl_0'*T0'*Om;
Oms=Oms/norm(Oms);
h4=quiver3(0,0,0,Us(1),Us(2),Us(3),'g','linewidth',1);
h5=quiver3(0,0,0,Oms(1),Oms(2),Oms(3),'c');
legend([h1 h2 h4 h5],{'Asse-Terreno','$V_{Profili}$','$V_{CG}$','$V_{Ang}$'},'fontsize',8,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
