function [quat,ustart] = HandInitial(r0,theta,D,phi,Vs,BoomInfo)
%% Handinital funzione che permette di ricavare le condizioni iniziali
% rispetto a 5 parametri che sono spiegati con maggiore dettaglio sulla
% relazione
% INPUT
% - r0: velocità angolare in direzione z
% - theta: angolo di attitude
% - D:
% - phi
% - Vs
% - Tl_0: matrice di attitude iniziale
% - BoomInfo: struct con i dati del boomerang
% OUTPUT:
% - quat: Quaternioni iniziali
% - ustart: Velocità iniziale del baricentro nel sistema di riferimento Body
Tl_0=eye(3);
psi=pi-D;

T0=[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
V_tip=(T0*Tl_0*[Vs*cos(theta)*cos(D);-Vs*cos(theta)*sin(D);Vs*sin(theta)])'; %Velocità della tip nel piano del boomerang
r_mano=[0 0 r0];
P_tip=BoomInfo.Aero.P_Finish_Dx;
ustart=V_tip+cross(r_mano,-P_tip');
eul=[psi theta phi];
quat = eul2quat( eul );
quat=[quat(2) quat(3) quat(4) quat(1)];

end

