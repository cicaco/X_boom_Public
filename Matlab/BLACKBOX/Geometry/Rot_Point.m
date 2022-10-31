function [P_fin] = Rot_Point(u,T,P,o)
%% Rot_Point è una funzione che fornisce la matrice di rotazione rispetto ad un punto
% OUTPUT:
% - u: 1x3 di norma 1
% - T: Angolo in radianti
% - P: punti da ruotare 3XN
% - o: punto rispetto al quale ruotare 3X1
% OUTPUT:
% - P_fin: Punti ruotati
%%
ux=u(1);
uy=u(2);
uz=u(3);
c=cos(T);
s=sin(T);

R=[ux^2*(1-c)+c ux*uy*(1-c)-uz*s ux*uz*(1-c)+uy*s;...
    ux*uy*(1-c)+uz*s uy^2*(1-c)+c uy*uz*(1-c)-ux*s;...
    ux*uz*(1-c)-uy*s uy*uz*(1-c)+ux*s uz^2*(1-c)+c];

[~,m]=size(P);
Trasl=o.*ones(1,m);
P_trasl=P-Trasl;
P_ruot=R*P_trasl;
P_fin=P_ruot+Trasl;

end

