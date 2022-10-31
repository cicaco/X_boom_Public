function [dy]=EquationOfMotionsQuaternion(t,y, BoomInfo)
% La funzione calcola le equazioni di moto  tendendo conto anche della
% velocità indotta
% INPUT:
% - t: Time
% - y: Stati (1-4 quaternioni, 5-7 V angolari in S.Body, 8-10 V CG in
%   S.Body, 11-13 x,y,z in S. Inerziale
% - BoomInfo: Struct dati del Boomerang
% - Tl_0: matrice di attitude iniziale
% OUTPUT:
% - dy: derivata degli stati
Tl_0=eye(3);
I=BoomInfo.Mecc.I_rho;
m=BoomInfo.Mecc.m;
g=9.81;
% Definizione inerzie
Iz=I(3,3);
Ix= I(1,1);
Iy=I(2,2);
Ixy=I(1,2);
Ixz=I(1,3);
Iyz=I(2,3);
% Stati time t(i-1)
q1=y(1);
q2=y(2);
q3=y(3);
q4=y(4);
p=y(5);
q=y(6);
r=y(7);
ux=y(8);
uy=y(9);
uz=y(10);
quat=[q1 q2 q3 q4];
%Ricavo la matrice di atittude
T0 = quatToAtt( quat );
% Forza peso nel sistema di riferimento body
FG=T0*Tl_0*(-m*g*[0;0;1]);
% Calcolo forze aerodinamiche 
[F,M]=AeroDynamics_FAST([ux;uy;uz],[p;q;r],BoomInfo);
        
dy(1:4)=1/2*[0 r -q p; -r 0 p q; q -p 0 r; -p -q -r 0]*[q1 q2 q3 q4]';

M_pqr=[Ix -Ixy -Ixz; -Ixy Iy -Iyz ; -Ixz -Iyz Iz ];
dy(5:7)=M_pqr\(-cross([p;q;r],M_pqr*[p;q;r])+M);

dy(8)=(-m*q*uz+m*r*uy+F(1)+FG(1))/m;
dy(9)=(-m*r*ux+m*p*uz+F(2)+FG(2))/m;
dy(10)=(-m*p*uy+m*q*ux+F(3)+FG(3))/m;

VEL=Tl_0'*T0'*[ux;uy;uz];
dy(11)=VEL(1);
dy(12)=VEL(2);
dy(13)=VEL(3);
dy=dy';

end