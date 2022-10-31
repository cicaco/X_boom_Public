function AoAProbabilityPlot(YOUT, BoomInfo)

%AOAPROBABILITYPLOT(Y_OUT) -> calcola la probabilità che una tasca di
%ciacuna pala del boomerang venga investita da un determinato angolo di
%attacco
%
%Y_OUT -> output dello stato dopo l'integrazione numerica della traiettoria
%         è una matrice: riga -> istante temporale/ colonna -> indice dello
%         stato

% ricavo le variabili necesarie
ux=YOUT(:,7);
uy=YOUT(:,8);
uz=YOUT(:,9);
p=YOUT(:,4);
q=YOUT(:,5);
r=YOUT(:,6);
theta=YOUT(:,1);
phi=YOUT(:,2);
psi=YOUT(:,3);
n_t = length(p);

AoAT = [];

for i = 1:n_t;
% recupero angolo di attacco da aerodynamicfast
    [PIPPO,PLUTO,AoA1,AoA2]=AeroDynamics_FAST([ux(i);uy(i);uz(i)],[p(i);q(i);r(i)],BoomInfo);
    % PIPPO, PLUTO -> non usate
    % considerando gli angoli di attacco delle due pale
    AoAT = [AoAT, AoA1, AoA2];
end
%plot histogram with more recurrent AoA
figure;
%plot raggruppando tra loro campioni di 5 gradi come deltaAoA
histogram(AoAT*180/pi, 360/5);
title('AoA distribution');
xlabel('AoA [deg]');