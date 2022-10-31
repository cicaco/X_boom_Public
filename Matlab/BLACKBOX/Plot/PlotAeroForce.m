function PlotAeroForce(YOUT,TOUT,BoomInfo,varargin)
%% PlotAeroForce è una funzione che permette di ricavare le forze
% aerodinamiche durante il lancio.
% INPUT
% - YOUT: Vettore d'uscita della ode con angoli di eulero
% - TOUT: Tempo  rispetto a YOUT
% - BoomInfo: Struct con le informazioni geometriche del boomerang
% OUTPUT:
% - Grafico
% OPZIONI:
% - 'Global_Frame' forze nel sistema di riferimento globale
% - 'Velocità_indotta' forze aerodinamiche con la velocità indotta
%%
nVarargs = length(varargin);
% Analisi Opzioni

i=1;
C_body=0;
C_ind=0;
TIT='Sistema di riferimento Body';
while i<=nVarargs
    switch varargin{i}
        case 'Global_Frame'
            C_body=1;
            TIT='Sistema di riferimento globale';
        case 'Velocità_indotta'
            C_ind=1;
        otherwise
            fprintf('Opzioni sbagliate');
    end
    i=i+1;
end
% Ricavo le variabile necessarie
ux=YOUT(:,7);
uy=YOUT(:,8);
uz=YOUT(:,9);
p=YOUT(:,4);
q=YOUT(:,5);
r=YOUT(:,6);
theta=YOUT(:,1);
phi=YOUT(:,2);
psi=YOUT(:,3);
num=numel(p);
F=zeros(3,num);
M=zeros(3,num);

%Calcolo delle forze aerodinamiche
for i=1:num
    if C_ind==0
        [F(:,i),M(:,i)]=AeroDynamics_FAST([ux(i);uy(i);uz(i)],[p(i);q(i);r(i)],BoomInfo);
    else
        [F(:,i),M(:,i)]=AeroDynamics_FAST_IND([ux(i);uy(i);uz(i)],[p(i);q(i);r(i)],BoomInfo);
        
    end
    if C_body==1
        
        T0=[cos(theta(i))*cos(psi(i)), cos(theta(i))*sin(psi(i)), -sin(theta(i))
            -cos(phi(i))*sin(psi(i))+sin(phi(i))*sin(theta(i))*cos(psi(i)), cos(phi(i))*cos(psi(i))+sin(phi(i))*sin(theta(i))*sin(psi(i)), sin(phi(i))*cos(theta(i))
            sin(phi(i))*sin(psi(i))+cos(phi(i))*sin(theta(i))*cos(psi(i)), -sin(phi(i))*cos(psi(i))+cos(phi(i))*sin(theta(i))*sin(psi(i)), cos(phi(i))*cos(theta(i))];
        
        F(:,i)=T0'*F(:,i);
        M(:,i)=T0'*M(:,i);
    end
end

% Figura
figure()
subplot(3,2,1)
plot(TOUT(:),F(1,:)','r','linewidth',1);
legend('Fx');
grid on

subplot(3,2,2)
plot(TOUT(:),M(1,:)','r','linewidth',1);
legend('Mx');
grid on

subplot(3,2,3)
plot(TOUT(:),F(2,:)','b','linewidth',1);
legend('Fy');
grid on

subplot(3,2,4)
plot(TOUT(:),M(2,:)','b','linewidth',1);
legend('My');
grid on

subplot(3,2,5)
plot(TOUT(:),F(3,:)','g','linewidth',1);
legend('Fz');
grid on

subplot(3,2,6)
plot(TOUT(:),M(3,:)','g','linewidth',1);
legend('Mz');
sgtitle(TIT);
grid on
end

