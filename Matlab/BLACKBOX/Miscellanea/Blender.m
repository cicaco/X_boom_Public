function  Blender(TOUT,YOUT)
%% Blender è una funzione che permette di esportare i dati necessari a
% blender per il calcolo della simulazione
% INPUT
% - YOUT: Vettore d'uscita della ode con angoli di eulero
% - TOUT: Tempo  rispetto a YOUT
% OUTPUT:
% - Viene salvato un file .mat "T.mat"

%%
Time=TOUT(:);
x=YOUT(:,10);
y=YOUT(:,11);
z=YOUT(:,12);
Phi=YOUT(:,2)*180/pi;
Psi=YOUT(:,3)*180/pi;
Theta=YOUT(:,1)*180/pi;
save('T.mat','Time','Theta' ,'Phi' ,'Psi' , 'x' ,'y' ,'z');
end

