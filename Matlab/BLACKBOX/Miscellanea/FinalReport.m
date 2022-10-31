function [t_min,r_max]=FinalReport(YOUT_QUAT,TOUT,varargin)
%% FinalReport è una funzione che permette di analizzare la traiettoria
% INPUT:
% - YOUT_QUAT: Vettore d'uscita della ode
% - TOUT: Tempo  rispetto a YOUT
% - BoomInfo: Struct con le informazioni geometriche del boomerang
% OUTPUT:
% - t_min: Durata del lancio
% - r_max: Distanza massima del lancio

%%
nVarargs = length(varargin);
C_plot=0;
i=1;
warning ('off');
while i<=nVarargs
    switch varargin{i}
        case 'No_plot'
            C_plot=1;
    end
    i=i+1;
end
disp('-------------------------------------------------------------------')
% esclusione tocco terra
if YOUT_QUAT(end,12) <= 0.01 || sqrt(YOUT_QUAT(end,10)^2+YOUT_QUAT(end,11)^2)>=4 || YOUT_QUAT(end,12)>=3
    fprintf('Il boomerang non torna indietro \n');
    % il boomerang ritorna e sono in grado di riprenderlo
end
  t_min = TOUT(end);
    r_max = max(sqrt(YOUT_QUAT(:,10).^2+YOUT_QUAT(:,11).^2+YOUT_QUAT(:,12).^2));
if C_plot==0
    
    fprintf('Tempo di ritorno calcolato: %.5f s\n', t_min)
    fprintf('Massima distanza percorsa: %.5f m\n', r_max)
    disp('-------------------------------------------------------------------')
end
end

