function [YOUT] = Eul_Quat(YOUT_quat,TOUT)
%% Eul_Quat è una funzione che trasforma i quaternioni in angoli di eulero
% INPUT
% - YOUT_quat: Vettore d'uscita della ode
% - TOUT: Tempo  rispetto a YOUT
% OUTPUT:
% - YOUT: Vettore d'uscita della ode con angoli di eulero
euler=[];
for i=1:numel(TOUT)
    euli = quatToEuler(YOUT_quat(i,1:4) );
    euler(i,:)=[euli(2) euli(1) euli(3)];
end
YOUT=[unwrap(euler) YOUT_quat(:,5:end)];
end

