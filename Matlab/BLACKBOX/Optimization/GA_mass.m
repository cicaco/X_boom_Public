function [PARA] = GA_mass(x,lb,ub,BoomInfo,D,theta,Chi)
% Funzione fitness per l'ottimizzazione della massa, 
% INPUT:
% - x: Parametri di progetto ('Densità);
% - lb,ub: lower and upper boundary
% - BoomInfo: Struct sulla forma del boomerang
% - D,Theta,Chi: COndizioni di lancio
% OUTPUT:
% - PARA: Area della macchia (Negativa, problema di minimo)
if x>=lb && x<=ub 
BoomInfo.Mecc.I_rho=BoomInfo.Mecc.I_rho./BoomInfo.Mecc.Dens;
BoomInfo.Mecc.Dens=x;
BoomInfo.Mecc.m=BoomInfo.Mecc.V*x;
BoomInfo.Mecc.I_rho=BoomInfo.Mecc.I_rho.*x;
try
PARA = - SpotArea(BoomInfo,D,theta,Chi);
catch
    PARA=0;
end
else
    PARA=0;
end

