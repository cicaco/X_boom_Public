function [PARA] = GA_Spot_constrained(x,lb,ub,BoomInfo,D,theta,Chi)
if x(1)>=lb(1) && x(1)<=ub(1) &&  x(2)>=lb(2) && x(2)<=ub(2) && x(3)>=lb(3) && x(3)<=ub(3) 
BoomInfo.Profile.Xp_sx=BoomInfo.Profile.Xp_sx./BoomInfo.Profile.Chord;
BoomInfo.Profile.Xp_dx=BoomInfo.Profile.Xp_dx./BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_sx=BoomInfo.Profile.Zp_sx./BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_dx=BoomInfo.Profile.Zp_dx./BoomInfo.Profile.Chord;    
BoomInfo.Pianta.freccia=x(1)*pi/180;
BoomInfo.Pianta.l=x(2);
BoomInfo.Profile.Chord=BoomInfo.Pianta.l/(x(3));
%divido per la nuova corda
BoomInfo.Profile.Xp_sx=BoomInfo.Profile.Xp_sx.*BoomInfo.Profile.Chord;
BoomInfo.Profile.Xp_dx=BoomInfo.Profile.Xp_dx.*BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_sx=BoomInfo.Profile.Zp_sx.*BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_dx=BoomInfo.Profile.Zp_dx.*BoomInfo.Profile.Chord;
[BoomInfo] = Boom3DShape(BoomInfo);

try
PARA = - SpotArea(BoomInfo,D,theta,Chi);
catch
    PARA=0;
end
else
    PARA=0;
end

