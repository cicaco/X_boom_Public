function [PARA] = GA_Spot(x,BoomInfo,D,theta,Chi)
BoomInfo.Profile.Xp_sx=BoomInfo.Profile.Xp_sx.*BoomInfo.Profile.Chord;
BoomInfo.Profile.Xp_dx=BoomInfo.Profile.Xp_dx.*BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_sx=BoomInfo.Profile.Zp_sx.*BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_dx=BoomInfo.Profile.Zp_dx.*BoomInfo.Profile.Chord;    
BoomInfo.Pianta.freccia=x(1)*pi/180/10;
BoomInfo.Pianta.l=x(2)/1000;
BoomInfo.Profile.Chord=BoomInfo.Pianta.l/(x(3)/100);
%divido per la nuova corda
BoomInfo.Profile.Xp_sx=BoomInfo.Profile.Xp_sx./BoomInfo.Profile.Chord;
BoomInfo.Profile.Xp_dx=BoomInfo.Profile.Xp_dx./BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_sx=BoomInfo.Profile.Zp_sx./BoomInfo.Profile.Chord;
BoomInfo.Profile.Zp_dx=BoomInfo.Profile.Zp_dx./BoomInfo.Profile.Chord;
[BoomInfo] = Boom3DShape(BoomInfo);

try
PARA = - SpotArea(BoomInfo,D,theta,Chi);
catch
    PARA=0;
end

