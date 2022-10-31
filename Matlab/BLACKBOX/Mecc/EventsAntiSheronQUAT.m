
function [value, isterminal, direction] = EventsQUAT(T, Y)
% EVENTS returns check and stop criteria for boomerang flight dynamics
% simulation (equation of motions)

% [value, isterminal, direction] = EVENTS(T, Y)
%
% given:
% - T -> instant of time
% - Y -> state vector [theta; phi; psi; p; q; r; ux; uy; uz; x; y; z]...
% ... everything in BODY framework apart from x, y, z  in inertial one
%
% returns:
% - value -> Boolean 
%       - z <= 0.01                              -> ground contact
%       - distance from starting position <= 3 m -> boomerang comes back
%       - phi >= 90 degree                       -> inclination check
% - isterminal ??
% - direction  ??
value      = (Y(13) <= 0.01); % tocco con il terreno

if T>2 && value==0
    if sqrt(Y(11)^2+Y(12)^2)<=3 && Y(13)<=2.5
    value = 1; % distanza dal punto di lancio
    end
    %a cui interrompo l'integrazione: 3 metri
end


quat= [Y(1) Y(2) Y(3) Y(4)];
euler = quatToEuler( quat );

phi = euler(1);
theta = euler(2);
psi= euler(3);
Rotazione= [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);
        cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi) cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi) sin(phi)*cos(theta);
        sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi) -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi)  cos(phi)*cos(theta)];
    
vel_inerziale=Rotazione*[Y(8) Y(9) Y(10)]';

if T>2 && value==0 && vel_inerziale(1)>0
    if atan2(vel_inerziale(2),vel_inerziale(1))<pi/2
    value= (atan2(vel_inerziale(2),vel_inerziale(1))>pi/3);
%     if value
%         vel_inerziale
%         Rotazione*eye(3)
%     end
    end
end



% 
% if T<1 && value==0
%     value=( Y(2)>= 90); % controllo sull'inclinazione del boomerang 
% end

isterminal = 1;   % Stop the integration
direction  = 0;
end
