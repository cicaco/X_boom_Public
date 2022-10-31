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

isterminal = 1;   % Stop the integration
direction  = 0;

end