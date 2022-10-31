function coeff360 = f_polar_360(lon, airfoil_name, a, Re,ap,am,k, CD_90,varargin) 
% coeff360 = F_POLAR_360(lon, airfoil_name, Re)
% WARNING -> PUT XFOIL IN THE SAME FOLDER OF THIS SCRIPT AND XFOIL.M
% INPUT
% lon            : string can be set to 'load' or 'NACA'
% airfoil_name   : is the airfoil filename, if lon == NACA, just enter a
%                  string with the serie, ex. '0012'
% a              : vector of AoA specified for xFoil computation
% ap             : tuning parameter 1 for interpolation
% am             : tuning parameter 2 for interpolation
% k              : tuning parameter 3 for interpolation
% Re             : Reynolds number for which you want to perform the 
%                  computation
% CD_90          : value of drag expected at 90 deg 
% OUTPUT
% coeff360       : struct filled with 360 degree polar info
% coeff360.CL    : CL vector
% coeff360.CD    : CD vector
% coeff360.CM    : CM vector
% coeff360.alpha : AoA vector [deg]
% The function first recall xFoil, then extrapolate low AoA info in deep
% stall region, using the flat plate analoguos
% FLAT PLATE ANALOGOUS FOR CL, CD, CM -> for deep stall airfoil
% CAMBER EFFECTS ARE INCLUDED for CL and CM
% ALGORITHM IS BASE ON BJORN MONTGOMERIE MOST FAMOUS PAPER(THE ONE 
% IMPLEMENTED IN QBLADE)
% CAREFULL CD90 (CD AT 90 DEGREE MUST BE SPECIFIED) -> TOTALLY ARBITRARY
% EXTENSION IN MAINLY BASED ON EXPERIMENTAL RESULTS AND FLT PLATE ANALOGOUS

% Option
nVarargs = length(varargin);
C_save=0;
i=1;
warning ('off');
while i<=nVarargs
    switch varargin{i}
        case 'Save_Coeff'
            Filename=varargin{i+1};
            i=i+1;
            C_save=1;
    end
    i=i+1;
end
n       = length(a);
Mach    = 0;

if lon == "NACA"
    airfoil = strcat("NACA", airfoil_name);
elseif lon == "load"
    airfoil = airfoil_name;
else
    error("Input lon not recognized, it can only be 'load' or 'NACA'")
    %airfoil_name = 'fastcatch'
    %airfoil = 'fastcatch.dat'
end

%split coefficients computation  to help convergence
%positive angles
aPOS       = a((n-1)/2+1:end);
%negative angles, split to starts from 0 incidence
aNEG       = flip(a(1:(n-1)/2));
[polPOS,~] = xfoil(airfoil,aPOS,Re,Mach,'panels n 330', 'oper iter 1000');
[polNEG,~] = xfoil(airfoil,aNEG,Re,Mach,'panels n 330', 'oper iter 1000');
%reordering and restoring properly cl, cd and alpha
pol.CL     = [flip(polNEG.CL); polPOS.CL];
pol.CD     = [flip(polNEG.CD); polPOS.CD];
pol.Cm     = [flip(polNEG.Cm); polPOS.Cm];
pol.alpha  = [flip(polNEG.alpha); polPOS.alpha];

%% alghorithm positive si
%index for which we can consider linear polar -> -4 4 gradi
ii_lin = find(pol.alpha>=-5 & pol.alpha<=5);

% linear regression
p      = polyfit(pol.alpha(ii_lin), pol.CL(ii_lin), 1);
cl0    = p(2);% cl0
alpha0 = p(2)/p(1);
cl_a   = p(1);
cl_lin = @(a) cl0 + cl_a*a;
%% CL flat plate analogue
alpha_long = linspace(-180, 180, 361);
CL_90  = 0.08; %tipical value
delta1 = 57.6*CL_90*sin(alpha_long*pi/180);
delta2 = alpha0*cos(alpha_long*pi/180);
beta   = alpha_long - delta1 - delta2;
A      = 1 + cl0/sin(pi/4)*sin(alpha_long*pi/180);
% CL curved plate basic
CL_cPB = A * CD_90 .* sin(beta*pi/180).*cos(beta*pi/180);
%% changing the value with the one computed by Xfoil
ii_max = max(find(alpha_long<=min(pol.alpha)));
ii_min = min(find(alpha_long>=max(pol.alpha)));

%from where starts smoothing
aM1    = alpha_long(ii_min)-ap;
aM2    = alpha_long(ii_max)+am;
%smooth function introduction
f  = @(a)1./(1+k*(a-aM1).^4)+1./(1+k*(a-aM2).^4);
% smoothing CL_cPB
CL_cPB = CL_cPB.*(1-f(alpha_long)) + t(pol.CL,alpha_long).*f(alpha_long);

alpha_fin = [ alpha_long(1:ii_max-1), pol.alpha', alpha_long(ii_min + 1:end)];
CL_fin    = [ CL_cPB(1:ii_max-1), pol.CL', CL_cPB(ii_min+1:end)];

%% CD 

CD_cPB =  CD_90 .* sin(alpha_long*pi/180).^2;
CD_fin    = [ CD_cPB(1:ii_max-1), pol.CD', CD_cPB(ii_min+1:end)];

%% CM 
%derivinv armline
offset = @(a) 0.5111 - 0.001337 * a;

slope  = @(a) 0.001653 + 0.00016 * a;

armLine = offset(alpha0) + slope(alpha0) * (alpha_long-90);
% define armNeg
xA = abs(alpha0);
yA = offset(xA) + slope(xA)*(xA-90);
xB = -180 - xA;
yB =  offset(xA) + slope(xA)*(90);
k  = (yB-yA)/(xB-xA);
armNeg = yA + k.*(alpha_long - xA);

% pagina 27 --> ci sono un po' di errori in quel paper
% il braccio qui dovrebbe essere plottato fino ad alpha0,
% ma in quell'intorno c'è il risultarto di Xfoil quindi non ci serve
% cd has to be added !!
CM_out = (-CL_cPB.*cos(alpha_long*pi/180)-CD_cPB.*sin(alpha_long*pi/180)).*(armLine-0.25);
CM_neg = (-CL_cPB.*cos(alpha_long*pi/180)-CD_cPB.*sin(alpha_long*pi/180)).*(armNeg-0.25);
% add cm given from xfoil
CM_fin = [ CM_neg(1:ii_max-1), pol.Cm', CM_out(ii_min+1:end)];

%% write cl, cd e cm to file
coeff360.alpha = alpha_fin;
coeff360.CL    = CL_fin;
coeff360.CD    = CD_fin;
coeff360.CM    = CM_fin;
if C_save==1
    save(Filename,'alpha_fin','CL_fin','CD_fin','CM_fin');
end