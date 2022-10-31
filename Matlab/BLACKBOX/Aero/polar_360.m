clear all;
close all;
clc

% FLAT PLATE ANALOGOUS FOR CL, CD, CM -> for deep stall airfoil
% CAMBER EFFECTS ARE INCLUDED for CL and CM
% ALGORITHM IS BASE ON BJORN MONTGOMERIE MOST FAMOUS PAPER(THE ONE 
% IMPLEMENTED IN QBLADE)

% CAREFULL CD90 (CD AT 90 DEGREE MUST BE SPECIFIED) -> TOTALLY ARBITRARY
% EXTENSION IN MAINLY BASED ON EXPERIMENTAL RESULTS AND FLT PLATE ANALOGOUS

a       = linspace(-20,20, 41);
Re      = 50000; %boomerang should have this Re (almost)
Mach    = 0;
airfoil_name = 'fastcatch'
airfoil = 'fastcatch.dat'

[pol,foil] = xfoil(airfoil,a,Re,Mach,'panels n 330', 'oper iter 1000')

% find AoA stall
alpha_ck = 4 % I really hope stall occur after 4 degree
delta = 10;
while delta > 0;
    cl_check = interp1(pol.alpha, pol.CL, [alpha_ck, alpha_ck+1]);
    %when cl changes sign -> alpha_stall
    delta = cl_check(2) - cl_check(1);
    alpha_ck = alpha_ck +1;
end
alpha_stall = alpha_ck -1 ;

% compute negative stall
alpha_ck = -4 % I really hope stall occur after -4 degree
delta = -10;
while delta < 0;
    cl_check = interp1(pol.alpha, pol.CL, [alpha_ck, alpha_ck-1]);
    %when cl changes sign -> alpha_stall
    delta = cl_check(2) - cl_check(1);
    alpha_ck = alpha_ck -1;
end
alpha_mstall = alpha_ck +1 ;

% ok let's take 5 angles more after stall
a       = linspace(alpha_mstall-5, alpha_stall+5, (alpha_stall-alpha_mstall)+11);
[pol,foil] = xfoil(airfoil,a,Re,Mach,'panels n 330', 'oper iter 1000')
%% alghorithm positive side
%retrieve clalpha
a_vec = (min(pol.alpha)+7):(max(pol.alpha)-7);
% prendo un paio di punti prima dello stallo
cl_f = interp1(pol.alpha, pol.CL, a_vec);
%cl alpha
% cl_a = (cl_f(5)-cl_f(1))/(4); % alpha transformed in degree
% %local cl_a for small angle
% %increasing alpha until the curve becomes non-linear -> find alpha_M
% cl_aOLD = cl_a;
% ii = 5;
% err = 100;
% toll = 0.2;
% while err> toll
%     cl_aloc = cl_f(ii+1)-cl_f(ii);
%     err = abs((cl_aloc-cl_aOLD)/cl_aOLD);
%     cl_aOLD = cl_aloc;
%     ii = ii+1;
% end
% 
% alpha_M = ii;
% % identification of linear regime
% cl_l = cl_f(1:ii+1);
% a_l  = a_vec(1:ii+1);

% linear regression
p      = polyfit(a_vec, cl_f, 1);
cl0    = p(2);% cl0
alpha0 = p(2)/p(1);
%% CL flat plate analogue
alpha_long = linspace(-180, 180, 361);
CL_90  = 0.08; %tipical value
CD_90  = 2;
delta1 = 57.6*CL_90*sin(alpha_long*pi/180);
delta2 = alpha0*cos(alpha_long*pi/180);
beta   = alpha_long - delta1 - delta2;
A      = 1 + cl0/sin(pi/4)*sin(alpha_long*pi/180);
% CL curved plate basic
CL_cPB = A * CD_90 .* sin(beta*pi/180).*cos(beta*pi/180);


plot(alpha_long, CL_cPB);
grid on;

%% changing the value with the one computed by Xfoil
ii_max = find(alpha_long==min(pol.alpha));
ii_min = find(alpha_long==max(pol.alpha));

alpha_fin = [ alpha_long(1:ii_max-1), pol.alpha', alpha_long(ii_min + 1:end)];
CL_fin    = [ CL_cPB(1:ii_max-1), pol.CL', CL_cPB(ii_min+1:end)];

figure
plot(alpha_fin, CL_fin);
grid on;

%% CD 
CD_90  = 1.6;
CD_cPB =  CD_90 .* sin(alpha_long*pi/180).^2;
CD_fin    = [ CD_cPB(1:ii_max-1), pol.CD', CD_cPB(ii_min+1:end)];

figure
plot(alpha_fin, CD_fin);
grid on;
xlabel('alpha');
ylabel('CD')
%% CM 
%derivinv armline
offset = @(a) 0.5111 - 0.001337 * a;

slope  = @(a) 0.001653 + 0.00016 * a;

armLine = offset(alpha0) + slope(alpha0) * (alpha_long-90);
% define armNeg
xA = abs(alpha0);
yA = offset(xA) + slope(xA)*(xA-90)
xB = -180 - xA;
yB =  offset(xA) + slope(xA)*(90);
k  = (yB-yA)/(xB-xA);
armNeg = yA + k.*(alpha_long - xA)
% plot arm
figure;
plot(alpha_long, [armNeg(1:180), armLine(181:end)])
% pagina 27 --> ci sono un po' di errori in quel paper
% il braccio qui dovrebbe essere plottato fino ad alpha0,
% ma in quell'intorno c'è il risultarto di Xfoil quindi non ci serve
% cd has to be added !!
CM_out = (-CL_cPB.*cos(alpha_long*pi/180)-CD_cPB.*sin(alpha_long*pi/180)).*(armLine-0.25);
CM_neg = (-CL_cPB.*cos(alpha_long*pi/180)-CD_cPB.*sin(alpha_long*pi/180)).*(armNeg-0.25);
% add cm given from xfoil
CM_fin = [ CM_neg(1:ii_max-1), pol.Cm', CM_out(ii_min+1:end)];
figure;
plot(alpha_fin, CM_fin);
grid on;

%% write cl, cd e cm to file
coeff360.alpha = alpha_fin;
coeff360.CL    = CL_fin;
coeff360.CD    = CD_fin;
coeff360.CM    = CM_fin;

filename = strcat(airfoil_name,'_360.mat')
save(filename, "coeff360")