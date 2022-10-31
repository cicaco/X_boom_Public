function [BoomInfo] = Boom3DShape_VALIDAZIONE_BOOMERANG(BoomInfo,varargin)
%% Boom3DShapes creates the 3D shapes of the boomerang by traingulation
% method
% INPUT
% BoomInfo, struct with this information:
% - BoomInfo.Profile.Chord
% - BoomInfo.Profile.Chord;
% - BoomInfo.Geom3D.p_c: number of profile in the central part
% - BoomInfo.Pianta.l: lenght of half boomerang
% - BoomInfo.Pianta.freccia: Swept angle
% - BoomInfo.Pianta.diedro: Coning angle
% - BoomInfo.Pianta.pitch: Pitch Angle
% - BoomInfo.Geom3D.num: Number of profile on each blade
% - BoomInfo.Geom3D.PARA; Parameter that control the curve of the central
%   part. If Para=1, there is no curvature (V)
% - BoomInfo.Profile.Xp_dx; %Profile blade dx (Asse X)
% - BoomInfo.Profile.Zp_dx; %Profile blade dx (Asse Z)
% - BoomInfo.Profile.Xp_sx; %Profile blade sx (Asse X)
% - BoomInfo.Profile.Zp_sx; %Profile blade sx (Asse Z)
% OUTPUT
% New variable will bee addeded to the struct:
% - BoomInfo.Aero.P_origin_Dx: Punto di "origine" della pala dx (HP: no curvatura centrale)
% - BoomInfo.Aero.P_Start_Dx: Punto di inizio della pala dx (HP: no curvatura centrale)
% - BoomInfo.Aero.P_Finish_Dx:  Punto di fine della pala dx (HP: no curvatura centrale)
% - BoomInfo.Aero.Start_Dx: Distanza tra "Origine" e inizio per pala dx
% - BoomInfo.Aero.P_origin_Sx:
% - BoomInfo.Aero.P_Start_Sx:
% - BoomInfo.Aero.P_Finish_Sx:
% - BoomInfo.Aero.Start_Sx:
% - BoomInfo.Mecc.V: Volume Boomerang
% - BoomInfo.Mecc.I: Inerzia Boomerang
% - BoomInfo.Mecc.I_rho: inerzia con densità se specificata
% - BoomInfo.Mecc.m: Massa se densità specificata
% OPZIONI
% - 'Plot_figure': Vengono mostrate le figure
% - 'Create_Stl': Viene creato il file sitl
% - 'Density_variation': Opzione che permmette di creare una distribuzione
%   di densità sul boomerang (ad esempio creando una parte centrale del
%   boomerang più pesante), in questo caso è necessario dare in input un
%   secondo file avendo tali dimensione 1X(Num-3), ESEMPIO:
%   n=num+p_c
%   Dens_i=[1000.*ones(1,n-p_c-1) 1500*ones(1,2*p_c-1) 1000.*ones(1,n-p_c-1)];
% - 'Info': Viene dichirato cosa sta facendo il codice
% - 'FileName': Nome del FIle STL, altrimenti viene creato di default



%% Import Data from BoomInfo
Chord=BoomInfo.Profile.Chord;
p_c=BoomInfo.Geom3D.p_c;
l=BoomInfo.Pianta.l;
delta=BoomInfo.Pianta.freccia;
beta=BoomInfo.Pianta.diedro;
pitch=BoomInfo.Pianta.pitch;
num=BoomInfo.Geom3D.num;
PARA=BoomInfo.Geom3D.PARA;
Xp_dx=BoomInfo.Profile.Xp_dx;
Zp_dx=BoomInfo.Profile.Zp_dx;
Xp_sx=BoomInfo.Profile.Xp_sx;
Zp_sx=BoomInfo.Profile.Zp_sx;
ni=num;
num=num+p_c;
%% Set option of the function
C_fig=0;
C_stl=0;
C_VarDens=0;
C_info=0;
C_name=0;
nVarargs = length(varargin);
i=1;
cont=0;
warning ('off');
Err_1=0;
while i<=nVarargs
    switch varargin{i}
        case 'Plot_figure'
            C_fig=1;
        case 'Create_Stl'
            C_stl=1;
        case 'Density_variation'
            C_VarDens=1;
            Dens_i= varargin{i+1};
            i=i+1;
        case 'Info'
            C_info=1;
        case 'FileName'
            C_name=1;
            filename= [varargin{i+1},'.stl'];
            i=i+1;
        otherwise
            error('Verificare di aver inserito le opzioni corrette ')
    end
    i=i+1;
end
%
C_VarDens=1; %Constant Density
R=BoomInfo.Mecc.Dens;
Dens_i=[R.*ones(1,num-p_c-1) R*ones(1,2*p_c-1) R.*ones(1,num-p_c-1)];

%% Dati per il calettamento lungo la pala e la rotazione
% The boomerang is divided as it follows:
% Blade dx(y negative) 1:num-p_c
% Central part num-pc:num+p_c
% Blade sx(ypositive) num+p_c:end
% Creation of the "Transition" profile of the central section
[X_trans,Z_trans] = Profile2d_Trans(Xp_dx,Xp_sx,Zp_sx,Zp_dx,p_c*2-1);
%Considero una matrice di dimensione 2*num-1*132 che rappresenta il profilo
%di ciascuna delle sezione del boomerang
X_2d_i=[ones(num-p_c,1)*Xp_dx';X_trans;ones(num-p_c,1)*Xp_sx'];
Z_2d_i=[ones(num-p_c,1)*Zp_dx';Z_trans;ones(num-p_c,1)*Zp_sx'];

%Posizione del centro aerodinamico delle 2*num-1 sezioni (Pala dx: 1/4 Pala
%Sx 3/4
Fract=[1/4*ones(1,num-p_c) linspace(1/4,3/4,2*p_c-1) 3/4*ones(1,num-p_c)];

%Angoli di Freccia,Diedro, Pitch di ogni sezione
D_i=[-delta.*ones(1,num-p_c-1) linspace(-delta,delta,2*p_c+1) delta.*ones(1,num-p_c-1)];
B_i=[-beta.*ones(1,num-p_c-1) linspace(-beta,beta,2*p_c+1) beta.*ones(1,num-p_c-1)];
P_i=[-linspace(17*pi/180,0,num-p_c-1) linspace(-pitch,pitch,2*p_c+1) linspace(0,17*pi/180,num-p_c-1)];

%Angoli di Freccia e Diedro per trovare correttamente la posizione finale
%del centro aerodinamico
D_i_aer=[-delta.*ones(1,num-1) 0 delta.*ones(1,num-1)];
B_i_aer=[-beta.*ones(1,num) beta.*ones(1,num-1)];

%% Trovo la posizione finale del centro aerodinamico di ogni sezione
if C_info==1
    disp('-------------------------------------------------------------------')
    fprintf('Calcolo dei centri aerodinamici \n');
end
C=linspace(0,-Chord,num);
P=[C;zeros(1,num);zeros(1,num)];
U_Delta=[0 0 1];
li=linspace(l,0,num-p_c);
% Va aggiunta anche una translazione del primo profilo pari a q
Tx=[ -sin(delta).*li zeros(1,p_c*2-1) -sin(delta).*fliplr(li)];
Ty=[ -cos(delta).*li zeros(1,p_c*2-1) cos(delta).*fliplr(li)];
Tz=zeros(1,numel(Tx));
P_r=[];
P_center_r=[];


for i=1:2*num-1
    
    [P_1] = Rot_Point([0 0 1],D_i(i),P,[C(end)*PARA;0;0]); %Rotazione rispetto all'asse z, rispetto però al "bordo di uscita" del profilo
    P_1(1,:)=P_1(1,:)+Tx(i);
    P_1(2,:)=P_1(2,:)+Ty(i);
    [C_aer] = AerCenter(P_1,Chord,Fract(i));
    
    U_beta= Rot_Point(U_Delta,D_i_aer(i),[1; 0 ;0],[0 0 0]'); %devo ottenere il versore ruotato da x a x2 (body)
    [C_aer_rot] = Rot_Point(U_beta,B_i_aer(i),C_aer,[0;0;0]);
    
    C_fin(:,i)=C_aer;
    C_fin_rot(:,i)=C_aer_rot;
end

%% Creazione della nuova di punti che compongono il boomerang
%Ruoto ciascun profilo e lo traslo nel centro aerodinamico calcolato in
%precedenza
% Capire se si puù fare una parte centrale un po migliore
P_tot=[];
if C_info==1
    fprintf('Calcolo della geometria del Boomerang per punti\n');
end
for i=1:num*2-1
    P_2d=[X_2d_i(i,:);zeros(size(Z_2d_i(i,:)));Z_2d_i(i,:)];
    [P_fin] = Rot_Point([0 0 1],D_i(i),P_2d,[Fract(i)*C(end);0;0]); %Rotazione rispetto all'asse z, rispetto però al "bordo di uscita" del profilo
    U_beta= Rot_Point(U_Delta,D_i(i),[1; 0 ;0],[Fract(i)*C(end);0;0]); %devo ottenere il versore ruotato da x a x2 (body)
    [P_fin] = Rot_Point(U_beta,B_i(i),P_fin,[Fract(i)*C(end);0;0]);
    U_pitch_d= Rot_Point(U_Delta,D_i(i),[0; 1; 0 ],[Fract(i)*C(end);0;0]); % devo ottenre il versore ruotato da [0 1 0], ruotato di delta e poi di beta
    U_pitch= Rot_Point(U_beta,B_i(i),U_pitch_d,[Fract(i)*C(end);0;0]);
    [P_fin] = Rot_Point(U_pitch ,P_i(i),P_fin,[Fract(i)*C(end);0;0]);
    
    
    P_fin(1,:)=P_fin(1,:)-Fract(i)*C(end)+C_fin_rot(1,i);
    P_fin(2,:)=P_fin(2,:)+C_fin_rot(2,i);
    P_fin(3,:)=P_fin(3,:)+C_fin_rot(3,i);
    C_fin_rot(1,i)=C_fin_rot(1,i);
    if i~=num
        P_tot=[P_tot;P_fin];
    end
end

%% Carattersitiche inerziali
% Devo riportare il cad rispetto al baricentro (Solo posizione)
if C_fig==1
    figure()
    for i=1:2*num-2
        %plot3(P_tot(3*i-2,:),P_tot(3*i-1,:),P_tot(3*i,:),'*r');
        subplot(1,2,1)
        plot(P_tot(3*i-2,:),P_tot(3*i-1,:),'k');
        hold on
        axis equal
        grid on
        subplot(1,2,2)
        plot(P_tot(3*i-1,:),P_tot(3*i,:),'k');
        hold on
        axis equal
        grid on
    end
    %plot3(C_fin_rot(1,:),C_fin_rot(2,:),C_fin_rot(3,:),'oc');
    subplot(1,2,1)
    plot(C_fin_rot(1,:),C_fin_rot(2,:),'oc');
    subplot(1,2,2)
    plot(C_fin_rot(2,:),C_fin_rot(3,:),'oc');
    subplot(1,2,1)
    
    xlabel('X','fontsize',11,'interpreter','latex');
    set(gca,'TickLabelInterpreter','latex')
    ylabel('Y','fontsize',11,'interpreter','latex');
    subplot(1,2,2)
    
    xlabel('Y','fontsize',11,'interpreter','latex');
    set(gca,'TickLabelInterpreter','latex')
    ylabel('Z','fontsize',11,'interpreter','latex');
    sgtitle('Posizione finale dei centri aerodinamici','fontsize',12,'interpreter','latex');
end

% Start assembly each solid
n_prec=0;
tr=[];
xyz=[];
%warning('off','all')
% calcolo il centro di massa per ogni tasca
m_tot = 0;


Cg_tot=zeros(1,3);
Ixx = 0;
Iyy = 0;
Izz = 0;
Ixy = 0;
Iyz = 0;
Ixz = 0;
%Dens_i=[1000.*ones(1,num-p_c-1) 1500*ones(1,2*p_c-1) 1000.*ones(1,num-p_c-1)];
if C_info==1
    fprintf('Calcolo caratteristiche inerziali del Boomerang \n');
end
for i=2:2*num-2
    P_prec=P_tot(3*(i-1)-2:3*(i-1),:);
    P_succ=P_tot(3*i-2:3*i,:);
    if C_fig==1
        
        figure(20)
        plot3(P_prec(1,:),P_prec(2,:),P_prec(3,:),'k');
        hold on
        plot3(P_succ(1,:),P_succ(2,:),P_succ(3,:),'k');
    end
    if norm(P_prec-P_succ)~=0
        P_i=[P_prec';P_succ'];
        Mult=100;
        Err=0;
        % Il parametro Mult serve per evitare alcun Bug della funzione
        % matlab alphaShape e boundary facets
        while Err==0
            shp = alphaShape(P_i*Mult,1);
            [tr_i, xyz_i] = boundaryFacets(shp);
            xyz_i=xyz_i./Mult;
            try
                
                RBP=RigidBodyParams(triangulation(tr_i,xyz_i));
                Err=1;
            catch
                Mult=Mult-10;
            end
        end
        n_succ=length(xyz_i)+n_prec;
        tr=[tr;tr_i+n_prec];
        xyz=[xyz;xyz_i];
        n_prec=n_succ;
        
        if C_fig==1
            
            figure(30)
            if i<=ni
                plot(shp,'FaceColor',[0.153 0.255 0.102],'EdgeColor','k','LineStyle','none','LineWidth',0.1)
                hold on
            elseif i>=ni+2*p_c
                plot(shp,'FaceColor',[0.106 0.186 0.252],'EdgeColor','k','LineStyle','none','LineWidth',0.01)
                hold on
            else
                plot(shp,'FaceColor',[0.251 0.162 0.107],'EdgeColor','k','LineStyle','none','LineWidth',0.01)
                hold on
            end
            
        end
        if C_VarDens==1
            %Calcolo baricentro
            try
                RBP=RigidBodyParams(triangulation(tr_i,xyz_i));
                
            catch
                
                plot(shp)
                fprintf('Problem  at section number: %d \n',i);
                fprintf('Try to Modify p_c,num, o PARA .....');
            end
            
            CG_i=RBP.centroid;
            d=Dens_i(i-1); %densità unitaria
            m_i=d*RBP.volume;
            I_i=d*RBP.inertia_tensor;
            CG_tasche(:,i)=CG_i;
            
            XCg_tot = (m_tot * Cg_tot(1) + m_i * CG_i(1)) / (m_tot + m_i);
            YCg_tot = (m_tot * Cg_tot(2) + m_i * CG_i(2)) / (m_tot + m_i);
            ZCg_tot = (m_tot * Cg_tot(3) + m_i * CG_i(3)) / (m_tot + m_i);
            dX = XCg_tot - Cg_tot(1);
            dY = YCg_tot - Cg_tot(2);
            dZ = ZCg_tot - Cg_tot(3);
            dX_i = XCg_tot - CG_i(1);
            dY_i = YCg_tot - CG_i(2);
            dZ_i = ZCg_tot - CG_i(3);
            
            Ixx = Ixx +  I_i(1,1) + m_tot * (dY * dY + dZ * dZ) + m_i * (dY_i * dY_i + dZ_i * dZ_i);
            Iyy = Iyy +  I_i(2,2) + m_tot * (dX * dX + dZ * dZ) + m_i * (dX_i * dX_i + dZ_i * dZ_i);
            Izz = Izz +  I_i(3,3) + m_tot * (dX * dX + dY * dY) + m_i * (dY_i * dY_i + dX_i * dX_i);
            Ixy = Ixy +  I_i(1,2) + m_tot * dX * dY + m_i * dX_i * dY_i;
            Iyz = Iyz +  I_i(2,3) + m_tot * dZ * dY + m_i * dZ_i * dY_i;
            Ixz = Ixz +  I_i(1,3) + m_tot * dZ * dX + m_i * dZ_i * dX_i;
            m_tot = m_i + m_tot;
            
            Cg_tot(1) = XCg_tot;
            Cg_tot(2) = YCg_tot;
            Cg_tot(3) = ZCg_tot;
        end
    end
end
if Err_1==1
    fprintf('POSSIBILE errore nella costruzione geometrica.\nSe necessario prova a modificare num/p_c a secondo della posizione\n');
end

if C_fig==1
    figure (20)
    axis equal
    xlabel('X','fontsize',11,'interpreter','latex');
    set(gca,'TickLabelInterpreter','latex')
    ylabel('Y','fontsize',11,'interpreter','latex');
    zlabel('Z','fontsize',11,'interpreter','latex');
    title('Boomerang Profile 3D space','fontsize',12,'interpreter','latex');
    grid on
    figure (30)
    axis equal
    grid on
    axis equal
    xlabel('X','fontsize',11,'interpreter','latex');
    set(gca,'TickLabelInterpreter','latex')
    ylabel('Y','fontsize',11,'interpreter','latex');
    zlabel('Z','fontsize',11,'interpreter','latex');
    title('Boomerang Triangulation','fontsize',12,'interpreter','latex');
    %warning('on','all')
end
pr_fin=triangulation(tr,xyz);


RBP=RigidBodyParams(pr_fin);

if C_VarDens==0
    %densità costante
    CG=RBP.centroid;
else
    CG=Cg_tot;
end
xyz_CG=[xyz(:,1)-CG(1) xyz(:,2)-CG(2) xyz(:,3)-CG(3)];
pr_CG=triangulation(tr,xyz_CG);
RBP_CG=RigidBodyParams(pr_CG);

I=RBP_CG.inertia_tensor;
V=RBP_CG.volume;


%Stima baricentro cannata totalmente (Forse le tasche danno fastidio)?
% Magari si può calcolare dentro direttamente
%% Creazione file Stl
if C_stl==1
    
    if C_name==0
        filename=['Boom_D'+string(delta*180/pi)+'_B'+string(beta*180/pi)+'_P'+string(pitch*180/pi)+'.stl'];
    end
    stlwrite(pr_CG, filename);
    if C_info==1
        fprintf('Creazione del file STL: ');
        fprintf(filename);
        fprintf('\n');
        disp('-------------------------------------------------------------------')
        
    end
    
end
%% Aerodinamica
%per le caratterisitche aerodinamiche in C_fin_rot sono presenti le
%posizioni dei centri aerodinamici (non sono nel riferimento body)


Norm=(C_fin_rot(:,num+p_c)-C_fin_rot(:,num+p_c+1))/norm(C_fin_rot(:,num+p_c)-C_fin_rot(:,num+p_c+1));
[I_sx,~] = line_plane_intersection(Norm,C_fin_rot(:,num+p_c),[0 1 0]',[0 0 0]');


Norm=(C_fin_rot(:,num-p_c-1)-C_fin_rot(:,num-p_c))/norm(C_fin_rot(:,num-p_c-1)-C_fin_rot(:,num-p_c));
[I_dx,~] = line_plane_intersection(Norm,C_fin_rot(:,num-p_c),[0 1 0]',[0 0 0]');

%Boomerang perfettamente bilanciato
% CG(2)=0;
% CG(3)=0;
I_origin_Sx=I_sx-CG';
I_Start_Sx=C_fin_rot(:,num+p_c)-CG';
I_Finish_Sx=C_fin_rot(:,end)-CG';

I_origin_Dx=I_dx-CG';
I_Start_Dx=C_fin_rot(:,num-p_c)-CG';
I_Finish_Dx=C_fin_rot(:,1)-CG';

Start_Dx=norm(I_Start_Dx-I_origin_Dx);
l_check_Dx=norm(-I_Start_Dx+I_Finish_Dx);
Start_Sx=norm(I_Start_Sx-I_origin_Sx);
l_check_Sx=norm(-I_Start_Sx+I_Finish_Sx);

BoomInfo.Aero.P_origin_Dx=I_origin_Dx;
BoomInfo.Aero.P_Start_Dx=I_Start_Dx;
BoomInfo.Aero.P_Finish_Dx=I_Finish_Dx;
BoomInfo.Aero.Start_Dx=Start_Dx;
BoomInfo.Aero.P_origin_Sx=I_origin_Sx;
BoomInfo.Aero.P_Start_Sx=I_Start_Sx;
BoomInfo.Aero.P_Finish_Sx=I_Finish_Sx;
BoomInfo.Aero.Start_Sx=Start_Sx;
BoomInfo.Mecc.V=V;
if C_fig==1
    
    figure()
    h1=plot3(I_origin_Dx(1),I_origin_Dx(2),I_origin_Dx(3),'*r');
    hold on
    h2=plot3(I_Start_Dx(1),I_Start_Dx(2),I_Start_Dx(3),'ob');
    plot3(I_Finish_Dx(1),I_Finish_Dx(2),I_Finish_Dx(3),'ob');
    plot3([I_Finish_Dx(1) I_Start_Dx(1)],[I_Finish_Dx(2) I_Start_Dx(2)],[I_Finish_Dx(3) I_Start_Dx(3)],'k','linewidth',1.5);
    
    h3=plot3(I_origin_Sx(1),I_origin_Sx(2),I_origin_Sx(3),'*r');
    hold on
    h4=plot3(I_Start_Sx(1),I_Start_Sx(2),I_Start_Sx(3),'oc');
    plot3(I_Finish_Sx(1),I_Finish_Sx(2),I_Finish_Sx(3),'oc');
    plot3([I_Finish_Sx(1) I_Start_Sx(1)],[I_Finish_Sx(2) I_Start_Sx(2)],[I_Finish_Sx(3) I_Start_Sx(3)],'k','linewidth',1.5);
    set(gca,'TickLabelInterpreter','latex')
    
    legend([h1,h2,h3,h4],'Origine Pala Dx','Inizio e fine Pala Dx','Origine Pala Sx','Inizio e fine Pala Sx','fontsize',9,'interpreter','latex');
    grid on
end
if Ixy<0
    BoomInfo.Mecc.I_rho=[Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
    BoomInfo.Mecc.I=I.*R;
else
    BoomInfo.Mecc.I=[Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
    BoomInfo.Mecc.I_rho=I.*R;
end
% Inerzia=BoomInfo.Mecc.I_rho;
BoomInfo.Mecc.m=m_tot;
BoomInfo.Mecc.CG=CG;
BoomInfo.Geom3D.Fr_i=D_i;
BoomInfo.Geom3D.Di_i=B_i;
BoomInfo.Geom3D.Pi_i=P_i;
BoomInfo.Geom3D.Profile=P_tot;
BoomInfo.Geom3D.C_aer=C_fin_rot-CG';

end

