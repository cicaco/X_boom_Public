function [R] = Rot(u,T)
%% ROT è una funzione che fornisce la matrice di rotazione da un asse e un
%angolo assegnato come input
% INPUT:
% u: 1x3 di norma 1
% T: Angolo in radianti 
% OUTPUT:
% R: Matrice 3D
%% 
ux=u(1);
uy=u(2);
uz=u(3);
c=cos(T);
s=sin(T);
if numel(T)==1
    R=[ux^2*(1-c)+c ux*uy*(1-c)-uz*s ux*uz*(1-c)+uy*s;...
        ux*uy*(1-c)+uz*s uy^2*(1-c)+c uy*uz*(1-c)-ux*s;...
        ux*uz*(1-c)-uy*s uy*uz*(1-c)+ux*s uz^2*(1-c)+c];
else
    R_11=@(T) reshape(ux^2*(1-c(T))+c(T),[1 1 size(T,2)]).*ones(1,1,size(T,2));
    R_12=@(T) reshape(ux*uy*(1-c(T))-uz*s(T),[1 1 size(T,2)]).*ones(1,1,size(T,2));
    R_13=@(T) reshape(ux*uz*(1-c(T))+uy*s(T),[1 1 size(T,2)]).*ones(1,1,size(T,2));
    R_21=@(T) reshape(ux*uy*(1-c(T))+uz*s(T),[1 1 size(T,2)]).*ones(1,1,size(T,2));
    R_22=@(T) reshape(uy^2*(1-c(T))+c(T),[1 1 size(T,2)]).*ones(1,1,size(T,2));
    R_23=@(T) reshape(uy*uz*(1-c(T))-ux*s(T),[1 1 size(T,2)]).*ones(1,1,size(T,2));
    R_31=@(T) reshape(ux*uz*(1-c(T))-uy*s(T),[1 1 size(T,2)]).*ones(1,1,size(T,2));
    R_32=@(T) reshape(uy*uz*(1-c(T))+ux*s(T),[1 1 size(T,2)]).*ones(1,1,size(T,2));
    R_33=@(T) reshape(uz^2*(1-c(T))+c(T),[1 1 size(T,2)]).*ones(1,1,size(T,2));
    R=@(T) [R_11(T) R_12(T) R_13(T);R_21(T) R_22(T) R_23(T);R_31(T) R_32(T) R_33(T)];
    R=R(T);
end

end

