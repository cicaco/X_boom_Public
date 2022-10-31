function [Shape_d,Shape_v,Shape] = Shape2D(XStart,ZFinish,ZStart,Y_span,Plant3D)
%SHAPESCALES Summary of this function goes here
%   Detailed explanation goes here
Shape_d=[];
Shape_v=[];
Shape=[];
num_span=size(Y_span,2);
%Creo i profili di transizione
[Xp_2d_trans,Zp_2d_trans] = Profile2d_Trans(XStart,ZFinish,ZStart,num_span);

% Ho diviso il profilo in 6 zone, parto dalle tip, bisogna fare una
% transizione da un profilo nullo

for i =1:num_span
    %trovo il minimo rispetto alla Y di bezier
    Y=Y_span(i);
    Vect=Y.*ones(1,400);
    Diff=abs(Vect-Plant3D(2,:));
    [~,index]=min(Diff);
    Plant3D_copy=Plant3D(2,:);
    Plant3D_copy(index)=200;
    Diff=abs(Vect-Plant3D_copy);
    [~,index2]=min(Diff);
    Chord=abs(Plant3D(1,index)-Plant3D(1,index2));
    
    % Ho trovato i due punti
    if Plant3D(1,index)>Plant3D(1,index2)
        LE=Plant3D(1,index2);
    else
        LE=Plant3D(1,index);
    end
    Xp_2d=Xp_2d_trans(i,:);
    Zp_2d=Zp_2d_trans(i,:);  
    [n,m]=size(Zp_2d);
    New=[(Chord.*Xp_2d+LE)' (Plant3D(2,index2).*ones(n,m))' (Chord.*Zp_2d)'];
    Shape=[Shape; New];
    Shape_d=[Shape_d;New(1:m/2,:);];
    Shape_v=[Shape_v;New(m/2+1:end,:);];
end
end

