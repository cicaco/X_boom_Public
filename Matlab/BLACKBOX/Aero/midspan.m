function [y]=midspan(x)
%% MIDSPAN computes the coordinates for the points located between
% two consecutive sections
% 
% y = MIDSPAN(x) x is the spanwise coordinate of the sections
n=length(x);
y=zeros(1,n-1);
for j=1:n-1
    y(j)=0.5*(x(j)+x(j+1));
end
