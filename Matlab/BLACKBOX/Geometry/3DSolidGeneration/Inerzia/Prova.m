clear all
close all
addpath(genpath('Auxiliary Functions'));
load('sample_mesh') 	
RBP=RigidBodyParams(TR);
disp(RBP)
VisualizeLocalFrame(TR)
P=TR.Points;
T=TR.ConnectivityList;
figure()
for i=1:18976
p=fill3(P(T(i,:),1),P(T(i,:),2),P(T(i,:),3),'r');
hold on
p(1).FaceAlpha = 0.5;

end
grid on 
axis equal