% -------------------------------------------------------------------------
% File : ParticleFilterLocalization.m
% Discription : Mobible robot localization sample code with
%               ParticleFilterLocalization (PF)
%               The Robot can get a range data from RFID that its position
%               known.
% Environment : Matlab
% Author : Atsushi Sakai
% Copyright (c): 2014 Atsushi Sakai
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------
function [] = ParticleFilterLocalization()
%
close all, clear all;
disp('Particle Filter (PF) sample program start!!')
time = 0;
endtime = 60; % [sec]
global dt;
dt = 0.1; % [sec]
nSteps = ceil((endtime - time)/dt);
result.time=[];
result.xTrue=[];
result.xd=[];
result.xEst=[];
result.z=[];
result.PEst=[];
result.u=[];
% State Vector [x y yaw]'
xEst=[0 0 0]';
% True State
xTrue=xEst;
% Dead Reckoning Result
xd=xTrue;
% Covariance Matrix for predict
Q=diag([0.1 0.1 toRadian(3)]).^2;
% Covariance Matrix for observation
R=diag([1]).^2;%[range [m]
% Simulation parameter
global Qsigma
Qsigma=diag([0.1 toRadian(5)]).^2;
global Rsigma
Rsigma=diag([0.1]).^2;
% [x, y]
RFID=[10 0;
      10 10;
      0  15
      -5 20];
  
MAX_RANGE=20;%
NP=10;%
NTh=NP/2.0;%
px=repmat(xEst,1,NP);%
pw=zeros(1,NP)+1/NP;%
 
tic;
%movcount=0;
% Main loop
for i=1 : nSteps
    time = time + dt;
    % Input
    u=doControl(time);
    % Observation
    [z,xTrue,xd,u]=Observation(xTrue, xd, u, RFID, MAX_RANGE);
    % ------ Particle Filter --------
    for ip=1:NP
        x=px(:,ip) ;
        w=pw(ip);
        % Dead Reckoning and random sampling
        x=f(x, u)+sqrt(Q)*randn(3,1);
        % Calc Inportance Weight
        for iz=1:length(z(:,1))
            pz=norm(x(1:2)'-z(iz,2:3));
            dz=pz-z(iz,1);
            w=w*Gauss(dz,0,sqrt(R));
        end
        px(:,ip)=x;%
        pw(ip)=w;
    end
    
    pw=Normalize(pw,NP);%
    [px,pw]=Resampling(px,pw,NTh,NP);%
    xEst=px*pw';%
    
    % Simulation Result
    result.time=[result.time; time];
    result.xTrue=[result.xTrue; xTrue'];
    result.xd=[result.xd; xd'];
    result.xEst=[result.xEst;xEst'];
    result.u=[result.u; u'];
    
    %Animation (remove some flames)
    if rem(i,5)==0 
        hold off;
        arrow=0.5;
        %
        for ip=1:NP
            quiver(px(1,ip),px(2,ip),arrow*cos(px(3,ip)),arrow*sin(px(3,ip)),'ok');hold on;
        end
        plot(result.xTrue(:,1),result.xTrue(:,2),'.b');hold on;
        plot(RFID(:,1),RFID(:,2),'pk','MarkerSize',10);hold on;
        %
        if~isempty(z)
            for iz=1:length(z(:,1))
                ray=[xTrue(1:2)';z(iz,2:3)];
                plot(ray(:,1),ray(:,2),'-r');hold on;
            end
        end
        plot(result.xd(:,1),result.xd(:,2),'.k');hold on;
        plot(result.xEst(:,1),result.xEst(:,2),'.r');hold on;
        axis equal;
        grid on;
        drawnow;
        %
        %movcount=movcount+1;
        %mov(movcount) = getframe(gcf);% 
    end
end
toc
%
%movie2avi(mov,'movie.avi');
DrawGraph(result);

% Low Variance Sampling
function [px,pw]=Resampling(px,pw,NTh,NP)
Neff=1.0/(pw*pw');
if Neff<NTh %
    wcum=cumsum(pw);
    base=cumsum(pw*0+1/NP)-1/NP;%
    resampleID=base+rand/NP;%
    ppx=px;%
    ind=1;%ID
    for ip=1:NP
        while(resampleID(ip)>wcum(ind))
            ind=ind+1;
        end
        px(:,ip)=ppx(:,ind);%
        pw(ip)=1/NP;%
    end
end

% 
function pw=Normalize(pw,NP)
sumw=sum(pw);
if sumw~=0
    pw=pw/sum(pw);%
else
    pw=zeros(1,NP)+1/NP;
end

%
function p=Gauss(x,u,sigma)
p=1/sqrt(2*pi*sigma^2)*exp(-(x-u)^2/(2*sigma^2));

% Motion Model
function x = f(x, u)
global dt;
F = [1 0 0
    0 1 0
    0 0 1];
B = [
    dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt];
x= F*x+B*u;

% Calc Input Parameter
function u = doControl(time)
T=10; % [sec]
% [V yawrate]
V=1.0; % [m/s]
yawrate = 5; % [deg/s]
u =[ V*(1-exp(-time/T)) toRadian(yawrate)*(1-exp(-time/T))]';

%Calc Observation from noise prameter
function [z, x, xd, u] = Observation(x, xd, u, RFID,MAX_RANGE)
global Qsigma;
global Rsigma;
x=f(x, u);% Ground Truth
u=u+sqrt(Qsigma)*randn(2,1);%add Process Noise
xd=f(xd, u);% Dead Reckoning
%Simulate Observation
z=[];
for iz=1:length(RFID(:,1))
    d=norm(RFID(iz,:)-x(1:2)');
    if d<MAX_RANGE %
        z=[z;[d+sqrt(Rsigma)*randn(1,1) RFID(iz,:)]];
    end
end

%Plot Result
function []=DrawGraph(result)
figure(1);
hold off;
x=[ result.xTrue(:,1:2) result.xEst(:,1:2)];
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(x(:,1), x(:,2),'-.b','linewidth', 4); hold on;
plot(x(:,3), x(:,4),'r','linewidth', 4); hold on;
plot(result.xd(:,1), result.xd(:,2),'--k','linewidth', 4); hold on;
 
title('PF Localization Result', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('Ground Truth','PF','Dead Reckoning');
grid on;
axis equal;

% degree to radian
function radian = toRadian(degree)
radian = degree/180*pi;

% radian to degree
function degree = toDegree(radian)
degree = radian/pi*180;