% -------------------------------------------------------------------------
% File : GridMapSample.m
% Discription : Sample code to build grid map
% Environment : Matlab
% Author : Atsushi Sakai
% Copyright (c): 2014 Atsushi Sakai
% License : GPL Software License Agreement
% -------------------------------------------------------------------------
function GridMapSample()
close all, clear all;
%
%Grid Map
gm.CENTER=[0 0];%[x y]
gm.RESO=1;
gm.WIDTH=100;
gm.HEIGHT=100;
gm.nGrid=gm.WIDTH*gm.HEIGHT;
%
gm.data=zeros(1,gm.nGrid)+0.5;
pose=[0 0 0];%[x,y,yaw]
z=GetObservation();%
% End Point Update
gm1=HitGridUpdate(gm,z);
figure(1);
ShowGridMap(gm1,z);%
% Likelihood Update
gm2=LikelihoodUpdate(gm,z);
figure(2);
ShowGridMap(gm2,z);%

% Ray Casting Update
gm3=RayCastingUpdate(gm,z);
figure(3);
ShowGridMap(gm3,z);%

function gm=RayCastingUpdate(gm,z)
gm=PreCasting(gm,z.ANGLE_TICK);
rayId=0;
for iz=1:length(z.data(:,1))%‚
    range=z.data(iz,1);
    rayId=rayId+1;%
    % update
    for ir=1:length(gm.precasting(rayId).grid(:,1))
        grange=gm.precasting(rayId).grid(ir,1);
        gid=gm.precasting(rayId).grid(ir,5);
        
        if grange<(range-gm.RESO/2) %free
            gm.data(gid)=0;
        elseif grange<(range+gm.RESO/2) %hit
            gm.data(gid)=1;
        end %grid‚Íunknown
    end
end

function gm=PreCasting(gm,angleTick)
precasting=[];%
for ia=(0-angleTick/2):angleTick:(360+angleTick/2)
    ray.minAngle=ia;
    ray.maxAngle=ia+angleTick;
    grid=[];%
    for ig=1:(gm.nGrid)
        gxy=GetXYFromDataIndex(ig,gm);
        range=norm(gxy);
        angle=atan2(gxy(2),gxy(1));
        if angle<0 %[0 360]
            angle=angle+2*pi;
        end
        if ray.minAngle<=toDegree(angle) && ray.maxAngle>=toDegree(angle)
            grid=[grid;[range,angle,gxy,ig]];
        end
    end
    % range
    if ~isempty(grid)
        ray.grid=sortrows(grid,1);
    end
    precasting=[precasting;ray];
end
gm.precasting=precasting;%Grid Map


function gm=LikelihoodUpdate(gm,z)
for ig=1:(gm.nGrid-1)
    gxy=GetXYFromDataIndex(ig,gm);%
    zxy=FindNearest(gxy,z);%
    p=GaussLikelihood(gxy,zxy);%
    gm.data(ig)=p*10;%
end

function p=GaussLikelihood(gxy,zxy)
Sigma=diag([3,3]);%
p=det(2*pi*Sigma)^(-0.5)*exp(-0.5*(gxy-zxy)*inv(Sigma)*(gxy-zxy)');

function zxy=FindNearest(xy,z)
d=z.data(:,3:4)-repmat(xy,length(z.data(:,1)),1);
min=inf;%
minid=0;
for id=1:length(d(:,1))
    nd=norm(d(id,:));
    if min>nd
        min=nd;
        minid=id;
    end
end
zxy=z.data(minid,3:4);

function xy=GetXYFromDataIndex(ig,gm)
%Grid‚ x,y
indy=rem(ig,gm.WIDTH)-1;
indx=fix(ig/gm.WIDTH);

x=GetXYPosition(indx,gm.WIDTH,gm.RESO);
y=GetXYPosition(indy,gm.HEIGHT,gm.RESO);
xy=[x y];

function position=GetXYPosition(index, width, resolution)
position=resolution*(index-width/2)+resolution/2;

function gm=HitGridUpdate(gm,z)
for iz=1:length(z.data(:,1))
    zx=z.data(iz,3);
    zy=z.data(iz,4);
    ind=GetDBIndexFromXY(zx,zy,gm);
    gm.data(ind)=1.0;
end
gm.data=Normalize(gm.data);%

function ind=GetDBIndexFromXY(x,y,gm)
%xy
x=x-gm.CENTER(1);
y=y-gm.CENTER(2);
%
indX=GetXYIndex(x,gm.WIDTH, gm.RESO);%
indY=GetXYIndex(y,gm.HEIGHT,gm.RESO)+1;%
ind=GetDBIndexFromIndex(indX,indY,gm);%

function ind=GetDBIndexFromIndex(indX,indY,gm)
ind=gm.WIDTH*indX+indY;
if(indX>=gm.WIDTH)
    ind=-1;
elseif(indY>=gm.HEIGHT)
    ind=-1;        
elseif(ind>=gm.nGrid)
    ind=-1;
end

function ind=GetXYIndex(position, width, resolution)
ind=fix((position/resolution+width/2.0));

function z=GetObservation()
%
z.data=[];%[range, angle x y;...]
z.ANGLE_TICK=20;%[deg] 5, 10, 20
z.MAX_RANGE=50;%[m]
z.MIN_RANGE=5;%[m]

for angle=0:z.ANGLE_TICK:360
    range=rand()*(z.MAX_RANGE-z.MIN_RANGE)+z.MIN_RANGE;
    rad=toRadian(angle);
    x=range*cos(rad);
    y=range*sin(rad);
    z.data=[z.data;[range rad x y]]; 
end

function ShowGridMap(gm,z)
%
xmin=gm.CENTER(1)-gm.WIDTH*gm.RESO/2;
xmax=gm.CENTER(1)+gm.WIDTH*gm.RESO/2-gm.RESO;
ymin=gm.CENTER(2)-gm.HEIGHT*gm.RESO/2;
ymax=gm.CENTER(2)+gm.HEIGHT*gm.RESO/2-gm.RESO;
%XY
[X,Y] = meshgrid(xmin:gm.RESO:xmax, ymin:gm.RESO:ymax);
sizeX=size(X);
data=reshape(gm.data,sizeX(1),sizeX(2));%
surf(X,Y,data);hold on;
view(2)
%
plot3(z.data(:,3),z.data(:,4),zeros(1,length(z.data(:,1)))+1.0,'xy');
plot3(0,0,1.0,'^c');%

function Z=Normalize(Z)
%
sumZ=sum(sum(Z,1,'double'),'double');
Z=Z./sumZ;

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;





