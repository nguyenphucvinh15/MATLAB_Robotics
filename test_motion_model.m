function test_motion_model()
%
addpath('librobotics')
%
global widthrobot lengthrobot dt
widthrobot=0.4;
lengthrobot=0.6;
dt = 0.1; % [sec]
global MIN_X MIN_Y MAX_X MAX_Y STEP
MIN_X=-8;MIN_Y=-8;MAX_X=8;MAX_Y=8; STEP = 0.1;
figure1 = figure('position', [200, 50, 700, 600]);
axis equal
axis([MIN_X+2 MAX_X MIN_Y+2 MAX_Y-2])
xlabel('X [m]'); ylabel('Y [m]');
hold on
%
x=[0,0,0,0]';% [x, y, theta, v]
X=[];
Xnoise=[];
q=0.1;
Q=q^2;
u=[5,-0.5]';
plot(x(1),x(2),'+b')
N=50;
for i=1:N
    x=f(x,u);
    X= [X,x];
    xnoise = x + Q*randn(4,1);
    Xnoise=[Xnoise,xnoise];
    drawrobot([x(1),x(2),x(3)],'m',1,widthrobot,lengthrobot);
end
plot(X(1,:),X(2,:),'+r')
plot(Xnoise(1,:),Xnoise(2,:),'+b')

function x = f(x, u)
global dt;
F = [1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 0];
B = [
    dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0];
x= F*x+B*u;