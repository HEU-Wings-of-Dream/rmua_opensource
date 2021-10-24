clear all;
close all;
nowx = 0;nowy = 0;angle = 0;
goalx = 1;goaly = 1;errorx = 0;errory = 0;
vx_ = 0;vy_ = 0; %PID修正速度
vx = 0;vy = 0;  %实际速度
NOWX = [];NOWY = [];
ERRORX = [];ERRORY = [];
GOAL=[];
VX = [];VY = [];V = [];VX_=[];VY_=[];
y=randn(1,2500)/20.0;
dt = 0.05; %dt = 0.1
kp = 1;Ti = 1;Td = 0.1;
Ix = 0;Iy = 0;
last_errorx = goalx - nowx; last_errory = goaly - nowy;

for i = 1:1:300 %i为模拟次数
    GOAL = [GOAL,1];
    
    %计算误差
    errorx = goalx - nowx;
    errory = goaly - nowy;
    
    Ix = Ix + errorx * dt;
    Iy = Iy + errory * dt;
    
    %控制vx
    vx_ = kp * errorx  + kp * Td * (last_errorx - errorx) / dt;%+ kp / Ti * Ix
    %控制vy
    vy_ = kp * errory  + kp * Td * (last_errory - errory) / dt;%+ kp / Ti * Iy
    
    nowx = nowx + vx_ * dt + y(i); %更新位置  
    nowy = nowy + vy_ * dt + y(i); %更新位置
    
    NOWX = [NOWX, nowx];NOWY = [NOWY, nowy];
    VX = [VX,vx];VY = [VY,vy];
    VX_ = [VX_,vx_];VY_ = [VY_,vy_];
    ERRORX = [ERRORX,errorx];ERRORY = [ERRORY,errory];
    
    %lastdis = dis;
    last_errorx = errorx;
    last_errory = errory;
end

figure(1);hold on; plot(NOWX,'r');  plot(GOAL,'b'); hold off; title('x坐标（红）与期望坐标（蓝）');
%figure(2);plot(V);title('v');
figure(2); hold on; plot(VX_,'b'); plot(ERRORX,'g'); title( '控制下的Vx(蓝) && errorx(绿)');hold off;
figure(3); hold on; plot(VY_,'b'); plot(ERRORY,'g'); title( '控制下的Vy(蓝) && errory(绿)');hold off;
figure(4); plot(1:2500,y);

















