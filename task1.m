clc
clear
close all
%% 参数设置
M = 8;                                 % 调制阶数
L_data = 3*1e3;                           % 数据长度
L_symbol = L_data/log2(M);              % 符号长度
Pc=[0.1,0.2,0.05,0.15,0.1,0.25,0.1,0.05];
send_set = [-2+2j,2j,2+2j,-2,2,-2-2j,-2j,2-2j];     % 发射端星座点
N0=0.9;
color=zeros(1,length(L_symbol));
% 噪声 noise
x_1=-3:6/L_symbol:3-6/L_symbol;
y_1=-3:6/L_symbol:3-6/L_symbol;
dot=zeros(1,length(L_symbol));
for i=1:length(x_1)
    for j=1:length(y_1)
        dot(j)=x_1(i)+y_1(j)*1j;
    end
    detect = zeros(1,L_symbol);         % 预置检测信号
    distance = zeros(1,M);              % 解调：距离检测 
    for t = 1:L_symbol
        for w = 1:M
            %distance(w) = norm(dot(t) - send_set(w))^2;     % 接收信号到所有星座点的距离
            distance(w)=-N0*log(Pc(w))+norm(dot(t) - send_set(w))^2;
        end
        pos = find(distance == min(distance));      % 最小距离星座点的位置
        pos=min(pos);
        color(t)=pos;
    end
    scatter(real(dot),imag(dot),1,color);
    hold on
end
send_set = [-2+2j,2j,2+2j,-2,2,-2-2j,-2j,2-2j];
scatter(real(send_set),imag(send_set),30,'filled');
axis([-3 3 -3 3])
hold on;
rectangle('Position',[-2, -2, 4, 4],'Curvature',[1, 1]);axis equal; % 画圆
rectangle('Position',[-2, -2, 4, 4],'Curvature',[0, 0],'LineStyle','--');axis equal; % 画矩形
rectangle('Position',[-2*sqrt(2), -2*sqrt(2), 4*sqrt(2), 4*sqrt(2)],'Curvature',[1, 1]);axis equal; % 画圆