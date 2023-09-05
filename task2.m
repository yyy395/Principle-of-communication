clc
clear
close all
%% 参数设置
M = 8;                                 % 调制阶数
L_data = 3*1e6;                           % 数据长度
L_symbol = L_data/log2(M);              % 符号长度
data = round(rand(1,L_data));           % 原始数据
data_1=rand(1,L_symbol);
Pc=[0.1,0.2,0.05,0.15,0.1,0.25,0.1,0.05];
% SNR变化 % 随着SNR增加 BLER减少
EsN0_dB = 5:2:17;                         % Es/N0 dB形式
EsN0 = 10.^(EsN0_dB/10);                % 每符号能量/噪声
% 累计3bit能量
EbN0 = EsN0/log2(M);                  % 每比特能量/噪声
error = zeros(1,length(EsN0_dB));       % 预置错误符号个数
error_1 = zeros(1,length(EsN0_dB));
ser = zeros(1,length(EsN0_dB));         % 预置仿真误符号率
ser_1 = zeros(1,length(EsN0_dB));
tser_8QAM_1 = zeros(1,length(EsN0_dB));  % 预置8ary理论误符号率
tser_8QAM_2 = zeros(1,length(EsN0_dB));
% I Q两个方向的调制 
send = zeros(1,L_symbol);               % 预设发送信号
send_set = [-2+2j,2j,2+2j,-2,2,-2-2j,-2j,2-2j];     % 发射端星座点
x=[-2,0,2,-2,2,-2,0,2];
y=[2,2,2,0,0,-2,-2,-2];
%等概率发送
Es_avg = sum(Pc.*abs(send_set).^2);
N0 = Es_avg ./ EsN0;
% bit 映射 二进制数-- 星座图的复数 % 格雷码的编码方式
for q = 1:L_symbol
    if (data_1(q)>0&&data_1(q)<=0.1)
        data(3*q-2:3*q) = [0,0,0];
        send(q) = send_set(1);          % 000 => -2+2j
    elseif (data_1(q)>0.1&&data_1(q)<=0.3)
        data(3*q-2:3*q) = [0,0,1];
        send(q) = send_set(2);          % 001 => +2j
    elseif (data_1(q)>0.3&&data_1(q)<=0.35)
        data(3*q-2:3*q) = [0,1,1];
        send(q) = send_set(3);          % 011 => +2+2j
    elseif (data_1(q)>0.35&&data_1(q)<=0.5)
        data(3*q-2:3*q) = [1,0,0];
        send(q) = send_set(4);          % 100 => -2
    elseif (data_1(q)>0.5&&data_1(q)<=0.6)
        data(3*q-2:3*q) = [0,1,0];
        send(q) = send_set(5);          % 010 => +2
    elseif (data_1(q)>0.6&&data_1(q)<=0.85)
        data(3*q-2:3*q) = [1,0,1];
        send(q) = send_set(6);          % 101 => -2-2j
    elseif (data_1(q)>0.85&&data_1(q)<=0.95)
        data(3*q-2:3*q) = [1,1,1];
        send(q) = send_set(7);          % 111 => -2j
    elseif (data_1(q)>0.95&&data_1(q)<=1)
        data(3*q-2:3*q) = [1,1,0];
        send(q) = send_set(8);          % 110 => +2-2j

    end
end
for q = 1:length(EsN0_dB)
    % 噪声 noise
    noise = sqrt(N0(q)/2)*randn(1,L_symbol) + 1j*sqrt(N0(q)/2)*randn(1,L_symbol);   % AWGN
    receive = (send + noise);       % 接收信号
    detect = zeros(1,L_symbol);         % 预置检测信号
    detect_1 = zeros(1,L_symbol);
    distance = zeros(1,M);              % 解调：距离检测
    distance_1 = zeros(1,M);
    for t = 1:L_symbol
        for w = 1:M
            distance_1(w) = norm(receive(t) - send_set(w))^2;
            distance(w)=Pc(w)*(1/sqrt(pi*N0(q)))*exp(-norm(receive(t) - send_set(w))^2/N0(q));  % 接收信号到所有星座点的距离
        end
        pos = find(distance == max(distance));      % 最小距离星座点的位置
        pos_1 = find(distance_1 == min(distance_1));
        detect(t) = send_set(pos);                  % 解调后的符号% 接收端信号
        detect_1(t) = send_set(pos_1);
        if (detect(t) ~= send(t)) 
            error(q) = error(q) + 1;                % 统计错误符号数
        end
        if (detect_1(t) ~= send(t)) 
            error_1(q) = error_1(q) + 1;                % 统计错误符号数
        end
    end
    ser(q) = error(q)/L_symbol;                     % 8ary仿真误符号率
    ser_1(q) = error_1(q)/L_symbol;
    tser_8QAM_1(q) = 2*qfunc(sqrt(2/N0(q)));   % 8ary最近邻接上界理论误符号率
    tser_8QAM_2(q)=0.45*(2*qfunc(sqrt(2/N0(q)))+2*qfunc(sqrt(8/N0(q)))+2*qfunc(sqrt(10/N0(q)))+qfunc(sqrt(16/N0(q))))+0.55*(2*qfunc(sqrt(2/N0(q)))+2*qfunc(sqrt(4/N0(q)))+2*qfunc(sqrt(10/N0(q)))+qfunc(sqrt(8/N0(q))));  % 8ary标准邻接上界理论误符号率
end
figure
semilogy(EsN0_dB,ser,'o',EsN0_dB,ser_1,'*',EsN0_dB,tser_8QAM_1,'b',EsN0_dB,tser_8QAM_2,'r');     % 画图
grid on;                                        % 坐标轴开启
axis([5 17 10^-6 10^0])                        % 限制作图范围
xlabel('Eb/N0 (dB)');                           % 横坐标
ylabel('SER');                                  % 纵坐标
legend('8-ary最优接收机仿真误符号率','8-ary最小距离仿真误符号率','最近联合上界','标准联合上界');   % 图例