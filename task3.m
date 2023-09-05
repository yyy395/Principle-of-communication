clc
clear
close all
%% 参数设置
M = 8;                                 % 调制阶数
L_data = 3*1e6;                           % 数据长度
L_symbol = L_data/log2(M);              % 符号长度
data = round(rand(1,L_data));           % 原始数据
data_1=rand(1,L_symbol);
data_2=data;
data_3=data;
Pc=[0.1,0.2,0.05,0.15,0.1,0.25,0.1,0.05];
% SNR变化 % 随着SNR增加 BLER减少
EsN0_dB = 5:2:17;                         % Es/N0 dB形式
EsN0 = 10.^(EsN0_dB/10);                % 每符号能量/噪声
% 累计3bit能量
EbN0 = EsN0/log2(M);                  % 每比特能量/噪声
error = zeros(1,length(EsN0_dB));       % 预置错误符号个数
ber_error1=zeros(1,length(EsN0_dB));
ser = zeros(1,length(EsN0_dB));         % 预置仿真误符号率
ber = zeros(1,length(EsN0_dB));
ber_rand=zeros(1,length(EsN0_dB));
tser_8QAM_1 = zeros(1,length(EsN0_dB));  % 预置8ary理论误符号率
tser_8QAM_2 = zeros(1,length(EsN0_dB));
tser_8QAM_3 = zeros(1,length(EsN0_dB));
label=[[0,0,0],[0,0,1],[0,1,0],[0,1,1],[1,0,0],[1,0,1],[1,1,0],[1,1,1]];
rand_label=zeros(1,length(label));
n=randperm(8);
for i=1:8
    rand_label(3*i-2:3*i)=label(3*n(i)-2:3*n(i));
end
%% 16QAM调制
% I Q两个方向的调制 
send = zeros(1,L_symbol);               % 预设发送信号
send_set = [-2+2j,2j,2+2j,-2,2,-2-2j,-2j,2-2j];     % 发射端星座点
%等概率发送
Es_avg = sum(Pc.*abs(send_set).^2);
N0 = Es_avg ./ EsN0;
% bit 映射 二进制数-- 星座图的复数 % 格雷码的编码方式
for q = 1:L_symbol
    if (data_1(q)>0&&data_1(q)<=0.1)
        data_2(3*q-2:3*q) = [0,0,0];
        data_3(3*q-2:3*q) = rand_label(1:3);
        send(q) = send_set(1);          % 000 => -2+2j
    elseif (data_1(q)>0.1&&data_1(q)<=0.3)
        data_2(3*q-2:3*q) = [0,0,1];
        data_3(3*q-2:3*q) = rand_label(4:6);
        send(q) = send_set(2);          % 001 => +2j
    elseif (data_1(q)>0.3&&data_1(q)<=0.35)
        data_2(3*q-2:3*q) = [0,1,1];
        data_3(3*q-2:3*q) = rand_label(7:9);
        send(q) = send_set(3);          % 011 => +2+2j
    elseif (data_1(q)>0.35&&data_1(q)<=0.5)
        data_2(3*q-2:3*q) = [1,0,0];
        data_3(3*q-2:3*q) = rand_label(10:12);
        send(q) = send_set(4);          % 100 => -2
    elseif (data_1(q)>0.5&&data_1(q)<=0.6)
        data_2(3*q-2:3*q) = [0,1,0];
        data_3(3*q-2:3*q) = rand_label(13:15);
        send(q) = send_set(5);          % 010 => +2
    elseif (data_1(q)>0.6&&data_1(q)<=0.85)
        data_2(3*q-2:3*q) = [1,0,1];
        data_3(3*q-2:3*q) = rand_label(16:18);
        send(q) = send_set(6);          % 101 => -2-2j
    elseif (data_1(q)>0.85&&data_1(q)<=0.95)
        data_2(3*q-2:3*q) = [1,1,1];
        data_3(3*q-2:3*q) = rand_label(19:21);
        send(q) = send_set(7);          % 111 => -2j
    elseif (data_1(q)>0.95&&data_1(q)<=1)
        data_2(3*q-2:3*q) = [1,1,0];
        data_3(3*q-2:3*q) = rand_label(22:24);
        send(q) = send_set(8);          % 110 => +2-2j

    end
end
err_bit=zeros(1,8);
for j=1:3
    if(rand_label(j)~=rand_label(j+3))
        err_bit(1)=err_bit(1)+1;
        err_bit(2)=err_bit(2)+1;
    end
    if(rand_label(j)~=rand_label(j+9))
        err_bit(1)=err_bit(1)+1;
        err_bit(4)=err_bit(4)+1;
    end
    if(rand_label(j+3)~=rand_label(j+6))
        err_bit(2)=err_bit(2)+1;
        err_bit(3)=err_bit(3)+1;
    end
    if(rand_label(j+6)~=rand_label(j+12))
        err_bit(3)=err_bit(3)+1;
        err_bit(5)=err_bit(5)+1;
    end
    if(rand_label(j+9)~=rand_label(j+15))
        err_bit(4)=err_bit(4)+1;
        err_bit(6)=err_bit(6)+1;
    end
    if(rand_label(j+12)~=rand_label(j+21))
        err_bit(5)=err_bit(5)+1;
        err_bit(8)=err_bit(8)+1;
    end
    if(rand_label(j+15)~=rand_label(j+18))
        err_bit(6)=err_bit(6)+1;
        err_bit(7)=err_bit(7)+1;
    end
    if(rand_label(j+18)~=rand_label(j+21))
        err_bit(7)=err_bit(7)+1;
        err_bit(8)=err_bit(8)+1;
    end
end
for q = 1:length(EsN0_dB)
    % 噪声 noise
    noise = sqrt(N0(q)/2)*randn(1,L_symbol) + 1j*sqrt(N0(q)/2)*randn(1,L_symbol);   % AWGN
    receive = (send + noise);       % 接收信号
    detect = zeros(1,L_symbol);         % 预置检测信号
    distance = zeros(1,M);              % 解调：距离检测 
    for t = 1:L_symbol
        for w = 1:M
            %distance(w) = norm(receive(t) - send_set(w))^2;     % 接收信号到所有星座点的距离
            distance(w)=Pc(w)*(1/sqrt(pi*N0(q)))*exp(-norm(receive(t) - send_set(w))^2/N0(q));
        end
        pos = find(distance == max(distance));      % 最小距离星座点的位置
        detect(t) = send_set(pos);                  % 解调后的符号% 接收端信号
        idx=0;
        for l=1:8
            if(send(t)==send_set(l))
                idx=l;
            end
        end
        if (detect(t) ~= send(t)) 
            error(q) = error(q) + 1;                % 统计错误符号数
            ber_label1=rand_label(3*pos-2:3*pos);
            ber_label2=rand_label(3*idx-2:3*idx);
            for k=1:3
                if(ber_label1(k)~=ber_label2(k))
                    ber_error1(q)=ber_error1(q)+1;
                end
            end
        end
    end
    ser(q) = error(q)/L_symbol;                     % 8ary仿真误符号率
    ber(q)=error(q)/L_data;
    ber_rand(q)=ber_error1(q)/L_data;
    %tser_8QAM(q) = 3*qfunc(sqrt(4/5*EbN0(q)))*(1-3/4*qfunc(sqrt(4/5*EbN0(q))));   % 8QAM理论误符号率
    tser_8QAM_1(q) = 2*qfunc(sqrt(2/N0(q)))/3;   % 8ary最优理论误比特率
    tser_8QAM_2(q)=(0.45*(2*qfunc(sqrt(2/N0(q)))+2*qfunc(sqrt(8/N0(q)))+2*qfunc(sqrt(10/N0(q)))+qfunc(sqrt(16/N0(q))))+0.55*(2*qfunc(sqrt(2/N0(q)))+2*qfunc(sqrt(4/N0(q)))+2*qfunc(sqrt(10/N0(q)))+qfunc(sqrt(8/N0(q)))))/3;  %标准邻接上界理论误比特率
    tser_8QAM_3(q)=0.1*err_bit(1)*qfunc(sqrt(2/N0(q)))/3+0.2*err_bit(2)*qfunc(sqrt(2/N0(q)))/3+0.05*err_bit(3)*qfunc(sqrt(2/N0(q)))/3+0.15*err_bit(4)*qfunc(sqrt(2/N0(q)))/3+0.1*err_bit(5)*qfunc(sqrt(2/N0(q)))/3+0.25*err_bit(6)*qfunc(sqrt(2/N0(q)))/3+0.1*err_bit(7)*qfunc(sqrt(2/N0(q)))/3+0.05*err_bit(8)*qfunc(sqrt(2/N0(q)))/3;  %随机编码理论误比特率
end
figure
semilogy(EsN0_dB,ber,'o',EsN0_dB,ber_rand,'*',EsN0_dB,tser_8QAM_1,'b',EsN0_dB,tser_8QAM_2,'g',EsN0_dB,tser_8QAM_3,'r');     % 画图
grid on;                                        % 坐标轴开启
axis([5 17 10^-6 10^0])                        % 限制作图范围
xlabel('Eb/N0 (dB)');                           % 横坐标
ylabel('BER');                                  % 纵坐标
legend('8-ary格雷码编码仿真误比特率','8-ary随机编码仿真误比特率','8-ary理论最优误比特率','stanard union upper bound','随机编码理论误码率');   % 图例