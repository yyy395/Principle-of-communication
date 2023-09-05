clc
clear
close all
%% ��������
M = 8;                                 % ���ƽ���
L_data = 3*1e6;                           % ���ݳ���
L_symbol = L_data/log2(M);              % ���ų���
data = round(rand(1,L_data));           % ԭʼ����
data_1=rand(1,L_symbol);
Pc=[0.1,0.2,0.05,0.15,0.1,0.25,0.1,0.05];
% SNR�仯 % ����SNR���� BLER����
EsN0_dB = 5:2:17;                         % Es/N0 dB��ʽ
EsN0 = 10.^(EsN0_dB/10);                % ÿ��������/����
% �ۼ�3bit����
EbN0 = EsN0/log2(M);                  % ÿ��������/����
error = zeros(1,length(EsN0_dB));       % Ԥ�ô�����Ÿ���
error_1 = zeros(1,length(EsN0_dB));
ser = zeros(1,length(EsN0_dB));         % Ԥ�÷����������
ser_1 = zeros(1,length(EsN0_dB));
tser_8QAM_1 = zeros(1,length(EsN0_dB));  % Ԥ��8ary�����������
tser_8QAM_2 = zeros(1,length(EsN0_dB));
% I Q��������ĵ��� 
send = zeros(1,L_symbol);               % Ԥ�跢���ź�
send_set = [-2+2j,2j,2+2j,-2,2,-2-2j,-2j,2-2j];     % �����������
x=[-2,0,2,-2,2,-2,0,2];
y=[2,2,2,0,0,-2,-2,-2];
%�ȸ��ʷ���
Es_avg = sum(Pc.*abs(send_set).^2);
N0 = Es_avg ./ EsN0;
% bit ӳ�� ��������-- ����ͼ�ĸ��� % ������ı��뷽ʽ
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
    % ���� noise
    noise = sqrt(N0(q)/2)*randn(1,L_symbol) + 1j*sqrt(N0(q)/2)*randn(1,L_symbol);   % AWGN
    receive = (send + noise);       % �����ź�
    detect = zeros(1,L_symbol);         % Ԥ�ü���ź�
    detect_1 = zeros(1,L_symbol);
    distance = zeros(1,M);              % �����������
    distance_1 = zeros(1,M);
    for t = 1:L_symbol
        for w = 1:M
            distance_1(w) = norm(receive(t) - send_set(w))^2;
            distance(w)=Pc(w)*(1/sqrt(pi*N0(q)))*exp(-norm(receive(t) - send_set(w))^2/N0(q));  % �����źŵ�����������ľ���
        end
        pos = find(distance == max(distance));      % ��С�����������λ��
        pos_1 = find(distance_1 == min(distance_1));
        detect(t) = send_set(pos);                  % �����ķ���% ���ն��ź�
        detect_1(t) = send_set(pos_1);
        if (detect(t) ~= send(t)) 
            error(q) = error(q) + 1;                % ͳ�ƴ��������
        end
        if (detect_1(t) ~= send(t)) 
            error_1(q) = error_1(q) + 1;                % ͳ�ƴ��������
        end
    end
    ser(q) = error(q)/L_symbol;                     % 8ary�����������
    ser_1(q) = error_1(q)/L_symbol;
    tser_8QAM_1(q) = 2*qfunc(sqrt(2/N0(q)));   % 8ary����ڽ��Ͻ������������
    tser_8QAM_2(q)=0.45*(2*qfunc(sqrt(2/N0(q)))+2*qfunc(sqrt(8/N0(q)))+2*qfunc(sqrt(10/N0(q)))+qfunc(sqrt(16/N0(q))))+0.55*(2*qfunc(sqrt(2/N0(q)))+2*qfunc(sqrt(4/N0(q)))+2*qfunc(sqrt(10/N0(q)))+qfunc(sqrt(8/N0(q))));  % 8ary��׼�ڽ��Ͻ������������
end
figure
semilogy(EsN0_dB,ser,'o',EsN0_dB,ser_1,'*',EsN0_dB,tser_8QAM_1,'b',EsN0_dB,tser_8QAM_2,'r');     % ��ͼ
grid on;                                        % �����Ὺ��
axis([5 17 10^-6 10^0])                        % ������ͼ��Χ
xlabel('Eb/N0 (dB)');                           % ������
ylabel('SER');                                  % ������
legend('8-ary���Ž��ջ������������','8-ary��С��������������','��������Ͻ�','��׼�����Ͻ�');   % ͼ��