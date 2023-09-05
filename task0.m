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
EsN0_dB = 9:2:17;                         % Es/N0 dB��ʽ
EsN0 = 10.^(EsN0_dB/10);                % ÿ��������/����
% �ۼ�3bit����
EbN0 = EsN0/log2(M);                  % ÿ��������/����
error = zeros(1,length(EsN0_dB));       % Ԥ�ô�����Ÿ���
ser = zeros(1,length(EsN0_dB));         % Ԥ�÷����������
tser_8ary = zeros(1,length(EsN0_dB));  % Ԥ��8ary�����������
%% 16QAM����
% I Q��������ĵ��� 
send = zeros(1,L_symbol);               % Ԥ�跢���ź�
send_set = [-2+2j,2j,2+2j,-2,2,-2-2j,-2j,2-2j];     % �����������
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
    distance = zeros(1,M);              % ����������� 
    for t = 1:L_symbol
        for w = 1:M
            distance(w) = norm(receive(t) - send_set(w))^2;     % �����źŵ�����������ľ���
        end
        pos = find(distance == min(distance));      % ��С�����������λ��
        detect(t) = send_set(pos);                  % �����ķ���% ���ն��ź�
        if (detect(t) ~= send(t)) 
            error(q) = error(q) + 1;                % ͳ�ƴ��������
        end
    end
    ser(q) = error(q)/L_symbol;                     % 8ary�����������
    tser_8ary(q) = 2*qfunc(sqrt(2/N0(q)));   % 8ary�����������
end
figure
semilogy(EsN0_dB,ser,'o',EsN0_dB,tser_8ary,'b');     % ��ͼ
grid on;                                        % �����Ὺ��
axis([9 17 10^-6 10^0])                        % ������ͼ��Χ
xlabel('Es/N0 (dB)');                           % ������
ylabel('SER');                                  % ������
legend('8-ary�����������','8-ary�����������');   % ͼ��
