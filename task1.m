clc
clear
close all
%% ��������
M = 8;                                 % ���ƽ���
L_data = 3*1e3;                           % ���ݳ���
L_symbol = L_data/log2(M);              % ���ų���
Pc=[0.1,0.2,0.05,0.15,0.1,0.25,0.1,0.05];
send_set = [-2+2j,2j,2+2j,-2,2,-2-2j,-2j,2-2j];     % �����������
N0=0.9;
color=zeros(1,length(L_symbol));
% ���� noise
x_1=-3:6/L_symbol:3-6/L_symbol;
y_1=-3:6/L_symbol:3-6/L_symbol;
dot=zeros(1,length(L_symbol));
for i=1:length(x_1)
    for j=1:length(y_1)
        dot(j)=x_1(i)+y_1(j)*1j;
    end
    detect = zeros(1,L_symbol);         % Ԥ�ü���ź�
    distance = zeros(1,M);              % ����������� 
    for t = 1:L_symbol
        for w = 1:M
            %distance(w) = norm(dot(t) - send_set(w))^2;     % �����źŵ�����������ľ���
            distance(w)=-N0*log(Pc(w))+norm(dot(t) - send_set(w))^2;
        end
        pos = find(distance == min(distance));      % ��С�����������λ��
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
rectangle('Position',[-2, -2, 4, 4],'Curvature',[1, 1]);axis equal; % ��Բ
rectangle('Position',[-2, -2, 4, 4],'Curvature',[0, 0],'LineStyle','--');axis equal; % ������
rectangle('Position',[-2*sqrt(2), -2*sqrt(2), 4*sqrt(2), 4*sqrt(2)],'Curvature',[1, 1]);axis equal; % ��Բ