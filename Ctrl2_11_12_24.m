clc; clear all; close all;
%% Questao 1 - a -> Calcule a funcao de transferencia discreta Gz
s = tf('s');
Gs = 1/(3*s+5);
T = 0.1;

Gz = c2d(Gs,T)

%% Questao 1 - b -> Obtenha a equacao de diferencas y(k)
[a,b] = tfdata(Gz,'v');

t = 0:T:4;
% Entrada degrau
for i = 1:length(t)
    u_s(i) = 1;
end

% Resposta ao degrau unitario
for k = 1:length(t)
    switch k
        case 1
            y(k) = 0;       
        otherwise
            y(k) = -b(2)*y(k-1)+a(2)*u_s(k-1);
    end
end

figure,
stairs(t,u_s,'LineWidth',2); hold on; stairs(t,y,'LineWidth',3); grid on;
title('Resposta ao degrau unitario'); xlabel('Tempo(s)')

%% Questao 2 - a 
clc; clear all; close all;
% Utilizando o algoritmo da video-aula, sintonize um PID discreto para obter
% um tempo de assentamento de 0.4s sem erro em regime permanente para uma
% entrada degrau unitario

% Pontos iniciais
kp_ini = 0.1; ki_ini = 0.1; kd_ini = 0.1;
x0 = [kp_ini ki_ini kd_ini];

% Limites maximos e minimos dos parametros
kp_max = 45; ki_max = 45; kd_max =20;
kp_min=0; ki_min = 0; kd_min = 0;
v_max = [kp_max ki_max kd_max];
v_min = [kp_min ki_min kd_min];

% Tempo Resposta Desejado (Tempo de assentamento)
ts_des = 0.4;

% Tempo de amostragem e simulacao
Ts = 0.1;
tsim = 5;
tempo = 0:Ts:tsim-Ts;

% Definicao do perfil de entrada - Degrau unitario
u = ones(length(tempo),1);

% Saida Referencia - Degrau filtrado
G_ref = tf(1,[ts_des/4 1]);
yref = lsim(G_ref,u,tempo);

% Modelo da Planta Continua
K = 1/5;
tau = 3/5;
Gs = tf(K,[tau 1]);

% Modelo da Planta Cotinua
Gz = c2d(Gs,Ts);

% Determinacao dos coeficientes da Planta
[a,b] = tfdata(Gz,'v');

% Definicao do problema de otimizacao
%options = optimoptions('ga', 'Display', 'iter', 'MaxGenerations', 100, 'PopulationSize', 50); 
%[p, fval] = ga(@(params) FO_PID_wrap(params, yref, Ts, tsim, a, b), 3, [], [], [], [], v_min, v_max,[], options);
options = optimoptions('fmincon', 'Display', 'iter', 'MaxFunctionEvaluations', 3000, 'MaxIterations', 1000);
[p, fval] = fmincon(@FO_PID, x0, [], [], [], [], v_min, v_max, [], options, yref, Ts, tsim, a, b)

% Simulacao em Malha Fechada com parametros otimos
y_opt = Planta_PID(p(1),p(2),p(3),Ts,tempo,a,b);

% Plotar Resultados Obtidos
plot(tempo,y_opt,tempo,yref); grid on;
legend('Sintonia PID Otimizado','Referencia'); title('Resposta ao Degrau Filtrado - PID');
xlabel('Tempo(s)');
