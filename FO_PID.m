function f = FO_PID(x0,yref,Ts,t,a,b)

    % Parametros a serem otimizados
    kp = x0(1);
    ki = x0(2);
    kd = x0(3);

    %Simulacao do sistema
    tempo = 0:Ts:t-Ts;
    y = Planta_PID(kp,ki,kd,Ts,tempo,a,b);

    % Funcao custo Obtida pelos Objetivos do Controle
    f = sqrt(sum(abs(y-yref).^2));
end