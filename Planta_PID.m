function y1 = Planta_PID(kp,ki,kd,Ts,t,a,b)
    % Degrau unitario
    r = ones(length(t),1);
    
    % Pre alocando tamanho dos vetores da iteracao
    y = zeros(length(t),1); 
    e = zeros(length(t),1);
    u = zeros(length(t),1);
    
    for k = 1:length(t)
        switch k
            case 1
                y(k) = 0;
                e(k) = r(k)-y(k);
                u(k) = kp*e(k)+ki*Ts*e(k)+(kd/Ts)*e(k);
            case 2
                y(k) = (-b(2)*y(k-1)+a(2)*u(k-1))/b(1);
                e(k) = r(k)-y(k);
                u(k) = (kp*e(k)-kp*e(k-1))+ki*Ts*e(k)+(kd/Ts)*(e(k)-2*e(k-1))+u(k-1);
            otherwise
                y(k) = (-b(2)*y(k-1)+a(2)*u(k-1))/b(1);
                e(k) = r(k)-y(k);
                u(k) = (kp*e(k)-kp*e(k-1))+(ki*Ts*e(k))+(kd/Ts)*(e(k)-2*e(k-1)+e(k-2))+u(k-1);
        end
    end
    y1=y;
end