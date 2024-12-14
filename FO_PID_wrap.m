function cost = FO_PID_wrap(params, yref, Ts, tsim, a, b)
    cost = FO_PID(params, yref, Ts, tsim, a, b);
end
