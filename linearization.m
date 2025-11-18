function linearization()
    my_rate_func = @(t,V) box_rate_func(t, V, box_params);
    my_linear_rate = @(t,V) A * (V - Veq);

    epsilon = 0.1;
    V0 = Veq + [0; 0; epsilon; 0; 0; 0]; 
    tspan = 0:0.05:10;

    [t_nonlin, V_nonlin] = ode45(my_rate_func, tspan, V0);
    [t_lin, V_lin] = ode45(my_linear_rate, tspan, V0);

    figure()
    plot(tlist_nonlinear, Vlist_nonlinear(1,:));
    hold on;
    plot(tlist_lin, Vlist_lin(1,:));
    plot(tlist_nonlinear, Vlist_nonlinear(2,:));
    plot(tlist_lin, Vlist_lin(2,:));
    plot(tlist_nonlinear, Vlist_nonlinear(3,:));
    plot(tlist_lin, Vlist_lin(3,:));
    title('Linear vs Nonlinear Approximation')
    ylabel('Distance from Equilibrium');
    xlabel('Time (s)');
    legend('nonlinX', 'linX', 'nonlinY', 'linY', 'nonlin\theta', 'lin\theta');

    hold off;
end