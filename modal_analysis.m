function modal_analysis(Veq, A, my_rate_func)

    Q = -A(4:6, 1:3);
    
    [V, D] = eig(Q);
    eigenvalues = diag(D);
    omega_n_values = sqrt(abs(eigenvalues));
    U = V;
    for i = 1:3
        U_mode = U(:,i);
        omega_n = omega_n_values(i);
        %small number
        epsilon = 0.1;

        V0 = Veq + epsilon*[U_mode;0;0;0];
        tspan = [0 20];

        %run the integration of nonlinear system
        [tlist_nonlinear,Vlist_nonlinear] = ode45(my_rate_func,tspan,V0);
        
        tlist_nonlinear = tlist_nonlinear';
        Vlist_nonlinear = Vlist_nonlinear';

        %Linear Sim
        linear_rate = @(t, V) A*(V - Veq);

        [tlist_lin, Vlist_lin] = ode45(linear_rate, tspan, V0);
        tlist_lin = tlist_lin';
        Vlist_lin = Vlist_lin';
    
        x_modal = Veq(1)+epsilon*U_mode(1)*cos(omega_n*tlist_nonlinear);
        y_modal = Veq(2)+epsilon*U_mode(2)*cos(omega_n*tlist_nonlinear);
        theta_modal = Veq(3)+epsilon*U_mode(3)*cos(omega_n*tlist_nonlinear);

        figure();
        
        plot(tlist_nonlinear, Vlist_nonlinear(1,:), 'b');
        hold on;
        plot(tlist_lin, Vlist_lin(1,:), 'r--');
        plot(tlist_nonlinear, x_modal, 'k:');
        ylabel('X(t)');
        xlabel('Time (s)');
        legend('Nonlinear','Linear','Modal');

        plot(tlist_nonlinear, Vlist_nonlinear(2,:), 'b');
        hold on;
        plot(tlist_lin, Vlist_lin(2,:), 'r--');
        plot(tlist_nonlinear, y_modal, 'k:');
        ylabel('Y(t)');
        xlabel('Time (s)');
        legend('Nonlinear','Linear','Modal');
        
        plot(tlist_nonlinear, Vlist_nonlinear(3,:), 'b');
        plot(tlist_lin, Vlist_lin(3,:), 'r--');
        plot(tlist_nonlinear, theta_modal, 'k:');

        ylabel('\theta(t)');
        xlabel('Time (s)');
        legend('Nonlinear','Linear','Modal');
        hold off;
    end
end