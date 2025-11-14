function modal_analysis(Veq, A)

    Q = -A(4:6, 1:3);
    
    [V, D] = eig(Q);
    eigenvalues = diag(D);
    omega_n_values = sqrt(eigenvalues);
    U = V;
    for i = 1:3
        U_mode = U(:,i);
        omega_n = omega_n_values(i);
        %small number
        V0 = Veq + epsilon*[U_mode;0;0;0];
        epsilon = 0.1;
        tspan = [0 20];
        %run the integration of nonlinear system
        [tlist_nonlinear,Vlist_nonlinear] = ode45(my_rate_func,tspan,V0);

    
        x_modal = Veq(1)+epsilon*Umode(1)*cos(omega_n*tlist);
        y_modal = Veq(2)+epsilon*Umode(2)*cos(omega_n*tlist);
        theta_modal = Veq(3)+epsilon*Umode(3)*cos(omega_n*tlist);

        figure();
        hold on;
        plot(tlist_nonlinear, Vlist_nonlinear(1,:));
        plot(tlist_nonlinear, Vlist_nonlinear(2,:));
        plot(tlist_nonlinear, Vlist_nonlinear(3,:));        
        plot(tlist_nonlinear, x_modal);
        plot(tlist_nonlinear, y_modal);
        plot(tlist_nonlinear, theta_modal);
        hold off;
    end
end