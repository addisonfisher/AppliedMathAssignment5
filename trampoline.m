function trampoline()

    % System parameters 
    LW = 10; LH = 1; LG = 3;
    m = 1; Ic = (1/12)*(LH^2 + LW^2);
    g = 1;
    k = 20;
    k_list = [.5*k, .5*k, 2*k, 5*k];
    l0 = 1.5*LG;
    
    % Box
    Pbl_box = [-LW; -LH]/2;
    Pbr_box = [ LW; -LH]/2;
    Ptl_box = [-LW;  LH]/2;
    Ptr_box = [ LW;  LH]/2;
    boundary_pts = [Pbl_box, Pbr_box, Ptr_box, Ptl_box, Pbl_box];
    
    Pbl1_world = Pbl_box + [-LG; -LG];
    Pbl2_world = Pbl_box + [ LG; -LG];
    Pbr1_world = Pbr_box + [ 0 ; -l0];
    Pbr2_world = Pbr_box + [ l0;  0 ];
    P_world = [Pbl1_world, Pbl2_world, Pbr1_world, Pbr2_world];
    
    P_box = [Pbl_box, Pbl_box, Pbr_box, Pbr_box];
    
    % pack into struct
    box_params = struct();
    box_params.m = m;
    box_params.I = Ic;
    box_params.g = g;
    box_params.k_list = k_list;
    box_params.l0_list = l0*ones(1,size(P_world,2));
    box_params.P_world = P_world;
    box_params.P_box = P_box;
    box_params.boundary_pts = boundary_pts;
    
    %Finding equilibrium
    f_no_t = @(V) box_rate_func(0, V, box_params);
    
    % Initial guess 
    Vguess = [0; -1; 0; 0; 0; 0];   

    [Veq, fval, exitflag, output] = fsolve(f_no_t, Vguess);
    
    disp("Equilibrium state found:");
    disp(Veq);
    
    disp("f(Veq) (close to 0?)= ");
    disp(f_no_t(Veq));

    %Jacobina
    A = approximate_jacobian(f_no_t, Veq);

    %MOdal Analysis
    my_rate_func = @(t,V) box_rate_func(t,V,box_params);

    %%
    my_linear_rate = @(t, V) A * (V - Veq);
    
    tspan = 0:0.01:3;
    perturbation_vec = [0; 0; 1; 0; 0; 0]; %small pertub of theta
    
    epsilon_small = 0.05; %small initial push
    V0_small = Veq + epsilon_small * perturbation_vec;
    
    %run both sims
    [t_nonlin_s, V_nonlin_s] = ode45(my_rate_func, tspan, V0_small);
    [t_lin_s, V_lin_s] = ode45(my_linear_rate, tspan, V0_small);
    
    figure();
    hold on;
    plot(t_nonlin_s, V_nonlin_s(:, 1) - Veq(1), 'r-');
    plot(t_lin_s, V_lin_s(:, 1) - Veq(1), 'r--');
    plot(t_nonlin_s, V_nonlin_s(:, 2) - Veq(2), 'b-');
    plot(t_lin_s, V_lin_s(:, 2) - Veq(2), 'b--');
    plot(t_nonlin_s, V_nonlin_s(:, 3) - Veq(3), 'k-');
    plot(t_lin_s, V_lin_s(:, 3) - Veq(3), 'k--');
    hold off;

    title('Linear vs. Nonlinear (Small Perturbation, \epsilon = 0.05)');
    ylabel('Displacement from Equilibrium');
    xlabel('Time (s)');
    legend('nonlin \Delta x', 'lin \Delta x', 'nonlin \Delta y', 'lin \Delta y', 'nonlin \Delta\theta', 'lin \Delta\theta');
    
    %large pertub plot
    epsilon_large = 1.0;
    V0_large = Veq + epsilon_large * perturbation_vec;
    
    %run both sims
    [t_nonlin_l, V_nonlin_l] = ode45(my_rate_func, tspan, V0_large);
    [t_lin_l, V_lin_l] = ode45(my_linear_rate, tspan, V0_large);
    
    figure();
    hold on;
    plot(t_nonlin_l, V_nonlin_l(:, 1) - Veq(1), 'r-');
    plot(t_lin_l, V_lin_l(:, 1) - Veq(1), 'r--');
    plot(t_nonlin_l, V_nonlin_l(:, 2) - Veq(2), 'b-');
    plot(t_lin_l, V_lin_l(:, 2) - Veq(2), 'b--');
    plot(t_nonlin_l, V_nonlin_l(:, 3) - Veq(3), 'k-');
    plot(t_lin_l, V_lin_l(:, 3) - Veq(3), 'k--');
    hold off;
    
    title('Linear vs. Nonlinear (Large Perturbation, \epsilon = 1.0)');
    ylabel('Displacement from Equilibrium');
    xlabel('Time (s)');
    legend('nonlin \Delta x', 'lin \Delta x', 'nonlin \Delta y', 'lin \Delta y', 'nonlin \Delta\theta', 'lin \Delta\theta');
    %%

    modal_analysis(Veq, A, my_rate_func);


    % initial condition
    x0 = Veq(1); y0 = Veq(2); theta0 = Veq(3) + 0.05;    
    
    vx0 = 0; vy0 = 0; vtheta0 = 0;

    V0 = [x0; y0; theta0; vx0; vy0; vtheta0];
    
    t0 = 0; tf = 12; dt = 0.01;
    tspan = t0:dt:tf;
    
    
    [tlist, Vlist] = ode45(my_rate_func, tspan, V0);

    Vlist = Vlist.';  
    tlist = tlist.';
    
    %animate_simulation(tlist, Vlist, box_params);

end