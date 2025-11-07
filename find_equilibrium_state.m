function find_equilibrium_state()
  
    box_params = create_params();

    my_rate_func = @(V_in) box_rate_func(0,V_in,box_params);


    x0 = 0;        % initial x position
    y0 = -1;       % initial y position
    theta0 = 0.1;  % initial angle
    vx0 = 0;       % initial x velocity
    vy0 = 0;       % initial y velocity
    vtheta0 = 0;   % initial angular velocity

    % Initial state vector
    V_guess = [x0;y0;theta0;vx0;vy0;vtheta0];
   
    [V_eq,~,~] = fsolve(my_rate_func,V_guess);

    my_rate_func(V_eq)
    
    % accel_root_func = @(P) compute_accel(P(1), P(2), P(3), box_params);

    % x_guess = 0;
    % y_guess = -1;
    % theta_guess = 0;
    % P_guess = [x_guess; y_guess; theta_guess];
    
        
    % [P_eq, fval, exitflag] = fsolve(accel_root_func, P_guess);

    % V_eq = [P_eq(1); P_eq(2); P_eq(3); 0; 0; 0];

    %validate
    % tspan_verify = [0, 10];

    % [t_list_verify, V_list_verify] = ode45(my_rate_func, tspan_verify, V_eq);

    % final_state = V_list_verify(end, :)';
    % state_change_norm = norm(final_state - V_eq);
    
    % fprintf('Initial state (V_eq): \n');
    % disp(V_eq');
    % fprintf('Final state at t=%g: \n', tspan_verify(end));
    % disp(final_state');
    % fprintf('Norm of state change: %e\n', state_change_norm);
        
end