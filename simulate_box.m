function [t_list, v_list] = simulate_box()

    box_params = create_params();

    my_rate_func = @(t_in,V_in) box_rate_func(t_in,V_in,box_params);

    x0 = 0;        % initial x position
    y0 = -1;       % initial y position
    theta0 = 0.1;  % initial angle
    vx0 = 0;       % initial x velocity
    vy0 = 0;       % initial y velocity
    vtheta0 = 0;   % initial angular velocity

    % Initial state vector
    V0 = [x0;y0;theta0;vx0;vy0;vtheta0];
    
    % Time span for the simulation (e.g., 0 to 20 seconds)
    tspan = 0:0.1:20;

    %run the integration
    [t_list,v_list] = ode45(my_rate_func,tspan,V0);

    v_list = v_list';

end