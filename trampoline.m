function trampoline()
    % Example parameters 
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
    
    f_no_t = @(V) box_rate_func(0, V, box_params);
    
    % Initial guess 
    Vguess = [0; -3; 0; 0; 0; 0];   

    % Solve f(V) = 0
    [Veq, fval, exitflag, output] = fsolve(f_no_t, Vguess);
    
    disp("Equilibrium state found:");
    disp(Veq);
    
    disp("f(Veq) = ");
    disp(f_no_t(Veq));

    % initial condition
    x0 = 0; y0 = 0; theta0 = 0.05;    
    vx0 = 0; vy0 = 0; vtheta0 = 0;
    V0 = [x0; y0; theta0; vx0; vy0; vtheta0];
    
    t0 = 0; tf = 12; dt = 0.02;
    tspan = t0:dt:tf;
    
    my_rate_func = @(t,V) box_rate_func(t,V,box_params);
    
    [tlist, Vlist] = ode45(my_rate_func, tspan, V0);

    Vlist = Vlist.';  
    tlist = tlist.';
    
    animate_simulation(tlist, Vlist, box_params);

end