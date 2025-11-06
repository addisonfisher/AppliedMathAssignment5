%Computes the linear and angular acceleration of the box
%given its current position and orientation

%INPUTS:
%x: current x position of the box
%y: current y position of the box
%theta: current orientation of the box
%box_params: a struct containing the parameters that describe the system
%Fields:
%box_params.m: mass of the box
%box_params.I: moment of inertia w/respect to centroid
%box_params.g: acceleration due to gravity
%box_params.k_list: list of spring stiffnesses
%box_params.l0_list: list of spring natural lengths
%box_params.P_world: 2 x n list of static mounting
% points for the spring (in the world frame)
%box_params.P_box: 2 x n list of mounting points
% for the spring (in the box frame)

%OUTPUTS
%ax: x acceleration of the box
%ay: y acceleration of the box
%atheta: angular acceleration of the box
function [ax,ay,atheta] = compute_accel(x,y,theta,box_params)
    
    m = box_params.m;
    I = box_params.I;
    g = box_params.g;
    k_list = box_params.k_list;
    l0_list = box_params.l0_list;
    P_world_list = box_params.P_world;
    P_box_list = box_params.P_box;

    num_springs = length(k_list);

    F_net = [0; -m * g];
    tau_net = 0;

    %computing rigid body tranformation
    P_box_world = compute_rbt(x, y, theta, P_box_list);

    %iterate through each spring
    for i=1:num_springs

        k = k_list(i);
        l0 = l0_list(i);

        %get positions of spring
        PA = P_world_list(:, i);
        PB = P_box_world(:, i);

        %compute force from springs
        F_i = compute_spring_force(k, l0, PA, PB);

        %add to net force and torque
        F_net = F_net + F_i;
        r_i = PB - [x; y];

        tau_i = r_i(1) * F_i(2) - r_i(2) * F_i(1);
        tau_net = tau_net + tau_i;

    end
    %calc acceleration
    ax = F_net(1) / m;
    ay = F_net(2) / m;

    atheta = tau_net / I;

end