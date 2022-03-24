% Ashley Kim, CAAM 210, Spring 2022, Project #6
% bridge.m
% The purpose of this project is to create a visual diagram of a bridge and
% how this bridge would be impacted by the weight of a simulated car on the
% bridge
% Last Modified: March 23, 2022
function bridge
    build_load_plot_basic_bridge(1)
    build_load_plot_basic_bridge(2)
    build_load_plot_basic_bridge(3)
end
% driver function that calls the plots of the basic bridges to show
% deformities and adjacency matrices
function [adj, xc, yc, len] = build_basic_bridge(nos)
    % calculate some helpful numbers
    num_nodes = 2 * nos + 2;
    num_edges = 5 * nos + 5;
    s = 1/sqrt(2);
    % initialize the return values
    adj = zeros(num_edges, 2 * num_nodes);
    xc = zeros(num_edges, 2);
    yc = zeros(num_edges, 2);
    len = ones(num_edges, 1);
    % build the left side of bridge
    adj(1, 1) = 1;
    adj(2, [3, 4]) = [s s];
    xc(1, :) = [0 1];
    xc(2, :) = [0 1];
    yc(1, :) = [0 0];
    yc(2, :) = [0 1];
    len(2) = 1/s;
    % build the middle of bridge
    for i = 1:nos
        start_row = i*5;
        start_column = i*4;
        % Fiber 3
        adj(start_row - 2, start_column - 2) = -1;
        adj(start_row - 2, start_column) = 1;
        xc(start_row -2, :) = [i, i];
        yc(start_row -2, :) = [0, 1];
        % Fiber 4
        adj(start_row - 1, [start_column-1:start_column+2]) = [-s,s,s,-s];
        xc(start_row - 1,:) = [i, i + 1];
        yc(start_row - 1,:) = [1,0];
        len(start_row - 1) = 1/s;
        % Fiber 5
        adj(start_row, start_column - 1) = -1;
        xc(start_row,:) = [i, i + 1];
        yc(start_row,:) = [1, 1];
        adj(start_row, start_column + 3) = 1;
        % Fiber 6
        adj(start_row + 1,[start_column - 3:start_column - 2]) = [-s, -s];
        adj(start_row + 1,[start_column + 3:start_column + 4]) = [s, s];
        xc(start_row + 1,:) = [i, i + 1];
        yc(start_row + 1,:) = [0, 1];
        len(start_row + 1) = 1/s;
        % Fiber 7
        adj(start_row + 2, start_column - 3) = -1;
        adj(start_row + 2, start_column + 1) = 1;
        xc(start_row + 2,:) = [i, i + 1];
        yc(start_row + 2,:) = [0,0];
    end
    % build the right side of bridge
    adj(end - 2, end - 2) = -1;
    adj(end - 2, end) = 1;
    adj(end - 1, [end - 1, end]) = [-s,s];
    adj(end,end - 3) = -1;
    xc(end - 2,:) = [nos + 1,nos + 1];
    yc(end - 2,:) = [0, 1];
    xc(end - 1,:) = [nos + 1, nos + 2];
    yc(end - 1,:) = [1, 0];
    xc(end,:) = [nos + 1, nos + 2];
    yc(end,:) = [0,0];
    len(end - 1) = 1/s;
end
function [dx,dy,work,X,Y] = deform_basic_bridge(nos,adj,xc,yc,len,weight)
    % calculate some helpful numbers
    stiffness = adj' * diag(1./len) * adj;
    force = zeros(4 * nos + 4, 1);
    force(2:4:end) = -weight;
    displacements = stiffness\force; 
    work = displacements' * force;
    X = displacements(1:2:end);
    Y = displacements(2:2:end);
    % initialize the return values
    dx = zeros(size(xc));
    dy = zeros(size(yc));
    % deform the left side of bridge
    dx(1,:) = xc(1,:) + [0 X(1)];
    dx(2,:) = xc(2,:) + [0 X(2)];
    dy(1,:) = yc(1,:) + [0 Y(1)];
    dy(2,:) = yc(2,:) + [0 Y(2)];
    % deform the middle of bridge
    for i = 1:nos
        c= 2 * i;
        r = 5 * i;
        dx(r - 2,:) = xc(r - 2,:) + [X(c - 1) X(c)];
        dy(r - 2,:) = yc(r - 2,:) + [Y(c - 1) Y(c)];
        dx(r - 1,:) = xc(r - 1,:) + [X(c) X(c + 1)];
        dy(r - 1,:) = yc(r - 1,:) + [Y(c) Y(c + 1)];
        dx(r + 1,:) = xc(r + 1,:) + [X(c - 1) X(c + 2)];
        dy(r + 1,:) = yc(r + 1,:) + [Y(c - 1) Y(c + 2)];
        dx(r + 2,:) = xc(r + 2,:) + [X(c - 1) X(c + 1)];
        dy(r + 2,:) = yc(r + 2,:) + [Y(c - 1) Y(c + 1)];
        dx(r,:) = xc(r,:) + [X(c) X(c + 2)];
        dy(r,:) = yc(r,:) + [Y(c) Y(c + 2)];
    end
    % deform the right side of bridge
    dx(end - 2,:) = xc(end - 2,:) + [X(end - 1) X(end)];
    dy(end - 2,:) = yc(end - 2,:) + [Y(end - 1) Y(end)];
    dx(end - 1,:) = xc(end - 1,:) + [X(end) 0];
    dy(end - 1,:) = yc(end - 1,:) + [Y(end) 0];
    dx(end,:) = xc(end,:) + [X(end - 1) 0];
    dy(end,:) = yc(end,:) + [Y(end - 1) 0];
    % return 
end
% Function: plotbridge
% Inputs: nos (number of middle sections), xc (x-coordinatse for fibers),
% yc (y-coordinates for fibers), weight (measure of force), work (scalar
% measurement of bridge deformation), X & Y (helper matrices of displacement in
% horizontal and vertical directions)
% Outputs: None
function plotbridge(nos,xc,yc,weight,work)
% Creates new figure and plots the bridge
    figure
    x = '';
    x = strcat(x,num2str(nos)," Section Bridge, Weight of Car = ", num2str(weight),", Work = ",num2str(work));
    title(x)
    line(xc',yc','color','r')
    hold on
    fill([nos + 2, nos + 1, nos + 3, nos + 3],[0,-1,-1,0],'b')
    fill([0,-1,-1,1],[0,0,-1,-1],'b')
    hold off
end
% Function: plotspy
% Inputs: adj (adjacency matrix), nos (number of middle sections)
% Outputs: None
function plotspy(adj,nos)
    % Produces new figure and plots adjacency matrix
    figure
    x = '';
    x = strcat(x,num2str(nos)," Section bridge adjacency matrix");
    title(x)
    xlabel("Nonzero Matrix Columns")
    ylabel("Nonzero Matrix Rows")
    spy(adj)
end
% Function: build_load_plot_basic_bridge
% Inputs: nos (number of middle sections)
% Outputs: None
function build_load_plot_basic_bridge(nos)
    % Calls buildbasicbridge function and plots nondeformed bridge and
    % adjacency matrix
    [adj,xc,yc,len] = build_basic_bridge(nos);
    plotspy(adj,nos);
    plotbridge(nos,xc,yc,0,0);
    % Calls deform_basic_bridge and plots deformed bridges with hacing weight as
    % 0.01 and 0.05
    [dx,dy,work] = deform_basic_bridge(nos,adj,xc,yc,len,0.01);
    plotbridge(nos,dx,dy,0.01,work);
    [dx,dy,work] = deform_basic_bridge(nos,adj,xc,yc,len,0.05);
    plotbridge(nos,dx,dy,0.05,work);
end





