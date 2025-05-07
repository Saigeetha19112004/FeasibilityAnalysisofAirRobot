function drone_3d_simulation()
    % 3D Drone Simulation with Obstacle Avoidance
    
    % Simulation Parameters
    map_size = [100, 100, 50];  % 3D map dimensions (x, y, z)
    start_pos = [10, 10, 10];   % Starting position
    goal_pos = [90, 90, 40];    % Goal position
    
    % Simulation Setup
    rng('shuffle');  % Random seed for varied simulations
    
    % Generate 3D Obstacles
    [static_obstacles, dynamic_obstacles] = generate_3d_obstacles(map_size);
    
    % Visualization Setup
    figure;
    hold on;
    axis([0 map_size(1) 0 map_size(2) 0 map_size(3)]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Drone Path Simulation');
    grid on;
    
    % Plot Static Obstacles
    for i = 1:size(static_obstacles, 1)
        obstacle = static_obstacles(i, :);
        % Create cuboid obstacles
        cuboid_size = [5, 5, 5];  % Fixed obstacle size
        x = obstacle(1) - cuboid_size(1)/2;
        y = obstacle(2) - cuboid_size(2)/2;
        z = obstacle(3) - cuboid_size(3)/2;
        
        % Plot obstacle as a semi-transparent cuboid
        fill3([x x+cuboid_size(1) x+cuboid_size(1) x], ...
              [y y y y+cuboid_size(2)], ...
              [z z+cuboid_size(3) z z], 'r', 'FaceAlpha', 0.3);
        fill3([x x+cuboid_size(1) x+cuboid_size(1) x], ...
              [y+cuboid_size(2) y+cuboid_size(2) y+cuboid_size(2) y+cuboid_size(2)], ...
              [z z+cuboid_size(3) z z], 'r', 'FaceAlpha', 0.3);
    end
    
    % Path Planning
    path = a_star_3d(start_pos, goal_pos, map_size, static_obstacles, dynamic_obstacles);
    
    % Drone Movement and Visualization
    if ~isempty(path)
        % Plot drone path
        plot3(path(:,1), path(:,2), path(:,3), 'b-', 'LineWidth', 2);
        
        % Animate drone movement
        drone = plot3(start_pos(1), start_pos(2), start_pos(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
        
        % Move drone along path
        for i = 2:size(path, 1)
            set(drone, 'XData', path(i,1), 'YData', path(i,2), 'ZData', path(i,3));
            drawnow;
            pause(0.1);  % Control animation speed
        end
        
        % Mark goal
        plot3(goal_pos(1), goal_pos(2), goal_pos(3), 'r*', 'MarkerSize', 10);
    else
        disp('No path found!');
    end
    
    hold off;
end

function [static_obstacles, dynamic_obstacles] = generate_3d_obstacles(map_size)
    % Generate 3D static and dynamic obstacles
    
    % Static Obstacles (50% of map volume)
    num_static = round(0.5 * prod(map_size) / 1000);
    static_obstacles = [
        randi([1, map_size(1)], num_static, 1), ...
        randi([1, map_size(2)], num_static, 1), ...
        randi([1, map_size(3)], num_static, 1)
    ];
    
    % Dynamic Obstacles (moving objects like birds)
    num_dynamic = randi([5, 15]);
    dynamic_obstacles = [
        randi([1, map_size(1)], num_dynamic, 1), ...
        randi([1, map_size(2)], num_dynamic, 1), ...
        randi([1, map_size(3)], num_dynamic, 1)
    ];
end

function path = a_star_3d(start, goal, map_size, static_obstacles, dynamic_obstacles)
    % 3D A* Path Finding Algorithm
    
    % Heuristic function (Euclidean distance)
    heuristic = @(a, b) sqrt(sum((a-b).^2));
    
    % Possible movements (26 directions in 3D)
    movements = [
        1 0 0; -1 0 0;
        0 1 0; 0 -1 0;
        0 0 1; 0 0 -1;
        1 1 0; 1 -1 0; -1 1 0; -1 -1 0;
        1 0 1; 1 0 -1; -1 0 1; -1 0 -1;
        0 1 1; 0 1 -1; 0 -1 1; 0 -1 -1;
        1 1 1; 1 1 -1; 1 -1 1; 1 -1 -1;
        -1 1 1; -1 1 -1; -1 -1 1; -1 -1 -1
    ];
    
    % Initialize open and closed lists
    open_list = [];
    closed_list = [];
    
    % Create start node
    start_node = struct('pos', start, 'g', 0, 'h', heuristic(start, goal), 'f', 0, 'parent', []);
    start_node.f = start_node.g + start_node.h;
    open_list = [open_list, start_node];
    
    max_iterations = 1000;
    iterations = 0;
    
    while ~isempty(open_list) && iterations < max_iterations
        iterations = iterations + 1;
        
        % Find node with lowest f value
        [~, idx] = min([open_list.f]);
        current = open_list(idx);
        
        % Remove current from open list
        open_list(idx) = [];
        
        % Add to closed list
        closed_list = [closed_list, current];
        
        % Check if goal reached
        if all(current.pos == goal)
            path = reconstruct_path(current);
            return;
        end
        
        % Explore neighbors
        for i = 1:size(movements, 1)
            neighbor_pos = current.pos + movements(i, :);
            
            % Check if neighbor is valid
            if is_valid_position(neighbor_pos, map_size, static_obstacles, dynamic_obstacles)
                % Calculate costs
                g_cost = current.g + 1;
                h_cost = heuristic(neighbor_pos, goal);
                f_cost = g_cost + h_cost;
                
                % Check if neighbor is already in closed list
                if any(arrayfun(@(x) all(x.pos == neighbor_pos), closed_list))
                    continue;
                end
                
                % Check if neighbor is in open list
                existing = find(arrayfun(@(x) all(x.pos == neighbor_pos), open_list));
                
                if isempty(existing)
                    % Add new node to open list
                    new_node = struct('pos', neighbor_pos, 'g', g_cost, 'h', h_cost, 'f', f_cost, 'parent', current);
                    open_list = [open_list, new_node];
                else
                    % Update existing node if new path is better
                    if g_cost < open_list(existing).g
                        open_list(existing).g = g_cost;
                        open_list(existing).f = f_cost;
                        open_list(existing).parent = current;
                    end
                end
            end
        end
    end
    
    % No path found
    path = [];
end

function valid = is_valid_position(pos, map_size, static_obstacles, dynamic_obstacles)
    % Check if position is within map bounds
    if any(pos < 1) || pos(1) > map_size(1) || ...
       pos(2) > map_size(2) || pos(3) > map_size(3)
        valid = false;
        return;
    end
    
    % Check against static obstacles
    if any(ismember(static_obstacles, pos, 'rows'))
        valid = false;
        return;
    end
    
    % Check against dynamic obstacles
    if any(ismember(dynamic_obstacles, pos, 'rows'))
        valid = false;
        return;
    end
    
    valid = true;
end

function path = reconstruct_path(goal_node)
    % Reconstruct path from goal to start
    path = [];
    current = goal_node;
    
    while ~isempty(current)
        path = [current.pos; path];
        current = current.parent;
    end
end

% Run the simulation
drone_3d_simulation()