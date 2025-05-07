function simulate_drone_with_a_star()
    % Initialize 3D simulation environment
    fig = figure('Name', '3D Drone Simulation with Dynamic Obstacles', 'NumberTitle', 'off');
    ax = axes('XLim', [-50 50], 'YLim', [-50 50], 'ZLim', [0 100]);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('3D Drone Simulation with Dynamic Obstacles and A* Path Planning');
    grid on; hold on;
    view(3);

    % Define drone's starting position and destination
    start_position = [-40, -40, 20]; % Start at altitude 20
    destination = [40, 40, 20]; % End at same altitude
    drone_pos = start_position;

    % Define the number of dynamic obstacles (spheres)
    num_obstacles = 8;

    % Generate random obstacle positions
    [obstacle_positions, obstacle_radii] = generate_random_obstacles(start_position, destination, num_obstacles);

    % Draw obstacles as red spheres
    obstacle_handles = gobjects(num_obstacles, 1);
    for i = 1:num_obstacles
        obstacle_handles(i) = drawSphere(obstacle_positions(i, :), obstacle_radii(i), 'r');
    end

    % Draw drone as a blue sphere
    drone_radius = 2;
    drone = drawSphere(drone_pos, drone_radius, 'b');

    % Calculate initial path using A* algorithm
    disp("Calculating initial path using A* algorithm...");
    path = a_star_algorithm(drone_pos, destination, obstacle_positions, obstacle_radii, drone_radius);

    % Check if a valid path was found
    if isempty(path)
        disp("No valid path found. Drone cannot reach the destination.");
        return;
    end

    % Plot the calculated path
    path_plot = plot3(path(:, 1), path(:, 2), path(:, 3), 'g--', 'LineWidth', 2);

    % Simulate drone movement along the path
    disp("Drone starting movement...");
    path_index = 1;
    
    while path_index <= size(path, 1)
        % Update dynamic obstacles to move toward the drone
        obstacle_positions = update_dynamic_obstacles(obstacle_positions, drone_pos);
        
        % Update obstacle visualization
        for j = 1:num_obstacles
            delete(obstacle_handles(j));
            obstacle_handles(j) = drawSphere(obstacle_positions(j, :), obstacle_radii(j), 'r');
        end

        % Update drone position
        drone_pos = path(path_index, :);
        delete(drone);
        drone = drawSphere(drone_pos, drone_radius, 'b');
        plot3(drone_pos(1), drone_pos(2), drone_pos(3), '.', 'Color', 'b', 'MarkerSize', 10);
        
        % Check for collisions and recalculate path if necessary
        if check_collision(drone_pos, obstacle_positions, obstacle_radii, drone_radius)
            disp("Collision detected! Recalculating path...");
            path = a_star_algorithm(drone_pos, destination, obstacle_positions, obstacle_radii, drone_radius);
            if isempty(path)
                disp("No valid path found. Drone cannot reach the destination.");
                return;
            end
            % Plot the new path
            delete(path_plot);
            path_plot = plot3(path(:, 1), path(:, 2), path(:, 3), 'g--', 'LineWidth', 2);
            path_index = 1; % Reset path index
        else
            path_index = path_index + 1;
        end
        
        pause(0.1);
        drawnow;
    end
    
    disp("Drone has reached the destination.");
end

function obstacle_positions = update_dynamic_obstacles(obstacle_positions, drone_pos)
    % Move all obstacles toward the drone with some randomness
    for i = 1:size(obstacle_positions, 1)
        % Calculate direction vector from obstacle to drone
        direction = drone_pos - obstacle_positions(i, :);
        direction = direction / (norm(direction) + eps); % Normalize direction vector
        
        % Add some randomness to movement
        random_factor = 0.3 * randn(1, 3);
        
        % Move obstacle toward the drone with some randomness
        obstacle_positions(i, :) = obstacle_positions(i, :) + 1.5 * direction + random_factor;
        
        % Keep obstacles within bounds
        obstacle_positions(i, 1) = max(min(obstacle_positions(i, 1), 50), -50);
        obstacle_positions(i, 2) = max(min(obstacle_positions(i, 2), 50), -50);
        obstacle_positions(i, 3) = max(min(obstacle_positions(i, 3), 80), 5); % Keep between 5 and 80 altitude
    end
end

function [obstacle_positions, obstacle_radii] = generate_random_obstacles(start, goal, num_obstacles)
    grid_limits = [-50, 50; -50, 50; 5, 80]; % X, Y, Z limits
    obstacle_positions = zeros(num_obstacles, 3);
    obstacle_radii = 2 + rand(num_obstacles, 1) * 3; % Radii between 2 and 5

    for i = 1:num_obstacles
        while true
            % Generate random position
            obstacle = [randi(grid_limits(1, :)), randi(grid_limits(2, :)), randi(grid_limits(3, :))];
            
            % Ensure the obstacle doesn't overlap with start or goal positions
            start_dist = norm(obstacle - start);
            goal_dist = norm(obstacle - goal);
            
            if start_dist > 10 && goal_dist > 10
                obstacle_positions(i, :) = obstacle;
                break;
            end
        end
    end
end

function path = a_star_algorithm(start, goal, obstacles, obstacle_radii, drone_radius)
    grid_size = 100;
    resolution = 2;
    
    % Add parent NaN to the start
    open_list = [start, 0, heuristic(start, goal), NaN, NaN, NaN];
    closed_list = [];

    while ~isempty(open_list)
        [~, idx] = min(open_list(:, 4) + open_list(:, 5));
        current_node = open_list(idx, :);

        if norm(current_node(1:3) - goal) < 5 % Close enough to goal
            path = reconstruct_path(current_node, closed_list);
            return;
        end

        open_list(idx, :) = [];
        closed_list = [closed_list; current_node];

        neighbors = generate_neighbors(current_node, grid_size, resolution);
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            
            if ismember(round(neighbor), round(closed_list(:, 1:3)), 'rows')
                continue;
            end
            
            if check_collision(neighbor, obstacles, obstacle_radii, drone_radius)
                continue;
            end

            tentative_g = current_node(4) + norm(neighbor - current_node(1:3));
            idx_open = find(ismember(round(open_list(:, 1:3)), round(neighbor), 'rows'));

            if isempty(idx_open)
                h = heuristic(neighbor, goal);
                open_list = [open_list; neighbor, tentative_g, h, current_node(1:3)];
            elseif tentative_g < open_list(idx_open, 4)
                open_list(idx_open, 4) = tentative_g;
                open_list(idx_open, 6:8) = current_node(1:3);
            end
        end
    end
    path = [];
end

function h = heuristic(node, goal)
    h = norm(node - goal);
end

function neighbors = generate_neighbors(node, grid_size, resolution)
    [X, Y, Z] = meshgrid(-1:1, -1:1, -1:1);
    offsets = [X(:), Y(:), Z(:)];
    offsets(ismember(offsets, [0, 0, 0], 'rows'), :) = [];
    neighbors = node(1:3) + offsets * resolution;
    
    % Keep within bounds
    in_bounds = all(neighbors >= [-50, -50, 0], 2) & all(neighbors <= [50, 50, 100], 2);
    neighbors = neighbors(in_bounds, :);
end

function collision = check_collision(position, obstacles, obstacle_radii, drone_radius)
    distances = vecnorm(obstacles - position, 2, 2);
    collision = any(distances < (obstacle_radii + drone_radius));
end

function path = reconstruct_path(node, closed_list)
    path = node(1:3);
    while any(~isnan(node(6:8)))
        idx = find(ismember(round(closed_list(:, 1:3)), round(node(6:8)), 'rows'));
        if isempty(idx)
            break;
        end
        node = closed_list(idx, :);
        path = [node(1:3); path];
    end
end

function h = drawSphere(center, radius, color)
    [X, Y, Z] = sphere(20);
    X = X * radius + center(1);
    Y = Y * radius + center(2);
    Z = Z * radius + center(3);
    h = surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'none');
end