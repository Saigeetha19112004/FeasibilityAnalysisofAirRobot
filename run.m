function drone_simulation()
    % Drone Simulation Main Function
    
    % Simulation Parameters
    map_size = [100, 100];
    start_pos = [10, 10];
    goal_pos = [90, 90];
    
    % Initialize simulation variables
    current_pos = start_pos;
    
    % Generate static obstacles
    static_obstacles = generate_static_obstacles(map_size);
    
    % Simulation loop
    max_iterations = 200;
    for iteration = 1:max_iterations
        % Generate dynamic obstacles
        dynamic_obstacles = generate_dynamic_obstacles();
        
        % Find path using modified A* algorithm
        path = find_path(current_pos, goal_pos, map_size, static_obstacles, dynamic_obstacles);
        
        % Check if path is found
        if isempty(path)
            disp('No path found. Ending simulation.');
            break;
        end
        
        % Move along the path
        if size(path, 1) > 1
            current_pos = path(2, :);
            path(1, :) = [];
        end
        
        % Print current position
        fprintf('Iteration %d: Position [%d, %d]\n', iteration, current_pos(1), current_pos(2));
        
        % Check if goal is reached
        if isequal(current_pos, goal_pos)
            disp('Goal reached!');
            break;
        end
        
        % Randomly change goal with 10% probability
        if rand < 0.1
            goal_pos = randi([1, 100], [1, 2]);
            disp('Goal changed!');
        end
    end
    
    disp('Simulation completed.');
end

function obstacles = generate_static_obstacles(map_size)
    % Generate 50% static obstacles
    num_obstacles = round(0.5 * prod(map_size));
    obstacles = zeros(num_obstacles, 2);
    
    for i = 1:num_obstacles
        obstacles(i, :) = randi([1, 100], [1, 2]);
    end
end

function obstacles = generate_dynamic_obstacles()
    % Generate dynamic obstacles (birds)
    num_birds = randi([5, 15]);
    obstacles = zeros(num_birds, 2);
    
    for i = 1:num_birds
        obstacles(i, :) = randi([1, 100], [1, 2]);
    end
end

function path = find_path(start, goal, map_size, static_obstacles, dynamic_obstacles)
    % Simple path finding algorithm
    path = [];
    current = start;
    
    while ~isequal(current, goal)
        % Determine direction to move
        dx = sign(goal(1) - current(1));
        dy = sign(goal(2) - current(2));
        
        % Potential next positions
        candidates = [
            current(1) + dx, current(2);
            current(1), current(2) + dy;
            current(1) + dx, current(2) + dy
        ];
        
        % Find valid move
        valid_move = [];
        for i = 1:size(candidates, 1)
            if is_valid_move(candidates(i, :), map_size, static_obstacles, dynamic_obstacles)
                valid_move = candidates(i, :);
                break;
            end
        end
        
        % If no valid move, break
        if isempty(valid_move)
            break;
        end
        
        % Update path and current position
        path = [path; valid_move];
        current = valid_move;
        
        % Prevent infinite loop
        if size(path, 1) > 100
            break;
        end
    end
end

function valid = is_valid_move(pos, map_size, static_obstacles, dynamic_obstacles)
    % Check map boundaries
    if pos(1) < 1 || pos(1) > map_size(1) || ...
       pos(2) < 1 || pos(2) > map_size(2)
        valid = false;
        return;
    end
    
    % Check static obstacles
    if any(ismember(static_obstacles, pos, 'rows'))
        valid = false;
        return;
    end
    
    % Check dynamic obstacles
    if any(ismember(dynamic_obstacles, pos, 'rows'))
        valid = false;
        return;
    end
    
    valid = true;
end

% Run the simulation
drone_simulation()