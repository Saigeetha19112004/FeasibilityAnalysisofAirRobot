classdef DroneSimulation3D
    properties
        % Simulation parameters
        worldSize = [100 100 50]; % [x y z] dimensions
        startPos = [5 5 10];      % Starting position [x y z]
        goalPos = [95 95 40];     % Goal position [x y z]
        droneSpeed = 2;           % Drone speed (units per iteration)
        
        % Obstacle parameters
        staticObstacles;          % Array of static obstacles
        numStaticObs = 25;        % Number of static obstacles (50% of total)
        dynamicObstacles;         % Array of dynamic obstacles (birds)
        numDynamicObs = 25;       % Number of dynamic obstacles (birds)
        birdSpeedRange = [1 3];   % Range of bird speeds
        
        % Path planning
        gridResolution = 1;       % Grid resolution for A*
        grid;                    % Occupancy grid
        path;                     % Current path
        currentPos;               % Current drone position
        
        % Weather parameters
        goodWeather = true;       % Default to good weather
        weatherAPIKey = 'your_openweather_api_key'; % Replace with your API key
        weatherCity = 'London';    % City for weather data
        
        % Visualization
        fig;
        dronePlot;
        goalPlot;
        staticObsPlots = [];
        dynamicObsPlots = [];
        pathPlot;
    end
    
    methods
        function obj = DroneSimulation3D()
            % Initialize the simulation
            obj = obj.initializeWorld();
            obj = obj.getWeatherData();
            obj = obj.createOccupancyGrid();
            obj = obj.findInitialPath();
            obj.visualize();
        end
        
        function obj = initializeWorld(obj)
            % Create static obstacles (buildings)
            obj.staticObstacles = repmat(struct('position',[],'size',[]), obj.numStaticObs, 1);
            for i = 1:obj.numStaticObs
                % Random position and size for buildings
                pos = rand(1,3) .* obj.worldSize;
                size = [5+rand*10, 5+rand*10, 5+rand*20]; % Random building dimensions
                obj.staticObstacles(i).position = pos;
                obj.staticObstacles(i).size = size;
            end
            
            % Create dynamic obstacles (birds)
            obj.dynamicObstacles = repmat(struct('position',[],'velocity',[]), obj.numDynamicObs, 1);
            for i = 1:obj.numDynamicObs
                % Random initial position for birds
                pos = rand(1,3) .* obj.worldSize;
                
                % Random velocity for birds
                speed = obj.birdSpeedRange(1) + rand * (obj.birdSpeedRange(2)-obj.birdSpeedRange(1));
                angleXY = rand * 2 * pi;
                angleZ = rand * pi - pi/2; % -45 to 45 degrees elevation
                
                vel = [speed*cos(angleXY)*cos(angleZ), ...
                       speed*sin(angleXY)*cos(angleZ), ...
                       speed*sin(angleZ)];
                
                obj.dynamicObstacles(i).position = pos;
                obj.dynamicObstacles(i).velocity = vel;
            end
            
            obj.currentPos = obj.startPos;
        end
        
        function obj = getWeatherData(obj)
            % Try to get weather data from OpenWeather API
            try
                % Create the API URL
                url = sprintf('http://api.openweathermap.org/data/2.5/weather?q=%s&appid=%s', ...
                             obj.weatherCity, obj.weatherAPIKey);
                
                % Get the data
                data = webread(url);
                
                % Check weather condition (simplified - good weather if no rain/snow)
                weatherCondition = data.weather(1).main;
                if contains(weatherCondition, {'Rain', 'Snow', 'Thunderstorm', 'Extreme'})
                    obj.goodWeather = false;
                    disp('Bad weather detected - flight mode may be affected');
                else
                    obj.goodWeather = true;
                    disp('Good weather - normal flight mode');
                end
            catch
                warning('Could not fetch weather data. Using default good weather.');
                obj.goodWeather = true;
            end
        end
        
        function obj = createOccupancyGrid(obj)
            % Create a 3D occupancy grid
            gridSize = ceil(obj.worldSize / obj.gridResolution);
            obj.grid = zeros(gridSize);
            
            % Mark static obstacles in the grid
            for i = 1:obj.numStaticObs
                obs = obj.staticObstacles(i);
                minCorner = floor(obs.position / obj.gridResolution);
                maxCorner = min(floor((obs.position + obs.size) / obj.gridResolution), gridSize);
                
                % Ensure we don't go out of bounds
                maxCorner = min(maxCorner, gridSize);
                minCorner = max(minCorner, [1 1 1]);
                
                % Mark occupied cells
                obj.grid(minCorner(1):maxCorner(1), ...
                        minCorner(2):maxCorner(2), ...
                        minCorner(3):maxCorner(3)) = 1;
            end
            
            % Adjust grid based on weather
            if ~obj.goodWeather
                % In bad weather, reduce maximum altitude
                maxZ = floor(obj.worldSize(3)*0.6 / obj.gridResolution); % 60% of max height
                obj.grid(:,:,maxZ:end) = 1; % Mark higher altitudes as occupied
            end
        end
        
        function obj = findInitialPath(obj)
            % Find initial path using A* algorithm
            startNode = floor(obj.startPos / obj.gridResolution);
            goalNode = floor(obj.goalPos / obj.gridResolution);
            
            obj.path = obj.aStar(startNode, goalNode);
            
            if isempty(obj.path)
                error('No initial path found to goal!');
            end
            
            % Convert path back to world coordinates
            obj.path = obj.path * obj.gridResolution;
        end
        
        function path = aStar(obj, startNode, goalNode)
            % A* pathfinding algorithm implementation
            
            % Create open and closed lists
            openList = PriorityQueue();
            closedList = false(size(obj.grid));
            
            % Initialize start node
            startNode = startNode(:)'; % Ensure row vector
            goalNode = goalNode(:)';   % Ensure row vector
            
            % Node info: [x, y, z, g, h, f, parentX, parentY, parentZ]
            startInfo = [startNode, 0, obj.heuristic(startNode, goalNode), ...
                        obj.heuristic(startNode, goalNode), NaN, NaN, NaN];
            
            openList.insert(startInfo, startInfo(6)); % Key is f-score
            
            % Directions for 26-connected 3D grid
            directions = [];
            for dx = -1:1
                for dy = -1:1
                    for dz = -1:1
                        if ~(dx == 0 && dy == 0 && dz == 0)
                            directions = [directions; dx dy dz];
                        end
                    end
                end
            end
            
            % Main A* loop
            while ~openList.isEmpty()
                % Get node with lowest f-score
                current = openList.extractMin();
                currentPos = current(1:3);
                
                % Check if we've reached the goal
                if all(currentPos == goalNode)
                    % Reconstruct path
                    path = obj.reconstructPath(current, closedList);
                    return;
                end
                
                % Add to closed list
                closedList(currentPos(1), currentPos(2), currentPos(3)) = true;
                
                % Explore neighbors
                for i = 1:size(directions, 1)
                    neighborPos = currentPos + directions(i,:);
                    
                    % Check bounds
                    if any(neighborPos < 1) || any(neighborPos > size(obj.grid))
                        continue;
                    end
                    
                    % Check if obstacle or in closed list
                    if obj.grid(neighborPos(1), neighborPos(2), neighborPos(3)) || ...
                       closedList(neighborPos(1), neighborPos(2), neighborPos(3))
                        continue;
                    end
                    
                    % Calculate tentative g-score
                    stepCost = norm(directions(i,:)); % Euclidean distance for cost
                    tentativeG = current(4) + stepCost;
                    
                    % Check if neighbor is in open list
                    [inOpen, existingNode] = openList.find(neighborPos);
                    
                    if ~inOpen || tentativeG < existingNode(4)
                        % Calculate heuristic and f-score
                        h = obj.heuristic(neighborPos, goalNode);
                        f = tentativeG + h;
                        
                        % Create neighbor info
                        neighborInfo = [neighborPos, tentativeG, h, f, currentPos];
                        
                        if ~inOpen
                            openList.insert(neighborInfo, f);
                        else
                            openList.decreaseKey(existingNode, neighborInfo, f);
                        end
                    end
                end
            end
            
            % No path found
            path = [];
        end
        
        function h = heuristic(~, pos, goal)
            % Euclidean distance heuristic
            h = norm(pos - goal);
        end
        
        function path = reconstructPath(~, goalNode, closedList)
            % Reconstruct path from goal to start
            path = [];
            current = goalNode;
            
            while ~any(isnan(current(5:7)))
                path = [current(1:3); path];
                parentPos = current(5:7);
                
                % Find parent in closed list (this is simplified - in real implementation
                % you'd need to track parents properly)
                % For this demo, we'll just follow the parent pointers
                current(1:3) = parentPos;
            end
            
            % Add start node
            path = [current(1:3); path];
        end
        
        function obj = updateDynamicObstacles(obj)
            % Update positions of dynamic obstacles (birds)
            for i = 1:obj.numDynamicObs
                % Update position
                obj.dynamicObstacles(i).position = obj.dynamicObstacles(i).position + ...
                                                  obj.dynamicObstacles(i).velocity;
                
                % Check bounds and bounce if needed
                pos = obj.dynamicObstacles(i).position;
                vel = obj.dynamicObstacles(i).velocity;
                
                for dim = 1:3
                    if pos(dim) < 0 || pos(dim) > obj.worldSize(dim)
                        vel(dim) = -vel(dim); % Reverse direction
                        
                        % Add some randomness to direction change
                        if dim == 3 % Z-axis
                            vel(1:2) = vel(1:2) .* (0.8 + 0.4*rand(1,2));
                        else
                            vel(3) = vel(3) * (0.8 + 0.4*rand);
                        end
                        
                        obj.dynamicObstacles(i).velocity = vel;
                        obj.dynamicObstacles(i).position(dim) = ...
                            max(0, min(pos(dim), obj.worldSize(dim)));
                    end
                end
                
                % Occasionally change direction randomly
                if rand < 0.05 % 5% chance per iteration
                    speed = norm(vel);
                    angleXY = rand * 2 * pi;
                    angleZ = rand * pi - pi/2;
                    obj.dynamicObstacles(i).velocity = [speed*cos(angleXY)*cos(angleZ), ...
                                                       speed*sin(angleXY)*cos(angleZ), ...
                                                       speed*sin(angleZ)];
                end
            end
        end
        
        function obj = checkForCollisions(obj)
            % Check if dynamic obstacles are too close to the drone's path
            
            % Check each segment of the path
            for i = 1:size(obj.path,1)-1
                segmentStart = obj.path(i,:);
                segmentEnd = obj.path(i+1,:);
                
                % Check each dynamic obstacle
                for j = 1:obj.numDynamicObs
                    birdPos = obj.dynamicObstacles(j).position;
                    birdVel = obj.dynamicObstacles(j).velocity;
                    birdRadius = 2; % Approximate bird size
                    
                    % Find closest point on segment to bird
                    [closestPt, dist] = obj.pointToSegmentDistance(birdPos, segmentStart, segmentEnd);
                    
                    % Check if bird is too close to the path
                    if dist < birdRadius + 3 % 3 units safety margin
                        % Check if bird is moving toward the path
                        toPath = closestPt - birdPos;
                        if norm(toPath) > 0
                            toPath = toPath / norm(toPath);
                            approachAngle = acosd(dot(birdVel/norm(birdVel), toPath));
                            
                            if approachAngle < 90 % Bird is moving toward path
                                disp('Dynamic obstacle detected near path! Replanning...');
                                obj = obj.replanPath();
                                return;
                            end
                        end
                    end
                end
            end
            
            % Also check current drone position against all obstacles
            droneRadius = 2;
            for i = 1:obj.numStaticObs
                obs = obj.staticObstacles(i);
                if obj.pointInBox(obj.currentPos, obs.position, obs.size)
                    disp('Collision with static obstacle! Replanning...');
                    obj = obj.replanPath();
                    return;
                end
            end
            
            for i = 1:obj.numDynamicObs
                birdPos = obj.dynamicObstacles(i).position;
                if norm(obj.currentPos - birdPos) < droneRadius + 2 % bird radius = 2
                    disp('Collision with dynamic obstacle! Replanning...');
                    obj = obj.replanPath();
                    return;
                end
            end
        end
        
        function obj = replanPath(obj)
            % Replan path from current position to goal
            
            % Update occupancy grid with dynamic obstacles
            obj = obj.updateOccupancyWithDynamicObstacles();
            
            % Find new path using A*
            startNode = floor(obj.currentPos / obj.gridResolution);
            goalNode = floor(obj.goalPos / obj.gridResolution);
            
            newPath = obj.aStar(startNode, goalNode);
            
            if ~isempty(newPath)
                % Convert path back to world coordinates
                obj.path = newPath * obj.gridResolution;
                disp('New path found!');
            else
                warning('No new path found to goal! Trying to continue with current path.');
            end
        end
        
        function obj = updateOccupancyWithDynamicObstacles(obj)
            % Temporarily add dynamic obstacles to the grid
            
            % Reset grid to only static obstacles
            obj = obj.createOccupancyGrid();
            
            % Add dynamic obstacles as temporary obstacles
            for i = 1:obj.numDynamicObs
                birdPos = obj.dynamicObstacles(i).position;
                birdSize = [3 3 3]; % Approximate bird size in grid
                
                gridPos = floor(birdPos / obj.gridResolution);
                gridSize = ceil(birdSize / obj.gridResolution);
                
                minCorner = max(gridPos - gridSize/2, [1 1 1]);
                maxCorner = min(gridPos + gridSize/2, size(obj.grid));
                
                obj.grid(minCorner(1):maxCorner(1), ...
                       minCorner(2):maxCorner(2), ...
                       minCorner(3):maxCorner(3)) = 1;
            end
        end
        
        function [closestPt, dist] = pointToSegmentDistance(~, point, segStart, segEnd)
            % Find closest point on segment to the given point and the distance
            
            segVec = segEnd - segStart;
            segLength = norm(segVec);
            segDir = segVec / segLength;
            
            pointVec = point - segStart;
            proj = dot(pointVec, segDir);
            
            if proj < 0
                closestPt = segStart;
            elseif proj > segLength
                closestPt = segEnd;
            else
                closestPt = segStart + proj * segDir;
            end
            
            dist = norm(point - closestPt);
        end
        
        function inside = pointInBox(~, point, boxPos, boxSize)
            % Check if point is inside a box
            inside = all(point >= boxPos) && all(point <= boxPos + boxSize);
        end
        
        function obj = updateDronePosition(obj)
            % Move drone along the current path
            
            if isempty(obj.path)
                return;
            end
            
            % Find next waypoint (skip if too close)
            nextWaypoint = [];
            waypointIndex = 1;
            while waypointIndex <= size(obj.path,1)
                nextWaypoint = obj.path(waypointIndex,:);
                if norm(obj.currentPos - nextWaypoint) > obj.droneSpeed
                    break;
                end
                waypointIndex = waypointIndex + 1;
            end
            
            if isempty(nextWaypoint)
                disp('Reached goal!');
                return;
            end
            
            % Move toward next waypoint
            direction = nextWaypoint - obj.currentPos;
            distance = norm(direction);
            
            if distance > 0
                direction = direction / distance;
                moveDistance = min(obj.droneSpeed, distance);
                obj.currentPos = obj.currentPos + direction * moveDistance;
            end
            
            % If we're close to the waypoint, remove it from path
            if distance < obj.droneSpeed && waypointIndex <= size(obj.path,1)
                obj.path = obj.path(waypointIndex:end,:);
            end
        end
        
        function obj = visualize(obj)
            % Create or update visualization
            
            if isempty(obj.fig) || ~isvalid(obj.fig)
                % Create new figure
                obj.fig = figure('Name', '3D Drone Simulation', 'NumberTitle', 'off');
                hold on;
                grid on;
                axis equal;
                xlim([0 obj.worldSize(1)]);
                ylim([0 obj.worldSize(2)]);
                zlim([0 obj.worldSize(3)]);
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                view(3);
                
                % Plot static obstacles (buildings)
                for i = 1:obj.numStaticObs
                    obs = obj.staticObstacles(i);
                    [X,Y,Z] = obj.createBox(obs.position, obs.size);
                    obj.staticObsPlots(i) = surf(X, Y, Z, 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'none');
                    alpha(obj.staticObsPlots(i), 0.7);
                end
                
                % Plot dynamic obstacles (birds)
                for i = 1:obj.numDynamicObs
                    birdPos = obj.dynamicObstacles(i).position;
                    [X,Y,Z] = sphere(10);
                    X = X * 1 + birdPos(1);
                    Y = Y * 1 + birdPos(2);
                    Z = Z * 1 + birdPos(3);
                    obj.dynamicObsPlots(i) = surf(X, Y, Z, 'FaceColor', [1 0.5 0], 'EdgeColor', 'none');
                end
                
                % Plot drone
                [X,Y,Z] = sphere(10);
                X = X * 1.5 + obj.currentPos(1);
                Y = Y * 1.5 + obj.currentPos(2);
                Z = Z * 1.5 + obj.currentPos(3);
                obj.dronePlot = surf(X, Y, Z, 'FaceColor', 'b', 'EdgeColor', 'none');
                
                % Plot goal
                [X,Y,Z] = sphere(10);
                X = X * 2 + obj.goalPos(1);
                Y = Y * 2 + obj.goalPos(2);
                Z = Z * 2 + obj.goalPos(3);
                obj.goalPlot = surf(X, Y, Z, 'FaceColor', 'g', 'EdgeColor', 'none');
                
                % Plot path
                if ~isempty(obj.path)
                    obj.pathPlot = plot3(obj.path(:,1), obj.path(:,2), obj.path(:,3), 'r-', 'LineWidth', 2);
                end
                
                title('3D Drone Navigation with A* Path Planning');
            else
                % Update existing visualization
                
                % Update drone position
                [X,Y,Z] = sphere(10);
                X = X * 1.5 + obj.currentPos(1);
                Y = Y * 1.5 + obj.currentPos(2);
                Z = Z * 1.5 + obj.currentPos(3);
                set(obj.dronePlot, 'XData', X, 'YData', Y, 'ZData', Z);
                
                % Update dynamic obstacles
                for i = 1:obj.numDynamicObs
                    birdPos = obj.dynamicObstacles(i).position;
                    [X,Y,Z] = sphere(10);
                    X = X * 1 + birdPos(1);
                    Y = Y * 1 + birdPos(2);
                    Z = Z * 1 + birdPos(3);
                    set(obj.dynamicObsPlots(i), 'XData', X, 'YData', Y, 'ZData', Z);
                end
                
                % Update path
                if ~isempty(obj.path)
                    set(obj.pathPlot, 'XData', obj.path(:,1), 'YData', obj.path(:,2), 'ZData', obj.path(:,3));
                else
                    set(obj.pathPlot, 'XData', [], 'YData', [], 'ZData', []);
                end
            end
            
            drawnow;
        end
        
        function [X,Y,Z] = createBox(~, position, size)
            % Create box vertices for plotting
            x = [0 0 0 0 1 1 1 1] * size(1) + position(1);
            y = [0 0 1 1 0 0 1 1] * size(2) + position(2);
            z = [0 1 0 1 0 1 0 1] * size(3) + position(3);
            
            % Define the 6 faces of the box
            X = [x(1:4); x(5:8); x([1 2 6 5]); x([3 4 8 7]); x([1 4 8 5]); x([2 3 7 6])];
            Y = [y(1:4); y(5:8); y([1 2 6 5]); y([3 4 8 7]); y([1 4 8 5]); y([2 3 7 6])];
            Z = [z(1:4); z(5:8); z([1 2 6 5]); z([3 4 8 7]); z([1 4 8 5]); z([2 3 7 6])];
        end
        
        function runSimulation(obj, numSteps)
            % Run the simulation for a number of steps
            for step = 1:numSteps
                % Update dynamic obstacles
                obj = obj.updateDynamicObstacles();
                
                % Check for collisions and replan if needed
                obj = obj.checkForCollisions();
                
                % Move drone
                obj = obj.updateDronePosition();
                
                % Update visualization
                obj = obj.visualize();
                
                % Check if goal reached
                if norm(obj.currentPos - obj.goalPos) < 3
                    disp('Goal reached!');
                    break;
                end
                
                % Pause for animation
                pause(0.05);
            end
        end
    end
end

classdef PriorityQueue < handle
    % Priority queue implementation for A* algorithm
    
    properties (Access = private)
        elements = [];
        priorities = [];
        count = 0;
    end
    
    methods
        function insert(obj, element, priority)
            % Insert element with given priority
            obj.count = obj.count + 1;
            obj.elements(obj.count, :) = element;
            obj.priorities(obj.count) = priority;
        end
        
        function [minElement, minPriority] = extractMin(obj)
            % Extract element with minimum priority
            if obj.isEmpty()
                error('Priority queue is empty');
            end
            
            [minPriority, idx] = min(obj.priorities);
            minElement = obj.elements(idx, :);
            
            % Remove the element
            obj.elements(idx, :) = [];
            obj.priorities(idx) = [];
            obj.count = obj.count - 1;
        end
        
        function [exists, element] = find(obj, position)
            % Check if element with given position exists and return it
            exists = false;
            element = [];
            
            for i = 1:obj.count
                if all(obj.elements(i, 1:3) == position)
                    exists = true;
                    element = obj.elements(i, :);
                    return;
                end
            end
        end
        
        function decreaseKey(obj, oldElement, newElement, newPriority)
            % Decrease priority of existing element
            for i = 1:obj.count
                if all(obj.elements(i, 1:3) == oldElement(1:3))
                    obj.elements(i, :) = newElement;
                    obj.priorities(i) = newPriority;
                    return;
                end
            end
        end
        
        function empty = isEmpty(obj)
            % Check if queue is empty
            empty = (obj.count == 0);
        end
    end
end

%To run the simulation:
%sim = DroneSimulation3D();
%sim.runSimulation(500);