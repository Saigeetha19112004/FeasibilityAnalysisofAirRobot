% PriorityQueue.m
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