classdef Line
% Simple class for working with lines between points in x/z space. A point is
% simply a struct with an x and z field. 
   
    properties
        x
        z
        n_points = 100
    end
    
    methods
        
        function obj = Line(start, finish, n_points)
        % Construct Line of n_points between start point and finish point.
            
            if nargin > 0
                if nargin == 3
                    obj.n_points = n_points;
                end
                obj.verifyPoints(start, finish);  % Simple check.
                
                % Create line using linear interpolation between points.
                obj.x = linspace(start.x, finish.x, obj.n_points);
                obj.z = linspace(start.z, finish.z, obj.n_points);
            end
            
        end
        
        function start = getStartPoint(obj)
        % Returns the start point of the line.
           
            start.x = obj.x(1);
            start.z = obj.z(1);
            
        end
        
        function finish = getFinishPoint(obj)
        % Returns the end point of the line.
            
            finish.x = obj.x(end);
            finish.z = obj.z(end);
            
        end
        
    end
    
    methods (Static)
       
        function verifyPoints(point1, point2)
        % Small check - points must be not equal to create a Line.
            
            if point1.x == point2.x && point1.z == point2.z
                error('Can''t draw a line between two equal Points.');
            end
            
        end
        
    end
    
end