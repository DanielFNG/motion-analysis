classdef Line
% Simple class for working with lines between Points in x/z space.
   
    properties
        x
        z
        n_points = 100
    end
    
    methods
        
        function obj = Line(start, finish, n_points)
        % Construct Line of n_points between start Point and finish Point.
            
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
        % Returns the start Point of the line.
           
            start = Point(obj.x(1), obj.z(1));
            
        end
        
        function finish = getFinishPoint(obj)
        % Returns the end Point of the line.
            
            finish = Point(obj.x(end), obj.z(end));
            
        end
        
    end
    
    methods (Static)
       
        function verifyPoints(point1, point2)
        % Small check - Points must not be equal to create a Line.
            
            if point1 == point2
                error('Can''t draw a line between two equal Points.');
            end
            
        end
        
    end
    
end