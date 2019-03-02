classdef Line
   
    properties
        x
        z
        n_points = 100
    end
    
    methods
        
        function obj = Line(start, finish, n_points)
            
            if nargin > 0
                if nargin == 3
                    obj.n_points = n_points;
                end
                obj.verifyPoints(start, finish);
                obj.x = linspace(start.x, finish.x, obj.n_points);
                obj.z = linspace(start.z, finish.z, obj.n_points);
            end
            
        end
        
        function start = getStartPoint(obj)
           
            start = Point(obj.x(1), obj.z(1));
            
        end
        
        function finish = getFinishPoint(obj)
            
            finish = Point(obj.x(end), obj.z(end));
            
        end
        
    end
    
    methods (Static)
       
        function verifyPoints(point1, point2)
            
            if point1 == point2
                error('Can''t draw a line between two equal Points.');
            end
            
        end
        
    end
    
end