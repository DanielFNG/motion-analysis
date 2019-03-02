classdef PointSet
    
    properties
        Points
        NPoints
    end
    
    methods
        
        function obj = PointSet(points)
            
            if nargin > 0
                
                obj.Points = points;
                obj.NPoints = length(points);
            end
            
        end
        
        function bool = eq(obj, another_obj)
           
            x_1 = sort([obj.Points.x]);
            x_2 = sort([another_obj.Points.x]);
            z_1 = sort([obj.Points.z]);
            z_2 = sort([another_obj.Points.z]);
            
            if all(x_1 == x_2) && all(z_1 == z_2)
                bool = true;
            else
                bool = false;
            end
            
        end
        
        function polygon = constructPolygon(obj, n_points)
            
            lines = zeros(1, obj.NPoints);
            
            for i=1:obj.NPoints
                next = mod(i, obj.NPoints) + 1;
                lines(i) = Line(obj.Points(i), obj.Points(next), n_points);
            end
            
            polygon = Polygon(lines);
            
        end
        
    end

end