classdef PointSet
% Class for storing & working with sets of points in x/z space. A point is
% simply a struct with an x and z field. 
    
    properties
        Points
        NPoints
    end
    
    methods
        
        function obj = PointSet(points)
        % Construct a PointSet from an array of points.    
            
            if nargin > 0
                obj.Points = points;
                obj.NPoints = length(points);  % Store number of points.
            end
            
        end
        
        function x = getXValues(obj)
        % Get an array consisting of the x values of all points in the set.
            
            x = [obj.Points.x];
            
        end
        
        function z = getZValues(obj)
        % Get an array consisting of the z values of all points in the set.
            
            z = [obj.Points.z];
            
        end
        
        function bool = eq(obj, another_obj)
        % Test equality of PointSet objects.
        %
        % Two PointSet objects are equal if the sorted arrays of the x & z
        % values are equivalent.
            
            x_1 = sort(obj.getXValues());
            x_2 = sort(another_obj.getXValues());
            z_1 = sort(obj.getZValues());
            z_2 = sort(another_obj.getZValues());
            
            if all(x_1 == x_2) && all(z_1 == z_2)
                bool = true;
            else
                bool = false;
            end
            
        end
        
        function polygon = constructPolygon(obj, varargin)
        % Construct a Polygon from a pointset.
        %
        % If provided, varargin is a single argument which controls the
        % number of points used to construct the lines between points.
        
            % Assert that we need at least 3 points.
            if obj.NPoints < 3
                error('Can''t create a polygon from less than 3 points.');
            end
            
            % Initialise array of Line objects.
            lines(obj.NPoints) = Line();  
            
            % Create each Line in turn.
            for i=1:obj.NPoints
                next = mod(i, obj.NPoints) + 1;
                lines(i) = Line(obj.Points(i), obj.Points(next), varargin{:});
            end
            
            % Group them in to a LineSet.
            line_set = LineSet(lines);
            
            % Create a Polygon from the LineSet.
            polygon = Polygon(line_set);
            
        end
        
    end

end