classdef Polygon
% Class for storing & working with polygons in x/z space.
   
    properties (SetAccess = private)
        LineSet
    end
    
    methods (Access = {?PointSet})  % Only creatable from a PointSet.
       
        function obj = Polygon(line_set)
        % Polygons are created from LineSet objects which are verified to
        % be connected.
            
            obj.LineSet = line_set;
            
        end
        
    end
    
    methods
        
        function vals = getXValues(obj)
        % Returns the x values of all points which make up the polygon.
            
            vals = [obj.LineSet.Lines(:).x];
            
        end
        
        function vals = getZValues(obj)
        % Returns the z values of all points which make up the polygon.
            
            vals = [obj.LineSet.Lines(:).z];
            
        end
        
    end
    
end