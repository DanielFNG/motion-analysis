classdef Polygon
% Class for storing & working with polygons in x/z space.
   
    properties
        LineSet
    end
    
    methods
       
        function obj = Polygon(line_set)
        % Polygons are created from LineSet objects which are verified to
        % be connected.
            
            obj.verifyLineSet(line_set);
            obj.LineSet = line_set;
            
        end
        
        function vals = getXValues(obj)
        % Returns the x values of all points which make up the polygon.
            
            vals = [obj.LineSet.Lines(:).x];
            
        end
        
        function vals = getZValues(obj)
        % Returns the z values of all points which make up the polygon.
            
            vals = [obj.LineSet.Lines(:).z];
            
        end
        
    end
    
    methods (Static)
        
        function verifyLineSet(line_set)
        % Verifies that the lines composing a LineSet are connected, and
        % thus suitable for constructing a polygon. 
           
           [start, finish] = line_set.getEndPoints();
            if ~(start == finish)
                error('These lines are not connected, this is not a polygon.');
            end
            
        end
        
    end
    
end