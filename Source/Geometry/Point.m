classdef Point
% Simple class for working with points in x/z space and testing equality.
   
    properties
        x
        z
    end
    
    methods 
        
        function obj = Point(x, z)
        % Construct a point from a single pair of x and z values. 
           
            if nargin > 0
                obj.x = x;
                obj.z = z;
            end
            
        end
        
        function bool = eq(obj, another_obj)
        % Points are equal if they are equal in terms of x & z. 
           
            if obj.x == another_obj.x && obj.z == another_obj.z
                bool = true;
            else
                bool = false;
            end
            
        end
        
    end
    
end