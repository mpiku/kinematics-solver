classdef indexer < handle
    %INDEXER Workaround for STATIC DATA
    
    properties
        index = 0; % element index (0 - reserved for ground element)
        elements_array = []; % keep reference (handles) to all elements available
    end
    
    methods
        function i = nextObj( obj, element )
            % Provide new element for indexer.
            % Input:
            %  * element - element to be pushed to our static watchdog
            i = obj.index;
            obj.index = obj.index + 1;
            obj.elements_array = [ obj.elements_array element ];
        end
    end
    
end

