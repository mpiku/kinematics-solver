classdef element < handle
    %ELEMENT Represents single element in mechanism
    %   Implements element of mechanism which has its own coordinate system
    %   in absolute coordinates approach.
    
    properties
        glob_indexer = kinSolver.indexer();
        index = 0;
        
        r_c = zeros(2, 1);
        fi_c = 0;
        
        r_c_prim = zeros(2, 1);
        fi_c_prim = 0;
        
        r_c_bis = zeros(2, 1);
        fi_c_bis = 0;
        
        solution = []; % Keeps information about time, q, q', q'' in format:
                       % solution(1, :) = time, solution(2-3-4, :) = q etc.
        
        cell_points = {}; % Points of element in local coordinate system
        vector_constraints = []; % Constraints for which the element is a
                                 % base element.
                                 % #TODO Another container should be
                                 % implemented so that it can keep
                                 % information about constraints which this
                                 % element constitue but not as a base
                                 % element, e.g. vector_of_known_constr
       vector_lines = [];
                                 
       % Software archictecture properties
       solver = struct('time', 0); % Reference to the solver which is known
                                   % to the object after call of
                                   % solver.broadcastSolverRef();
    end
    
    methods
        function obj = element( point )
            %   Constructor
            %   Input:
            %    * point - point being a center of gracity for the element
            obj.index = obj.glob_indexer.nextObj( obj );
            obj.r_c = [point(1) point(2)]';
        end
        function delete( obj )
            % Destructor removes references (handles) to the object
            % from static obj.glob_indexer property.
            delete( obj.glob_indexer.elements_array( obj.index + 1) );
        end
        function addPoint( obj, point )
            %   Adds point to cell array of element's points
            %   (obj.cell_points).
            point = [point(1) point(2)]';
            local_point = point - obj.r_c;
            obj.cell_points = [ obj.cell_points local_point ];
        end
        
        % Adding constraints methods
        % #TODO? This might be implemented in constraint classes. Think
        % over which place is more proper. Basically these methods checks
        % wheter creating give constraint can be added and then they create
        % it pushing the constraint to obj.vector_constraints.
        function add_K_JointConstr(obj, element, point)
            %   Input:
            %    * element - 2nd element
            %    * point - point common for both elements
            my_point_index = obj.whichIndex( point );
            element_point_index = element.whichIndex( point );
            if ( my_point_index * element_point_index ) < 0
                disp( sprintf('B£¥D DODAWANIA WIÊZU: Para obrotowa(%d, %d)', ...
                obj.index, element.index) );
                return
            end
            obj.vector_constraints = [ obj.vector_constraints ...
                kinSolver.kJointConstr(obj, my_point_index, element, element_point_index) ];
            disp( sprintf('DODANO WIÊZ: Para obrotowa(%d, %d)', ...
                obj.index, element.index) );
        end
        function add_K_PrismConstr(obj, element, point_A, point_B)
            %   Method calculate automatically vector perpendicular to the
            %   axis of movement (v_B aka v_j).
            %   Input:
            %    * element - 2nd element
            %    * point_A - point on base element
            %    * point_B - point on 2nd element
            my_point_A_index = obj.whichIndex( point_A );
            element_point_B_index = element.whichIndex( point_B );
            if my_point_A_index < 0
                disp( sprintf('B£¥D DODAWANIA WIÊZU: Para postêpowa(%d, %d) - b³êdny punkt w el. bazowym', ...
                obj.index, element.index) );
                return
            elseif element_point_B_index < 0
                disp( sprintf('B£¥D DODAWANIA WIÊZU: Para postêpowa(%d, %d) - b³êdny punkt w el. do³¹czonym', ...
                obj.index, element.index) );
                return
            end
            % Find v_B element
            v_AB = point_B - point_A;
            v_B = kinSolver.rot(element.fi_c)' * [v_AB(2) -v_AB(1)]';
            % Push to obj.vector_constraints
            obj.vector_constraints = [ obj.vector_constraints ...
                kinSolver.kPrismConstr(obj, my_point_A_index, element, element_point_B_index, v_B) ];
            disp( sprintf('DODANO WIÊZ: Para postêpowa(%d, %d)', ...
                obj.index, element.index) );
        end
        function add_D_JointConstr(obj, element, point, f, f_prim, f_bis)
            % Checks if kinematic constraint of the type already exists
            % and on succeess adds driving constraint.
            % Input:
            %  * element - 2nd element
            %  * point - common point for both elements
            %  * f, f_prim, f_bis - passed as anonymous functions @(t) ...
            %       self explaining
            cstr_avail = false;
            for i = 1:numel( obj.vector_constraints )
                cstr = obj.vector_constraints(i);
                if element == cstr.el_B
                    if cstr.el_B.whichIndex( point ) == cstr.B_index
                        cstr_avail = true;
                        break
                    end
                end
            end
            % #TODO Implement check on future vector_of_known_constraints
            if ~cstr_avail
                disp( sprintf('B£¥D DODAWANIA WIÊZU: Para obrotowa - kieruj¹cy(%d, %d) - brak wiêzu kinematycznego', ...
                obj.index, element.index) );
                return
            end
            obj.vector_constraints = [ obj.vector_constraints ...
                kinSolver.dJointConstr(obj, element, f, f_prim, f_bis) ];
            disp( sprintf('DODANO WIÊZ: Para obrotowa - kieruj¹cy(%d, %d)', ...
                obj.index, element.index) );
        end
        function add_D_PrismConstr(obj, element, point_A, point_B, f, f_prim, f_bis)
            % Checks if kinematic constraint of the type already exists
            % and on succeess adds driving constraint. It automatically
            % calculates u versor (u_B aka u_j).
            % Input:
            %  * element - 2nd element
            %  * point_A - point associated with base element
            %  * point_B - point associated with 2nd element
            %  * f, f_prim, f_bis - passed as anonymous functions @(t) ...
            %       self explaining
            cstr_avail = false;
            for i = 1:numel( obj.vector_constraints )
                cstr = obj.vector_constraints(i);
                if element == cstr.el_B
                    if cstr.el_B.whichIndex( point_B ) == cstr.B_index ...
                            && cstr.el_A.whichIndex( point_A ) == cstr.A_index ...
                        cstr_avail = true;
                        break
                    end
                end
            end
            % #TODO Implement check on future vector_of_known_constraints
            if ~cstr_avail
                disp( sprintf('B£¥D DODAWANIA WIÊZU: Para postêpowa - kieruj¹cy(%d, %d) - brak wiêzu kinematycznego', ...
                obj.index, element.index) );
                return
            end
            u_B= kinSolver.rot( element.fi_c ) * ( ( point_B - point_A ) / norm( point_B - point_A ) );
            my_point_A_index = obj.whichIndex( point_A );
            element_point_B_index = element.whichIndex( point_B );
            % Push to obj.vector_constraints
            obj.vector_constraints = [ obj.vector_constraints ...
                kinSolver.dPrismConstr(obj, my_point_A_index, element, element_point_B_index, u_B, f, f_prim, f_bis) ];
            disp( sprintf('DODANO WIÊZ: Para postêpowa - kieruj¹cy(%d, %d)', ...
                obj.index, element.index) );
        end
        
        % Getting matrices associated with constraints
        function Phi = getPhi(obj)
            % Returns Phi constraints matrix (n x 1) of the element
            Phi = [];
            for i = 1:numel( obj.vector_constraints )
                Phi = [Phi; obj.vector_constraints(i).getConstraint()];
            end
        end
        function Phi_prim = getPhiPrim(obj)
            % Returns Phi' (n x 1) of the element
            Phi_prim = [];
            for i = 1:numel( obj.vector_constraints )
                Phi_prim = [Phi_prim; obj.vector_constraints(i).getPhiPrim()];
            end
        end
        
        % Misc functions
        function drawElement(obj, color)
            % Draws element.
            % Input:
            %  * color - if not specified methods finds color based on
            %  element's index (obj.index)
            color_arr = ['m' 'y' 'c' 'r' 'g' 'b' 'k'];
            if nargin == 2, element_color = color;
            else
                element_color = color_arr(mod(obj.index, 7) + 1);
            end
            for i=1:size(obj.cell_points, 2)
                % #TODO Might be written better.
                global_point = obj.r_c + kinSolver.rot(obj.fi_c) * cell2mat( obj.cell_points(i) );
                obj.vector_lines = [ obj.vector_lines line([obj.r_c(1) global_point(1)], [obj.r_c(2) global_point(2)], ...
                    'Color', element_color)];
            end
            p_loc = cell2mat( obj.cell_points );
            boundary_indices = boundary( p_loc(1, :)', p_loc(2, :)' );
            p_glob = kinSolver.rot( obj.fi_c ) * p_loc + obj.r_c * ones(1, numel(p_loc) / 2);
            obj.vector_lines = [ obj.vector_lines line( p_glob(1, boundary_indices), p_glob(2, boundary_indices), ...
                    'Color', element_color, 'LineWidth', 3) ];
        end
        function eraseElement(obj)
            % Erases element from the current drawing.
            delete(obj.vector_lines);
            clear obj.vector_lines;
            obj.vector_lines = [];
        end
        function ind = whichIndex(obj, point)
            % Returns index of given point in cell array of local points
            % (obj.cell_points). If point is not found -1 is returned.
            % Input:
            %  * point - point index of which is to be found
            ind = -1;
            local_point = point - obj.r_c;
            for i=1:size( obj.cell_points, 2)
                if cell2mat(obj.cell_points(i)) == local_point
                    ind = i;
                    break
                end
            end
        end
        function saveSingleSolution(obj)
            % Saves data to obj.solution matrix
            next_solution = [obj.solver.time; obj.r_c; obj.fi_c; ... % q
                obj.r_c_prim; obj.fi_c_prim; ... % q'
                obj.r_c_bis; obj.fi_c_bis]; % q''
            obj.solution = [obj.solution next_solution];
        end
        function solution = getSolution(obj, option, point)
            % Returns solution in format of big matrix with timespan as
            % header.
            % Input:
            %  * option - has default value - choose 'q', 'qprim' or 'qbis'
            %   to return respective part of the solution. If nothing is
            %   passed, returns whole solution. It's more like trimming
            %   received data than optimization.
            %  * point - has default value - returns solution for point of
            %   the elemenet given the point coordinates are provided in
            %   global coords system in time 0.
            rows_number = 2:10;
            if nargin > 1
                if strcmp(option, 'q')
                    rows_number = 2:4;
                elseif strcmp(option, 'qprim')
                    rows_number = 5:7;
                elseif strcmp(option, 'qbis')
                    rows_number = 8:10;
                end
            end
            if nargin < 3
                solution = [obj.solution(1, :); obj.solution(rows_number, :)];
            else
                temp = obj.getSolutionForPoint( point );
                solution = [obj.solution(1, :); temp(rows_number, :)];
            end
        end
        function solution = getSolutionForPoint( obj, point )
            % Returns whole solution matrix calculated for given point in
            % the element.
            % Input:
            %  * point - has default value -  point of the elemenet given
            %   the point coordinates are provided in global coords system
            %   in time 0.
            s = point - obj.solution(2:3, 1);
            time_steps = size( obj.solution, 2 );
            solution = obj.solution(1, 1:time_steps);
            omega = [0 -1; 1 0];
            for i=1:time_steps
                solution(2:3, i) = obj.solution(2:3, i) + kinSolver.rot(obj.solution(4, i))*s;
                solution(4, i) = obj.solution(4, i);
                solution(5:6, i) = obj.solution(5:6, i) + ...
                    omega*kinSolver.rot(obj.solution(4, i))*s*obj.solution(7, i);
                solution(7, i) = obj.solution(7, i);
                solution(8:9, i) = obj.solution(8:9, i) + ...
                     omega*kinSolver.rot(obj.solution(4, i))*s*obj.solution(10, i) + ...
                     - kinSolver.rot(obj.solution(4, i))*s*( obj.solution(7, i)^2 );
                solution(10, i) = obj.solution(10, i);
            end
        end
    end
    
end

