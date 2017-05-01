classdef solver < handle
    %SOLVER Managing mechanism and solution
    % Main class of kinematics solver project for Multibody Dynamics class
    % at Faculty of Power and Aeronautical Engineering WUT, Warsaw.
    
    properties
        ground_element
        time = 0;
        
        nRaphson_precision = 1e-10;
        nRaphson_max_iter = 25;
        
        default_time_resolution = 1e-2;
    end
    
    methods
        function obj = solver()
            % Constructor
            % Memory cleanup
            cleaner_el = element( [0 0] );
            for i=1:(numel(cleaner_el.glob_indexer.elements_array) - 1)
                delete(cleaner_el.glob_indexer.elements_array(i));
            end
            delete( cleaner_el );
            clear cleaner_el; clear element;
            
            % Create ground
            obj.ground_element = element( [0 0] );
        end
        function ground = getGround(obj)
            % Returns ground element reference (handle)
            ground = obj.ground_element;
        end
        function elements_array = getElements(obj)
            % Returns array of elements, rather used
            % iternal. #TODO Specify access.
            elements_array = obj.ground_element.glob_indexer.elements_array;
        end
        
        % Methods for obtaining / updating q, q', q''
        function q = getQ(obj)
            % Returns stystem's q.
            q = [];
            els_array = obj.getElements(); % #TODO Optimize for all methods!
            for i=2:numel(els_array)
                q = [q; els_array(i).r_c; els_array(i).fi_c]; % #TODO Optimize for all methods!
            end
        end
        function q = getQPrim(obj)
            % Returns system's q'.
            q = [];
            els_array = obj.getElements();
            for i=2:numel(els_array)
                q = [q; els_array(i).r_c_prim; els_array(i).fi_c_prim];
            end
        end
        function q = getQBis(obj)
            % Returns system's q''.
            q = [];
            els_array = obj.getElements();
            for i=2:numel(els_array)
                q = [q; els_array(i).r_c_bis; els_array(i).fi_c_bis];
            end
        end
        function setQ(obj, q)
            % Distribute q among all elements.
            % Input:
            %  * q - vecotr q
            els_array = obj.getElements();
            q_i = 1;
            for i=2:numel(els_array)
                els_array(i).r_c = [q(q_i) q(q_i + 1)]';
                els_array(i).fi_c = q(q_i + 2);
                q_i = q_i + 3;
            end
        end
        function setQPrim(obj, q)
            % Distribute q' among all elements.
            % Input:
            %  * q - vector q'
            els_array = obj.getElements();
            q_i = 1;
            for i=2:numel(els_array)
                els_array(i).r_c_prim = [q(q_i) q(q_i + 1)]';
                els_array(i).fi_c_prim = q(q_i + 2);
                q_i = q_i + 3;
            end
        end
        function setQBis(obj, q)
            % Distribute q'' among all elements.
            % Input:
            %  * q - vector q''
            els_array = obj.getElements();
            q_i = 1;
            for i=2:numel(els_array)
                els_array(i).r_c_bis = [q(q_i) q(q_i + 1)]';
                els_array(i).fi_c_bis = q(q_i + 2);
                q_i = q_i + 3;
            end
        end
        
        % Methods for obtaining Phi, Jacobi etc.
        function Phi = getPhi(obj)
            % Returns system's Phi (constraints' value)
            Phi = [];
            els_array = obj.getElements();
            for i=1:numel(els_array)
                Phi = [Phi; els_array(i).getPhi()];
            end
        end
        function Phi_prim = getPhiPrim(obj)
            % Returns system's Phi'
            Phi_prim = [];
            els_array = obj.getElements();
            for i = 1:numel(els_array)
                Phi_prim = [Phi_prim; els_array(i).getPhiPrim()];
            end
        end
        function Jacobi = getJacobi(obj)
            % Returns system's Jacobi
            Jacobi = [];
            els_array = obj.getElements();
            for i=2:numel(els_array)
                for j=1:numel(els_array(i).vector_constraints)
                    % This function goes 2 levels deep - it means that
                    % first it accesses an element and then iterate through
                    % all constraints of which the element is base element.
                    Jacobi = obj.conMatrices(Jacobi, els_array(i).vector_constraints(j).getJacobi());
                end
            end
        end
        function Gamma = getGamma(obj)
            % Returns system's Gamma
            Gamma = [];
            els_array = obj.getElements();
            for i=2:numel(els_array)
                for j=1:numel(els_array(i).vector_constraints)
                    % This function goes 2 levels deep - same as
                    % obj.getJacobi().
                    Gamma = [Gamma; els_array(i).vector_constraints(j).getGamma()];
                end
            end
        end
        
        % Solving functions
        function q = nRaphson(obj, q0)
            % Solves mechanism with NR method.
            % Input:
            %  * q0 - has default value - if it is not set, then elements 
            %       do not change their coords.
            if nargin == 2, obj.setQ( q0 ); else q0 = obj.getQ(); end
            
            Phi = obj.getPhi();
            iter = 1; % counter
            while( (norm(Phi) > obj.nRaphson_precision) && (iter < obj.nRaphson_max_iter) )
                Phi = obj.getPhi();
                obj.setQ(obj.getQ() - obj.getJacobi()\Phi);
                iter = iter + 1;
                
            end
            if iter > 25
                disp(fprintf('B£¥D NEWTON-RAPHSON: Po %d iteracjach nie uzyskano zbie¿noœci.', ...
                    obj.nRaphson_max_iter));
                obj.setQ( q0 );
            end
            q = obj.getQ(); % return result q (if someone really need it as return)
        end
        function solve(obj, timespan)
            % Main function for finding solution of mechanism in
            % given timespan.
            % Input:
            %   * timespan - timespan for which find solution. Given in
            %           [t_start t_end] format or [t_start:t_resolution:t_end]
            obj.broadcastSolverRef(); % Make sure that elements know about solver here!
            if numel(timespan) == 2
                timespan = [timespan(1):obj.default_time_resolution:timespan(2)];
            end
            disp(sprintf('ROZWI¥ZYWANIE: Rozpoczêto rozwi¹zywanie (%d chwil)', numel(timespan)));
            for i=1:numel(timespan)
                % Solve mechanism for single time
                obj.time = timespan(i);
                found_q = obj.nRaphson(); % Find q
                
                jacobi = obj.getJacobi();
                phiphi = obj.getPhiPrim();
                obj.setQPrim( -jacobi \ obj.getPhiPrim() ); % Find q'
                obj.setQBis( jacobi \ obj.getGamma() ); % Find q''
                
                obj.saveSingleSolution();
            end
            disp(sprintf('ROZWI¥ZYWANIE: Zakoñczono rozwi¹zywanie.'));
        end
        
        % Post-processing functions
        function solution_data = getSolution(obj, option)
            % Returns solution from all elements in a big
            % matrix format with timespan as the header.
            % Input:
            %  * option - has default value - set it to 'q', 'qprim' or
            %   'qbis' to narrow returned data respectively; if it is not
            %   set then whole solution_data is returned.
            
            % Manage default value
            els_array = obj.getElements();
            solution_data = els_array(2).solution(1, :);       
            if nargin ~= 2
                option = '';
            end
            for i = 2:numel(els_array)
                el_solution = els_array(i).getSolution( option );
                solution_data = [solution_data; el_solution(2:size( el_solution, 1), :)];
            end
        end
        function animateSolution(obj, time_factor)
            % Animates solution with certain time factor.
            % Input:
            %  * time_factor - time factor of animation, e.g time_factor =
            %       1.25 => animation x 1.25 speed of real time
            if nargin == 1, time_factor = 1; end
            animation_feed = obj.getSolution('q');
            time_delay = (animation_feed(1, 2) - animation_feed(1, 1)) / time_factor;
            frames = size(animation_feed, 2);
            current_frame = 1;
            number_of_qs = 3*numel(obj.getElements) - 3;
            
            % Set axis limits
            figure; % Creates new figure for animation #TODO Lock animation
                    % so that it does not overwrite other figures in case
                    % the animation figure is closed.
            obj.drawMechanism(); obj.eraseMechanism(); % Make sure figure window is on
            axis( obj.findAnimationBoundaries()); axis manual;
            
            disp('ANIMACJA: Proszê wcisn¹æ Ctrl+C, aby zakoñczyæ.');          
            while true
                obj.eraseMechanism();
                obj.setQ(animation_feed(2:(number_of_qs + 1), current_frame));
                obj.drawMechanism();
                current_frame = current_frame + 1;
                pause( time_delay );
                if current_frame > frames, current_frame = 1; end
            end
        end
        
        % Misc functions
        function drawMechanism(obj)
            % Draws mechanism but for the ground element.
            els_array = obj.getElements();
            for i = 2:numel(els_array)
                els_array(i).drawElement();
            end
            axis equal; grid on; % Just make axis ratio = 1 and turn on grid
        end
        function eraseMechanism(obj)
            % Erase mechanism but for the ground element.
            els_array = obj.getElements();
            for i = 2:numel(els_array)
                els_array(i).eraseElement();
            end
        end
    end
    methods (Access = private)
        function broadcastSolverRef(obj)
            % Broadcasts reference (handle) to all elements (so that they
            % have access to time for example).
            els_array = obj.getElements();
            for i=1:numel(els_array)
                els_array(i).solver = obj;
            end
        end
        function matrix = conMatrices(obj, A, B)
            % Concatenate matrices which are of different number of columns.
            % Input:
            %  * A - matrix A
            %  * B - matrix B (might have different number of columns)
            cols_diff = size(B, 2) - size(A, 2);
            A = [A zeros( size(A, 1), cols_diff )];
            B = [B zeros( size(B, 1), -cols_diff)];
            matrix = [A; B];
        end
        function saveSingleSolution(obj)
            % Saves solution from current time to special containers 
            % solution matrix) in elements;
            els_array = obj.getElements();
            for i=1:numel(els_array)
                els_array(i).saveSingleSolution();
            end
        end
        function boundaries = findAnimationBoundaries(obj)
            % Finds boundaries in which animation of given solution
            % should be drawn.
            els_array = obj.getElements();
            max_x = -inf; max_y = -inf;
            min_x = inf; min_y = inf;
            for i=1:numel(els_array)
                el_max_x = max(els_array(i).solution(2, :));
                el_max_y = max(els_array(i).solution(3, :));
                el_min_x = min(els_array(i).solution(2, :));
                el_min_y = min(els_array(i).solution(3, :));
                
                if max_x < el_max_x, max_x = el_max_x; end
                if max_y < el_max_y, max_y = el_max_y; end
                if min_x > el_min_x, min_x = el_min_x; end
                if min_y > el_min_y, min_y = el_min_y; end
            end
            extend_x = .2*(max_x - min_x); extend_y = .2*(max_y - min_y);
            boundaries = [(min_x - extend_x) (max_x + extend_x) ...
                (min_y - extend_y) (max_y + extend_y)];
        end
    end
end