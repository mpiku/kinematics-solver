classdef grapher < handle
    %GRAPHER Graphs plots
    % Helper class to contain all methods needed to graph needed plots
    
    methods(Static)
        function defaultLook( plot )
            % Provides default look to the plot
            ax = gca;
            grid on; grid minor;
            ax.GridColor = 'black';
            ax.MinorGridColor = 'black';
            ax.GridAlpha = 0.3;
            ax.MinorGridAlpha = 0.2;
            
            if nargin == 1
                plot.LineWidth = 3;
            end

        end
        function plot_(x, y)
            % Creates plot with default look
            plot(x, y);
        end
        function plot( element, option, point )
            % Creates plot of specified option in given element.
            % Input:
            %  * element - element which the plot is created for
            %  * option - option on what to plot
            %  * point - has default value - point of the element for which
            %   plot is created. If nothing is passed, then plot is created
            %   for center of the element.
            if nargin < 3
                solution = element.getSolution();
            else
                solution = element.getSolutionForPoint( point );
            end
            x = solution(1, :);
            simple_options = {'x' 'y' 'fi' 'xprim' 'yprim' 'fiprim' ...
                'xbis' 'ybis' 'fibis'};
            y_legend_options = {'x [m]' 'y [m]' 'fi [rad]' 'x'' [m/s]' ...
                'y'' [m/s]' 'fi'' [rad/s]' 'x'''' [m/s^{2}]' ...
                'y'''' [m/s^{2}]' 'fi'''' [rad/s^{2}]'};
            index = find(strcmp(simple_options, option), 1);
            if ~isempty(index)
                y = solution(index + 1, :);
                y_legend = strjoin(y_legend_options(index));
            else
                % Look for more complicated options
                extra_option = {'r' 'rprim' 'rbis'};
                y_legend_options = {'r [m]' 'v [m/s]' 'a [m/s^{2}]'};
                index = find(strcmp(extra_option, option), 1);
                if ~isempty(index)
                    y_legend = strjoin(y_legend_options(index));
                    index = 3*(index - 1) + 2;
                    y = sqrt( solution( index, :).^2 + solution( index+1, : ).^2 );
                end
            end
            my_plot = plot(x, y);
            kinSolver.grapher.defaultLook( my_plot );
            xlabel('t [s]'); ylabel(y_legend);
        end
        function addPlot( element, option, point )
            % Adds plot to currently active figure.
            % Input:
            %  * element - element which the plot is created for
            %  * option - option on what to plot
            %  * point - has default value - point of the element for which
            %   plot is created. If nothing is passed, then plot is created
            %   for center of the element.
            hold on;
            if nargin < 3
                kinSolver.grapher.plot( element, option );
            else
                kinSolver.grapher.plot( element, option, point);
            end
            kinSolver.grapher.defaultLook();
            hold off;
        end
        function trajectory( element, point )
            % Plot trajectory of the element with line width respective to
            % current point's velocity in relation to overall point's
            % velocity.
            % Input:
            %  * element - element which the plot is created for
            %  * point - has default value - point of the element for which
            %   plot is created. If nothing is passed, then plot is created
            %   for center of the element.
            max_line_width = 10;
            if nargin < 2
                solution = element.getSolution();
            else
                solution = element.getSolutionForPoint( point );
            end
            point_x = solution(2, :);
            point_y = solution(3, :);
            point_vel = sqrt( solution( 5, :).^2 + solution( 6, : ).^2 );
            line_width = smooth( ( max_line_width .* ( ( point_vel - min( point_vel ) ) ...
                ./ ( max( point_vel - min( point_vel ) ) ) ) ) + ...
                ones( 1, size( point_vel, 2 ) ) );
            figure; % Create new figure
            for i=1:( numel(point_x) - 1)
                line( point_x(i:i+1), point_y(i:i+1), 'LineWidth', line_width(i));
            end
            kinSolver.grapher.defaultLook();
            xlabel('x [m]'); ylabel('y [m]');
        end
    end
    
end

