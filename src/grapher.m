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
                'y'' [m/s]' 'fi'' [rad/s]' 'x'''' [m^{2}/s]' ...
                'y'''' [m^{2}/s]' 'fi'''' [rad^{2}/s]'};
            index = find(strcmp(simple_options, option), 1);
            if ~isempty(index)
                y = solution(index + 1, :);
                y_legend = strjoin(y_legend_options(index));
            else
                % Look for more complicated options
                extra_option = {'r' 'rprim' 'rbis'};
                y_legend_options = {'r [m]' 'v [m/s]' 'a [m^{3}/s]'};
                index = find(strcmp(extra_option, option), 1);
                if ~isempty(index)
                    y_legend = strjoin(y_legend_options(index));
                    index = 3*(index - 1) + 2;
                    y = sqrt( solution( index, :).^2 + solution( index+1, : ).^2 );
                end
            end
            my_plot = plot(x, y);
            grapher.defaultLook( my_plot );
            xlabel('t [s]'); ylabel(y_legend);
        end
        function addPlot( element, option, point )
            hold on;
            if nargin < 3
                grapher.plot( element, option );
            else
                grapher.plot( element, option, point);
            end
            grapher.defaultLook();
            hold off;
        end
    end
    
end

