function plot_semantic(obj, figure_title, filename)
    % plot ogm
    color_semantic = [[0 0.4470 0.7410]; [0.8500 0.3250 0.0980]; 
                      [0.9290 0.6940 0.1250]; [0.4940 0.1840 0.5560]; 
                      [0.4660 0.6740 0.1880]; [0.3010 0.7450 0.9330]; 
                      [0.6350 0.0780 0.1840]];
    figure; hold on;
    axis([obj.range_x obj.range_y]);
    axis equal tight

    unknown = obj.map.mean(1,:);
    
    % Compute colors
    colors = zeros(obj.map.size, 3);
    
    for i = 1:obj.map.size
        if isequal(obj.map.mean(i,:), unknown)
            colors(i, :) = 0.5*[1 1 1];
        else
            [~,k] = max(obj.map.mean(i,:));
            if k == obj.num_classes+1 % free space
                colors(i, :) = [1 1 1];
            else
                colors(i, :) = color_semantic(k, :);
            end
        end
    end
    
    x = obj.range_x(1):obj.grid_size:obj.range_x(2);
    y = obj.range_y(1):obj.grid_size:obj.range_y(2);
    [X, Y] = meshgrid(x, y);
    
    Z = ones(length(x), length(y));
    colors = reshape(colors, [length(x), length(y), 3]);
    
    surf(X, Y, Z, colors, 'edgecolor', 'none');
    view(0,90)
    
    set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
    set(groot, 'defaultLegendInterpreter','latex');
    set(gca, 'fontsize', 16)
    title(figure_title, 'FontSize', 16)
    print('-opengl','-dpng', '-r600', filename)
end