function plot_mean(obj, figure_title, filename)
    % plot ogm using conventional black-gray-white cells
    figure; hold on;
    axis([obj.range_x obj.range_y]);
    axis equal tight
    
    x = obj.range_x(1):obj.grid_size:obj.range_x(2);
    y = obj.range_y(1):obj.grid_size:obj.range_y(2);
    z = reshape(1-obj.map.mean, [length(x), length(y)]);
    
    [X, Y] = meshgrid(x, y);
    Z = z;
    surf(X, Y, Z, 'edgecolor', 'none');
    view(0,90)
    colormap('gray')
        
    set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
    set(groot, 'defaultLegendInterpreter','latex');
    set(gca, 'fontsize', 16)
    title(figure_title, 'FontSize', 16)
    print('-opengl','-dpng', '-r600', filename)
end