function plot_variance(obj, figure_title, filename)
    % plot ogm variance using colormap
    figure; hold on;
    axis([obj.range_x obj.range_y]);
    axis equal tight

    v_max = max(obj.map.variance);
    x = obj.range_x(1):obj.grid_size:obj.range_x(2);
    y = obj.range_y(1):obj.grid_size:obj.range_y(2);
    z = reshape(obj.map.variance, [length(x), length(y)]);

    [X, Y] = meshgrid(x, y);
    Z = z;
    surf(X, Y, Z, 'edgecolor', 'none');
    view(0,90)
    
    colormap jet
    colorbar 
    caxis([0, v_max])
    
    set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
    set(groot, 'defaultLegendInterpreter','latex');
    set(gca, 'fontsize', 16)
    title(figure_title, 'FontSize', 16)
    print('-opengl','-dpng', '-r600', filename)
end