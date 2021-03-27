 %% The following function computes the block of jacobian caused 
 %   by relative bearing measurement
     function [H_bearing_i,H_bearing_j,H_range_i,H_range_j] = H_relative(pos_i,pos_j)
        % For details of the following equation, readers are refered to
        % eq.(11) in Agostino et al. 2005
          delta_x = pos_j(1) - pos_i(1);
          delta_y = pos_j(2) - pos_i(2);
          H_bearing_i = [delta_y/(delta_x^2 + delta_y^2), -delta_x/(delta_x^2 + delta_y^2),-1];
          H_bearing_j = [-delta_y/(delta_x^2 + delta_y^2),delta_x/(delta_x^2 + delta_y^2),0];
        % For details of the following equation, readers are refered to
        % eq.(12) in Agostino et al. 2005 
          H_range_i = [-delta_x/sqrt(delta_x^2+delta_y^2),-delta_y/sqrt(delta_x^2+delta_y^2),0];
          H_range_j = [delta_x/sqrt(delta_x^2+delta_y^2),delta_y/sqrt(delta_x^2+delta_y^2),0];
      end