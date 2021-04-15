function [X_ground_truth,landmark,measurement_z_landmark,measurement_z_relative, action_for_robots, numSteps] ...
                                         = real_problem_gen(t_step, Num)
%{
    X_ground_truth = 3* Num by TIME matrix.every 3 rows
                     are the [x;y;orientation] measurements at given time
                                         
    landmark       = 15 by 2 matrix. gives the x and y pos for each
                     landmark for example, landmark(2,:) represents [x,y] of
                     second landmark
                                         
    measurment_z_landmark = 3*Num by TIME matrix. every 3 rows are the
                            [landmark_num; range; bearing] measurement
                            at given time. If no measurement at this time,
                            landmark_num = -1;
    
    measurement_z_relative = 2*Num*(Num-1) by TIME matrix. Described in slack
    action_for_robots    = 3*Num by TIME matrix. Every three rows  are the
                           [v;w; gamma] measurements at given time
%}
                                                                               
% Constant, object declerations    
DATASET = 2;
TIME_STEP = t_step;                            %determines what time interval beween measurements for example, TIME_STEP = 0.5 gives 0, 0.5, 1, 1.5, ...
NUM_SEC   = 100;
parseObj = dataparse(DATASET, TIME_STEP, NUM_SEC);

% Landmark data
landmark = parseObj.parse_landmark();
landmark= landmark(:,2:3);
BC = parseObj.parse_barcode();
landmark_codes  = containers.Map( BC(6:end,2)',BC(6:end,1)'); %maps barcode id to landmark



%Robot data
X_ground_truth          = zeros(3*Num, parseObj.time_length());
action_for_robots       = zeros(3*Num, parseObj.time_length());
measurement_z_landmark  = zeros(3*Num, parseObj.time_length());

SCALE = 10;
for rob_num = 1:Num
    [GT,OD,MS] = parseObj.parse_robot(rob_num, landmark_codes);
    X_ground_truth        (3*rob_num -2 : 3*rob_num, :)   = SCALE *GT(:,2:4)';
    action_for_robots     (3*rob_num -2 : 3*rob_num-1, :) = OD(:,2:3)';
    action_for_robots     (3*rob_num -2 , :) =  SCALE * action_for_robots     (3*rob_num -2 , :);
    
    % Landmark measurements. We flip range and bearing
    measurement_z_landmark(3*rob_num -2 : 3*rob_num, :)   = MS(:,2:4)';
    measurement_z_landmark_buff = SCALE * measurement_z_landmark(3*rob_num-1,:);
    measurement_z_landmark(3*rob_num-1,:) = measurement_z_landmark(3*rob_num,:);
    measurement_z_landmark(3*rob_num,:) = measurement_z_landmark_buff;



    measurement_z_landmark_buff = measurement_z_landmark(3*rob_num-1,:);
    measurement_z_landmark(3*rob_num-1,:) = measurement_z_landmark(3*rob_num,:);
    measurement_z_landmark(3*rob_num,:) = measurement_z_landmark_buff;
    
end


%Robot Relative data. Set 'var' for messy measurements
var                      = 0;
measurement_z_relative   = zeros(2*Num*(Num-1), parseObj.time_length());

jj = 1;
for i = 1:Num
    for j = 1:Num
        if i~=j
            rb_i = X_ground_truth(3*i - 2: 3*i, :)';
            rb_j = X_ground_truth(3*j - 2: 3*j, :)';
            measurement_z_relative(jj:jj+1, :) = relative_measurement_generation(rb_i, rb_j, var);
            jj = jj+2;
        end
    end
end
%Number of steps
numSteps = parseObj.time_length();
end

function measurement_rel = relative_measurement_generation(rb_j,rb_i,var)
  delta_x = rb_j(:,1) - rb_i(:,1);
  delta_y = rb_j(:,2) - rb_i(:,2);
  bearing = wrapToPi(atan2(delta_y, delta_x)) - rb_i(:,3);
  range = sqrt(delta_x.^2 + delta_y.^2);
  measurement_rel = [bearing';range'];
end





end

function measurement_rel = relative_measurement_generation(rb_j,rb_i,var)
  delta_x = rb_j(:,1) - rb_i(:,1);
  delta_y = rb_j(:,2) - rb_i(:,2);
  bearing = wrapToPi(atan2(delta_y, delta_x)) - rb_i(:,3);
  range = sqrt(delta_x.^2 + delta_y.^2);
  measurement_rel = [bearing';range'];
end


function [bearing,range] = Landmark_measurement_generation(landmark,pos)
    landmark_x = landmark(1,1);
    landmark_y = landmark(2,1);
    bearing = ...
        wrapToPi(atan2(landmark_y - pos(2), landmark_x - pos(1)) - pos(3));
    range  = ...
        sqrt((landmark_y - pos(2))^2 + (landmark_x - pos(1))^2);
end
