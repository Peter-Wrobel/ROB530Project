function toy_problem_gen

%% The following parameters will be supplied as inputs later
  deltaT = 0.1;
  stepsOrData = 50;
  Num = 2; % number of robots
  
% Stipulate the inputs for robot1 (current version is noiseless)
  action_for_robot1 = ones(3,stepsOrData-1);
  action_for_robot1(3,:) = zeros(1,stepsOrData-1); 
  
% Stipulate the inputs for robot2 (current version is noiseless)
  action_for_robot2 = ones(3,stepsOrData-1);
  action_for_robot2(3,:) = zeros(1,stepsOrData-1); 
  
%% Generate ground truth value and measurements    
% Generate the ground truth pos of robot1
  ground_truthROB1 = zeros(3,stepsOrData);
% Initialize the postion of the robot1 at t = 0
  ground_truthROB1(:,1) = [-2 3 pi]'; 
  for i = 2:stepsOrData
      ground_truthROB1(:,i) = ...
               movement(ground_truthROB1(:,i-1),action_for_robot1(:,i-1),deltaT);
  end

% Generate the ground truth pos of robot2
  ground_truthROB2 = zeros(3,stepsOrData);
% Initialize the postion of the robot2 at t = 0
  ground_truthROB2(:,1) = [2 3 pi]'; 
  for i = 2:stepsOrData
      ground_truthROB2(:,i) = ...
               movement(ground_truthROB2(:,i-1),action_for_robot2(:,i-1),deltaT);
  end  

%% Current measurement function only works for two robots
% Generate the measurement matrix z whose size is 2*Num*(Num-1) X stepsOrData-1
  measurement_z = NaN .* zeros(2*Num*(Num-1),stepsOrData-1);
  for k = 2 : stepsOrData
    for kk = 1 : Num
      for kkk = 1 : Num - 1
          [bearing,range] =...
           measurement_generation(ground_truthROB1(:,k),ground_truthROB2(:,k));
       
          measurement_z(2*(Num-1)*(kk-1) + 2*(kkk-1)+1,k) = bearing;
          measurement_z(2*(Num-1)*(kk-1) + 2*kkk,k) = range;
      end
    end
  end
  
%% Viusalize the trajectory of the group of robots
  plot(ground_truthROB1(1,:),ground_truthROB1(2,:),'b*','markersize',3);
  hold on
  plot(ground_truthROB2(1,:),ground_truthROB2(2,:),'rs','markersize',3);
  axis equal;
end

%% Motion of Robot
function [state_after] = movement(state_before,action,deltaT)
  state_after = NaN * zeros(size(state_before));
% We follow the motion model illustrated in HW5
% It should be noted that the noise have been included in the action term
  rate = (action(1,1)/action(2,1));
  if rate ~= 0
    state_after(1,1) = state_before(1,1) - rate*sin(state_before(3,1)) ...
                       + rate*sin(state_before(3,1)+action(2,1)*deltaT);
    state_after(2,1) = state_before(2,1) + rate*cos(state_before(3,1)) ...
                       - rate*cos(state_before(3,1)+action(2,1)*deltaT);
    state_after(3,1) = state_before(3,1) + action(2,1)*deltaT ...
                       + action(3,1)*deltaT;
  else
    state_after(1,1) = state_after(1,1);
    state_after(2,1) = state_after(2,1);
    state_after(3,1) = state_after(3,1);
  end 
end

%% Relative Measurement Model
% The following function generate relative bearing and range measurements
% based on the eq.(11) and (12) in Agostino et al. 2005
function [bearing,range] = measurement_generation(pos_i,pos_j)
  delta_x = pos_j(1) - pos_i(1);
  delta_y = pos_j(2) - pos_j(1);
  bearing = atan((-sin(pos_i(3))*delta_x+cos(pos_i(3))*delta_y)...
                /( cos(pos_i(3))*delta_x+sin(pos_i(3))*delta_y));
  range = sqrt(delta_x^2 + delta_y^2);
end