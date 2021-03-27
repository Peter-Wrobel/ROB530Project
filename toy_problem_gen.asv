function [X_ground_truth,measurement_z,action_for_robots] = toy_problem_gen(deltaT,numSteps)

%% The following parameters will be supplied as inputs later

  Num = 2; % number of robots
% Motion noise (see HW5)
  alphas = [0.00025 0.00005 ...
            0.0025 0.0005 ...
            0.0025 0.0005].^2; 
  
% Stipulate the inputs for robot1 (current version is noiseless)
  action_for_robot1 = (-1) .* ones(3,numSteps-1);
  action_for_robot1(2:3,:) = zeros(2,numSteps-1); 
  noise_v = alphas(1).*action_for_robot1(1,:).^2 + alphas(2).*action_for_robot1(2,:).^2 ...
            .* rand(1,numSteps-1);
  noise_omega = alphas(3).*action_for_robot1(1,:).^2 + alphas(4).*action_for_robot1(2,:).^2 ...
            .* rand(1,numSteps-1);
  noise_gamma = alphas(5).*action_for_robot1(1,:).^2 + alphas(6).*action_for_robot1(2,:).^2 ...
            .* rand(1,numSteps-1);
  action_for_robot1(1,:) = action_for_robot1(1,:) + noise_v;
  action_for_robot1(2,:) = action_for_robot1(2,:) + noise_omega;
  action_for_robot1(3,:) = action_for_robot1(3,:) + noise_gamma;
  
  
% Stipulate the inputs for robot2
  action_for_robot2 = ones(3,numSteps-1);
  action_for_robot2(2:3,:) = zeros(2,numSteps-1); 
  noise_v = alphas(1).*action_for_robot2(1,:).^2 + alphas(2).*action_for_robot2(2,:).^2 ...
            .* rand(1,numSteps-1);
  noise_omega = alphas(3).*action_for_robot2(1,:).^2 + alphas(4).*action_for_robot2(2,:).^2 ...
            .* rand(1,numSteps-1);
  noise_gamma = alphas(5).*action_for_robot2(1,:).^2 + alphas(6).*action_for_robot2(2,:).^2 ...
            .* rand(1,numSteps-1);
  action_for_robot2(1,:) = action_for_robot2(1,:) + noise_v;
  action_for_robot2(2,:) = action_for_robot2(2,:) + noise_omega;
  action_for_robot2(3,:) = action_for_robot2(3,:) + noise_gamma; 
  
  action_for_robots = NaN .* ones(3*Num,numSteps-1);
  action_for_robots(1:3,:) = action_for_robot1;
  action_for_robots(4:6,:) = action_for_robot2;
  
%% Generate ground truth value and measurements    
% Generate the ground truth pos of robot1
  ground_truthROB1 = zeros(3,numSteps);
% Initialize the postion of the robot1 at t = 0
  ground_truthROB1(:,1) = [0.5 3 pi]'; 
  for i = 2:numSteps
      ground_truthROB1(:,i) = ...
               movement(ground_truthROB1(:,i-1),action_for_robot1(:,i-1),deltaT);
  end

% Generate the ground truth pos of robot2
  ground_truthROB2 = zeros(3,numSteps);
% Initialize the postion of the robot2 at t = 0
  ground_truthROB2(:,1) = [-0.5 3 pi]'; 
  for i = 2:numSteps
      ground_truthROB2(:,i) = ...
               movement(ground_truthROB2(:,i-1),action_for_robot2(:,i-1),deltaT);
  end  

% Generate the ground truth pos X_ground_truth following the same structure
% specified in Agostino et al. 2005
  X_ground_truth = NaN .* ones(3*Num,numSteps);
  X_ground_truth(1:3,:) = ground_truthROB1;
  X_ground_truth(4:6,:) = ground_truthROB2;
  
%% Current measurement function only works for two robots
% Generate the measurement matrix z whose size is 2*Num*(Num-1) X stepsOrData-1
  measurement_z = NaN .* zeros(2*Num*(Num-1),numSteps);
  for k = 1 : numSteps
    for i = 1 : Num
      jj = 0;
      for j = 1 : Num
         % jj is used to record to serial num of observation for the same
         % robot i
          if j ~= i
            jj = jj + 1;  
            [bearing,range] =...
            measurement_generation(X_ground_truth(3*(i-1)+1:3*i,k),...
                                   X_ground_truth(3*(j-1)+1:3*j,k));
                             
            measurement_z(2*(Num-1)*(i-1) + 2*(jj-1)+1,k) = bearing;
            measurement_z(2*(Num-1)*(i-1) + 2*jj,k) = range;
          end
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
  delta_y = pos_j(2) - pos_i(2);
  bearing = wrapToPi(atan((-sin(pos_i(3))*delta_x+cos(pos_i(3))*delta_y)...
                         /( cos(pos_i(3))*delta_x+sin(pos_i(3))*delta_y)));
  range = sqrt(delta_x^2 + delta_y^2);
end