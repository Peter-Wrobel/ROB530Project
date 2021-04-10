function run_fused(numSteps, filter_name, if_toy_prob)

%      stepsOrData - is the number of time steps
%      filter_name - filter type, 'EKF' or 'PF'
%      if_toy_prob - determine which data set to use

%% Initializations
  addpath([cd, filesep, 'lib'])
  % Number of robots
    Num = 3;
  % Motion noise (in odometry space, see Table 5.5, p.134 in book).
  % variance of noise proportional to alphas
    alphas = [0.00025 0.00005 ...
              0.0025 0.0005 ...
              0.0025 0.0005].^2;   
    deltaT = 1;
  % Standard deviation of Gaussian sensor noise (independent of distance)
    beta = deg2rad(5);
    
 %% Load Data   
  if if_toy_prob
   % Existing_Data dictates whether to generate new toy data
     Existing_Data = 1;
     if ~Existing_Data
     % generate the data set for the toy problem
       [X_ground_truth,landmark,measurement_z_landmark,...
           measurement_z_relative,action_for_robots] = toy_problem_gen(deltaT,numSteps);
     else
       load('data_all.mat');
     end
  % Employ the real data set   
  else
      % Employ the real data set
     [X_ground_truth,landmark,measurement_z_landmark,...
         measurement_z_relative,action_for_robots, numSteps] = real_problem_gen(deltaT, Num);
     action_for_robots = action_for_robots + 0.0001.*randn(size(action_for_robots));
  end

  %% Initialization Based on Different Filter Selection
  sys = system_initialization(alphas, beta, deltaT);
  switch filter_name
      case {"EKF"}
      initialStateMean = X_ground_truth(:,1);
      initialStateCov = eye(3*Num);    
      filter = ...
        filter_initialization(sys, initialStateMean, initialStateCov, filter_name,Num);
      case {"PF"}
      % Record the filtered positions of the robot group
        filtered_robot1 = NaN .* zeros(3,numSteps-1);
        filtered_robot2 = NaN .* zeros(3,numSteps-1);
        filtered_robot3 = NaN .* zeros(3,numSteps-1);
        filtered_robot = NaN .* zeros(9,numSteps-1);
  
       % Initialize filters for different robots
         initialStateMean1 = X_ground_truth(1:3,1);
         initialStateMean2 = X_ground_truth(4:6,1);
         initialStateMean3 = X_ground_truth(7:9,1);
         initialStateCov = eye(3);
         filter1 = filter_initialization(sys, initialStateMean1, initialStateCov, filter_name,1);
         filter2 = filter_initialization(sys, initialStateMean2, initialStateCov, filter_name,1);
         filter3 = filter_initialization(sys, initialStateMean3, initialStateCov, filter_name,1);
  end
  
  %% Main Loop
  cla;
for t = 1 : numSteps-1
   
    motionCommand = action_for_robots(:,t); 
    % [relative bearing, relative range] (noisy observation)
    % 2*Num*(Num-1) * 1
    observation = measurement_z_landmark(:,t+1);
      
    switch filter_name
        case {"EKF"}
           filter.prediction(motionCommand);
           filter.mu = filter.mu_pred;
%            filter.correction_relative(observation);
%            filter.correction_landmark(observation);
%            filter.correction_batch(observation, observation);
%          draw_ellipse(filter.mu(1:2), filter.Sigma(1:2,1:2),9)
           filtered_robot1(:,t) = filter.mu(1:3,1);
           filtered_robot2(:,t) = filter.mu(4:6,1);
           filtered_robot3(:,t) = filter.mu(7:9,1);
       case {"PF"}
           % Peform PF for each robot
             filter1.prediction(motionCommand(1:3,1));
             filter2.prediction(motionCommand(4:6,1));
             filter3.prediction(motionCommand(7:9,1));
            
           % Correction with Landmark Measurements
            measurement_z_landmark_buff = measurement_z_landmark(2,:);
            measurement_z_landmark(2,:) = measurement_z_landmark(3,:);
            measurement_z_landmark(3,:) = measurement_z_landmark_buff;
%             if measurement_z_landmark(1,t+1) ~= -1
%                    filter1.correction_landmark...
%                        (measurement_z_landmark(2:3,t+1),landmark(measurement_z_landmark(1,t+1),:));
%             end
                
              
%              filter1.correction_landmark(measurement_z_landmark(1:2,t+1),landmark);
%              filter2.correction_landmark(measurement_z_landmark(3:4,t+1),landmark);
%              filter3.correction_landmark(measurement_z_landmark(5:6,t+1),landmark);
             
           % Recursive Correction with Relative Measurements
             relative_pos1 = [filter2.mu(:,1);filter3.mu(:,1)];
             relative_pos2 = [filter1.mu(:,1);filter3.mu(:,1)];
             relative_pos3 = [filter1.mu(:,1);filter2.mu(:,1)];
%              filter1.correction_relative(measurement_z_relative(1:4,t+1),relative_pos1,Num);
%              filter2.correction_relative(measurement_z_relative(5:8,t+1),relative_pos2,Num);
%              filter3.correction_relative(measurement_z_relative(9:12,t+1),relative_pos3,Num);
           
           % Record the estimated postions
             filtered_robot(1:3,t) = filter1.mu(:,1);
             filtered_robot(4:6,t) = filter2.mu(:,1);
             filtered_robot(7:9,t) = filter3.mu(:,1);
             
%              hold on
%              draw_ellipse(filter1.mu(1:2), filter1.Sigma(1:2,1:2),9)
%              hold on
%              draw_ellipse(filter2.mu(1:2), filter2.Sigma(1:2,1:2),9)
%              hold on
%              draw_ellipse(filter3.mu(1:2), filter3.Sigma(1:2,1:2),9)


  cla
  p1 = plot(X_ground_truth(1,:),X_ground_truth(2,:),'b*','markersize',5);
  hold on
  p2 = plot(X_ground_truth(4,:),X_ground_truth(5,:),'bs','markersize',5);
  hold on
  p3 = plot(X_ground_truth(7,:),X_ground_truth(8,:),'bd','markersize',5);
  hold on
  p4 = plot(filtered_robot(1,:),filtered_robot(2,:),'r*','markersize',20);
  hold on
  p5 = plot(filtered_robot(4,:),filtered_robot(5,:),'rs','markersize',5);
  hold on
  p6 = plot(filtered_robot(7,:),filtered_robot(8,:),'rd','markersize',5);
  axis equal;
    end
end

%% Visualization
%   cla
  p1 = plot(X_ground_truth(1,:),X_ground_truth(2,:),'b*','markersize',5);
  hold on
  p2 = plot(X_ground_truth(4,:),X_ground_truth(5,:),'bs','markersize',5);
  hold on
  p3 = plot(X_ground_truth(7,:),X_ground_truth(8,:),'bd','markersize',5);
  hold on
  p4 = plot(filtered_robot(1,:),filtered_robot(2,:),'r*','markersize',5);
  hold on
  p5 = plot(filtered_robot(4,:),filtered_robot(5,:),'rs','markersize',5);
  hold on
  p6 = plot(filtered_robot(7,:),filtered_robot(8,:),'rd','markersize',5);
  axis equal;
  hold on
%   plot(filter.particles(1,:),filter.particles(2,:),'go');
  legend([p1,p2,p3,p4,p5,p6],...
         {'Goundtruth for Robot1','Goundtruth for Robot2','Goundtruth for Robot3',...
          'Estimation for Robot1','Estimation for Robot2','Estimation for Robot3'...
          });
  title('PF with Relative Measurements Fused')
% Stop for debugging
  stop = 0;
end
