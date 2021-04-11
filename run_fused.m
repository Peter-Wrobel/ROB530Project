function run_fused(numSteps, filter_name, if_toy_prob)
cla;
%%   run_fused(stepsOrData, filter_name, if_toy_prob)
%      stepsOrData - is the number of time steps
%      filter_name - filter type, 'EKF' or 'PF'
%      if_toy_prob - determine which data set to use

%% Initializations
  addpath([cd, filesep, 'lib'])
  if if_toy_prob
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
   % generate the data set for the toy problem
     % [X_ground_truth,measurement_z,action_for_robots] = toy_problem_gen(deltaT,numSteps);
     [X_ground_truth,landmark,landmark_measurement_z,relative_measurement_z,action_for_robots] = toy_problem_gen(deltaT,numSteps, Num);
     
     initialStateMean = X_ground_truth(:,1);
     initialStateCov = eye(3*Num);
     
  else
   % Number of robots
     Num = 5;
   % Motion noise (in odometry space, see Table 5.5, p.134 in book).
   % variance of noise proportional to alphas
     alphas = [0.00025 0.00005 ...
               0.0025 0.0005 ...
               0.0025 0.0005].^2;   
     deltaT = 1;
   % Standard deviation of Gaussian sensor noise (independent of distance)
     beta = deg2rad(5);
   % generate the data set for the toy problem
     % [X_ground_truth,measurement_z,action_for_robots] = toy_problem_gen(deltaT,numSteps);
     [X_ground_truth,landmark,landmark_measurement_z,relative_measurement_z,action_for_robots, numSteps] = real_problem_gen(deltaT, Num);
     
     initialStateMean = X_ground_truth(:,1);
     initialStateCov = eye(3*Num);
      
  end

  sys = system_initialization(alphas, beta, deltaT);
  filter = filter_initialization(sys, initialStateMean, initialStateCov, filter_name,Num);
  
  % Record the filtered positions of the robot group
  filtered_robot1 = NaN .* zeros(3,numSteps-1);
  filtered_robot2 = NaN .* zeros(3,numSteps-1);
  filtered_robot3 = NaN .* zeros(3,numSteps-1);
for t = 1 : numSteps-1
    %=================================================
    % data available to your filter at this time step
    %=================================================
   
    % [Trans_vel,Angular_vel,gamma]' noisy control command
    motionCommand = action_for_robots(:,t); 
    % [relative bearing, relative range] (noisy observation)
    % 2*Num*(Num-1) * 1
    
    observation = relative_measurement_z(:,t+1);
    landmark_measurement_z_t = landmark_measurement_z(:,t+1);
    
    X_ground_truth_t = X_ground_truth(:, t+1);
    
    %=================================================
    %TODO: update your filter here based upon the
    %      motionCommand and observation
    %=================================================

      
    switch filter_name
        case {"EKF"}
           
            % Avoid zero division
            motionCommand = motionCommand + 0.00001.*randn(size(motionCommand));
           
            filter.prediction(motionCommand); 
            
            if if_toy_prob
                filter.correction_landmark(landmark_measurement_z_t, landmark);


           %elseif landmark_measurement_z_t %TODO MEASURMENT IS NOW 3*4, NOT 3*3
                                            %BECAUSE NOW EACH ROBOT HAS
                                            %LANDMARK ID PUSHED TOO. BE
                                            %AWARE OF -1 LANDMARK ID
              %landmark_measurement_z_t 
              %filter.correction_landmark(landmark_measurement_z_t, landmark);

           end

           filter.mu_pred = filter.mu;
           filter.Sigma_pred = filter.Sigma;
           filter.correction_relative(observation);
           
           %draw_ellipse(filter.mu(1:2), filter.Sigma(1:2,1:2),9)
    end
         filtered_robot1(:,t) = filter.mu(1:3,1);
         filtered_robot2(:,t) = filter.mu(4:6,1);
         filtered_robot3(:,t) = filter.mu(7:9,1);
end

%% Visualization
  p1 = plot(X_ground_truth(1,:),X_ground_truth(2,:),'b*','markersize',5);
  hold on
  p2 = plot(X_ground_truth(4,:),X_ground_truth(5,:),'bs','markersize',5);
  hold on
  p3 = plot(X_ground_truth(7,:),X_ground_truth(8,:),'b+','markersize',5);
  hold on
  p4 = plot(filtered_robot1(1,:),filtered_robot1(2,:),'r*','markersize',5);
  hold on
  p5 = plot(filtered_robot2(1,:),filtered_robot2(2,:),'rs','markersize',5);
  hold on
  p6 = plot(filtered_robot3(1,:),filtered_robot3(2,:),'r+','markersize',5);
  axis equal;
  legend([p1, p2, p3, p4, p5, p6],...
         {'Goundtruth for Robot1','Goundtruth for Robot2','Goundtruth for Robot3',...
          'Estimation for Robot1','Estimation for Robot2','Estimation for Robot3'});
%   title('EKF with Relative Measurements Fused - Relative only')
%   title('EKF with Relative Measurements Fused - Landmark only')
%   title('EKF with Relative Measurements Fused - landmark relative')
%   title('EKF with Relative Measurements Fused - relative landmark')
%  title('EKF with Relative Measurements Fused - Landmark correction no noise groundtruth as prediction')
% Stop for debugging
  stop = 0;
end
