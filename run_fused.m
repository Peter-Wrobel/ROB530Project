function varargout = run_fused(numSteps, filter_name, if_toy_prob)

%%   run_fused(stepsOrData, filter_name, if_toy_prob)
%      stepsOrData - is the number of time steps
%      filter_name - filter type, 'EKF' or 'PF'
%      if_toy_prob - determine which data set to use

%% Initializations
  addpath([cd, filesep, 'lib'])
  if if_toy_prob
   % Number of robots
     Num = 2;
   % Motion noise (in odometry space, see Table 5.5, p.134 in book).
   % variance of noise proportional to alphas
     alphas = [0.00025 0.00005 ...
               0.0025 0.0005 ...
               0.0025 0.0005].^2;   
     deltaT = 0.1;
   % Standard deviation of Gaussian sensor noise (independent of distance)
     beta = deg2rad(5);
   % generate the data set for the toy problem
     [X_ground_truth,measurement_z,action_for_robots] = toy_problem_gen(deltaT,numSteps);
     
     initialStateMean = X_ground_truth(:,1);
     initialStateCov = eye(3*Num);
  end

  sys = system_initialization(alphas, beta);
  filter = filter_initialization(sys, initialStateMean, initialStateCov, filter_name,Num);

for t = 1 : numSteps
    %=================================================
    % data available to your filter at this time step
    %=================================================
    % [Trans_vel,Angular_vel,gamma]' noisy control command
    motionCommand = action_for_robots(:,t); 
    % [relative bearing, relative range] (noisy observation)
    % 2*Num*(Num-1) * 1
    observation = measurement_z(:,t);
    
    %=================================================
    %TODO: update your filter here based upon the
    %      motionCommand and observation
    %=================================================
    
    switch filter_name
        case {"EKF"}
           filter.prediction(motionCommand);
           filter.correction(observation);
%            draw_ellipse(filter.mu(1:2), filter.Sigma(1:2,1:2),9)
    end
end
