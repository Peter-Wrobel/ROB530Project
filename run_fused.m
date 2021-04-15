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
     Existing_Data = 0;
     if ~Existing_Data
     % generate the data set for the toy problem
       [X_ground_truth,landmark,measurement_z_landmark,...
           measurement_z_relative,action_for_robots] = toy_problem_gen(deltaT,numSteps);
     else
       load('data_all_ss.mat');
     end
  % Employ the real data set   
  else
      % Employ the real data set
     [X_ground_truth,landmark,measurement_z_landmark,...
         measurement_z_relative,action_for_robots, numSteps] = real_problem_gen(deltaT, Num);
  end
     action_for_robots = action_for_robots + 0.0001.*randn(size(action_for_robots));
  %% Initialization Based on Different Filter Selection
  sys = system_initialization(alphas, beta, deltaT);
  switch filter_name
      case {"EKF"}
          % Batch Update
          initialStateMean = X_ground_truth(:,1);
          initialStateCov = eye(3*Num);    
          filter = ...
            filter_initialization(sys, initialStateMean, initialStateCov, filter_name,Num);

        filtered_robot = NaN .* zeros(9,numSteps-1);
        % Initialize filters for different robots
        initialStateMean1 = X_ground_truth(1:3,1);
        initialStateMean2 = X_ground_truth(4:6,1);
        initialStateMean3 = X_ground_truth(7:9,1);
        initialStateCov = eye(3);
        filter1 = filter_initialization(sys, initialStateMean1, initialStateCov, filter_name,1);
        filter2 = filter_initialization(sys, initialStateMean2, initialStateCov, filter_name,1);
        filter3 = filter_initialization(sys, initialStateMean3, initialStateCov, filter_name,1);

    
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

    switch filter_name
        case {"EKF"}

%            % Batch Update - Robot Dependent of each other in relative
%            % correction step
%            filter.prediction(motionCommand);
%            filter.correction_landmark(measurement_z_landmark(:,t+1), landmark);
%            filter.mu_pred = filter.mu;
%            filter.Sigma_pred = filter.Sigma;
%            % filter.correction_relative_batch_all(measurement_z_relative(:, t+1));
%            filter.correction_relative_single_all(measurement_z_relative(:, t+1));
%            filtered_robot(:,t) = filter.mu(:,1);
%            % Draw uncertainty           
%             draw_ellipse(filter.mu(1:2), filter.Sigma(1:2,1:2),9)
%             hold on
%             draw_ellipse(filter.mu(4:5), filter.Sigma(4:5,4:5),9)
%             hold on
%             draw_ellipse(filter.mu(7:8), filter.Sigma(7:8,7:8),9)
%             hold on


           % Individual Update - Robot Independent of each other
           filter1.prediction(motionCommand(1:3,1));
           filter2.prediction(motionCommand(4:6,1));
           filter3.prediction(motionCommand(7:9,1));
           
           filter1.mu = filter1.mu_pred;
           filter2.mu = filter2.mu_pred;
           filter3.mu = filter3.mu_pred;
           filter1.Sigma = filter1.Sigma_pred;
           filter2.Sigma = filter2.Sigma_pred;
           filter3.Sigma = filter3.Sigma_pred;
           
           % Correction with Landmark Measurements
           if ~if_toy_prob
              
              % Measurement Correction
              if measurement_z_landmark(1,t+1) ~= -1
                     filter1.correction_landmark_single...
                         (measurement_z_landmark(2:3,t+1),landmark(measurement_z_landmark(1,t+1),:));
                     hold on;
                     plot(landmark(measurement_z_landmark(1,t+1),1), landmark(measurement_z_landmark(1,t+1),2), 'kp');
              end
              if measurement_z_landmark(4,t+1) ~= -1
                     filter2.correction_landmark_single...
                         (measurement_z_landmark(5:6,t+1),landmark(measurement_z_landmark(4,t+1),:));
                     plot(landmark(measurement_z_landmark(4,t+1),1), landmark(measurement_z_landmark(4,t+1),2), 'cp');
              end
              if measurement_z_landmark(7,t+1) ~= -1
                     filter3.correction_landmark_single...
                         (measurement_z_landmark(8:9,t+1),landmark(measurement_z_landmark(7,t+1),:));
                     plot(landmark(measurement_z_landmark(7,t+1),1), landmark(measurement_z_landmark(7,t+1),2), 'mp'); % c, m
              end
              
%               % Relative Correction
%                filter1.mu_pred = filter1.mu;
               filter2.mu_pred = filter2.mu;
%                filter3.mu_pred = filter3.mu;
%                filter1.Sigma_pred = filter1.Sigma;
               filter2.Sigma_pred = filter2.Sigma;
%                filter3.Sigma_pred = filter3.Sigma;

%                % Recursive Correction with Relative Measurements
                relative_pos1 = [filter2.mu(:,1);filter3.mu(:,1)];
                relative_pos2 = [filter1.mu(:,1);filter3.mu(:,1)];
                relative_pos3 = [filter1.mu(:,1);filter2.mu(:,1)];
%                 filter1.correction_relative_single(measurement_z_relative(1:4,t+1),relative_pos1,Num);
                filter2.correction_relative_single(measurement_z_relative(5:8,t+1),relative_pos2,Num);
%                 filter3.correction_relative_single(measurement_z_relative(9:12,t+1),relative_pos3,Num);
               
           else
               % Recursive Correction with Landmark Measurements 
               filter1.correction_landmark_single(measurement_z_landmark(1:2,t+1), landmark);
               filter2.correction_landmark_single(measurement_z_landmark(3:4,t+1), landmark);
               filter3.correction_landmark_single(measurement_z_landmark(5:6,t+1), landmark);

               % Update for iterative prediction
               filter1.mu_pred = filter1.mu;
               filter2.mu_pred = filter2.mu;
               filter3.mu_pred = filter3.mu;
               filter1.Sigma_pred = filter1.Sigma;
               filter2.Sigma_pred = filter2.Sigma;
               filter3.Sigma_pred = filter3.Sigma;
               % Recursive Correction with Relative Measurements
                relative_pos1 = [filter2.mu(:,1);filter3.mu(:,1)];
                relative_pos2 = [filter1.mu(:,1);filter3.mu(:,1)];
                relative_pos3 = [filter1.mu(:,1);filter2.mu(:,1)];
                filter1.correction_relative_single(measurement_z_relative(1:4,t+1),relative_pos1,Num);
                filter2.correction_relative_single(measurement_z_relative(5:8,t+1),relative_pos2,Num);
                filter3.correction_relative_single(measurement_z_relative(9:12,t+1),relative_pos3,Num);
           end
           filtered_robot(1:3,t) = filter1.mu(:,1);
           filtered_robot(4:6,t) = filter2.mu(:,1);
           filtered_robot(7:9,t) = filter3.mu(:,1);
           
         if t > 99
             % Draw uncertainty
             hold on
             draw_ellipse(filter1.mu(1:2), filter1.Sigma(1:2,1:2),9)
             hold on
             draw_ellipse(filter2.mu(1:2), filter2.Sigma(1:2,1:2),9)
             hold on
             draw_ellipse(filter3.mu(1:2), filter3.Sigma(1:2,1:2),9)
             drawnow
         end
             
       case {"PF"}
           % Peform PF for each robot
             filter1.prediction(motionCommand(1:3,1));
             filter2.prediction(motionCommand(4:6,1));
             filter3.prediction(motionCommand(7:9,1));
            
           if ~if_toy_prob
               
           % Correction with Landmark Measurements
           
              %{
             if measurement_z_landmark(1,t+1) ~=-1
                 measurement_z_landmark(2,t+1) = measurement_z_landmark(2,t+1);
                 hold on;
                 %[x,y] = pol2cart(measurement_z_landmark(2,t+1),measurement_z_landmark(3,t+1));
                 %plot([landmark(measurement_z_landmark(1,t+1), 1)-x,landmark(measurement_z_landmark(1,t+1), 1)],...
                            %[landmark(measurement_z_landmark(1,t+1), 2)-y,landmark(measurement_z_landmark(1,t+1), 2)], '-g');

                   filter1.correction_landmark...
                        (measurement_z_landmark(2:3,t+1),landmark(measurement_z_landmark(1,t+1), :)' );
             end
           %}
             
             if measurement_z_landmark(4,t+1) ~= -1
                 hold on;
                 %[x,y] = pol2cart(measurement_z_landmark(5,t+1),measurement_z_landmark(6,t+1));
                 %plot([landmark(measurement_z_landmark(4,t+1), 1)-x,landmark(measurement_z_landmark(4,t+1), 1)],...
                            %[landmark(measurement_z_landmark(4,t+1), 2)-y,landmark(measurement_z_landmark(4,t+1), 2)], '-b');

                    filter2.correction_landmark...
                        (measurement_z_landmark(5:6,t+1),landmark(measurement_z_landmark(4,t+1), :)' );
             end
             
             

             if measurement_z_landmark(7,t+1) ~= -1
                    filter3.correction_landmark...
                        (measurement_z_landmark(8:9,t+1),landmark(measurement_z_landmark(7,t+1), :)' );
             end
           %%}
                
           else
               hold on;
              [x,y] = pol2cart(measurement_z_landmark(1,t+1),measurement_z_landmark(2,t+1));
               plot([landmark(1)-x,landmark(1)],...
                            [landmark(2)-y,landmark(2)]);
              %filter1.correction_landmark(measurement_z_landmark(1:2,t+1),landmark);
              %filter2.correction_landmark(measurement_z_landmark(3:4,t+1),landmark);
              %filter3.correction_landmark(measurement_z_landmark(5:6,t+1),landmark);
           end
           
            if t>0
           % Recursive Correction with Relative Measurements
             relative_pos1 = [filter2.mu(:,1);filter3.mu(:,1)];
             relative_pos2 = [filter1.mu(:,1);filter3.mu(:,1)];
             relative_pos3 = [filter1.mu(:,1);filter2.mu(:,1)];
             filter1.correction_relative(measurement_z_relative(1:4,t+1),relative_pos1,Num);
             %filter2.correction_relative(measurement_z_relative(5:8,t+1),relative_pos2,Num);
             %filter3.correction_relative(measurement_z_relative(9:12,t+1),relative_pos3,Num);
             end
             
           
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
%              drawnow
        end
     


%      cla
  hold on
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
  drawnow
end

%% Visualization
%   cla
hold on

  title('PF with Relative Measurements Fused')
for t = 1:numSteps-1
  p1 = plot(X_ground_truth(1,t),X_ground_truth(2,t),'b*','markersize',5);
  hold on
  p2 = plot(X_ground_truth(4,t),X_ground_truth(5,t),'bs','markersize',5);
  hold on
  p3 = plot(X_ground_truth(7,t),X_ground_truth(8,t),'bd','markersize',5);
  hold on
  p4 = plot(filtered_robot(1,t),filtered_robot(2,t),'r*','markersize',5);
  hold on
  p5 = plot(filtered_robot(4,t),filtered_robot(5,t),'rs','markersize',5);
  hold on
  p6 = plot(filtered_robot(7,t),filtered_robot(8,t),'rd','markersize',5);
  axis equal;
  pause(0.05);
  hold on
  
  set(gcf,'Position',[0 0 1000 800]);
  
%   plot(filter.particles(1,:),filter.particles(2,:),'go');
  legend([p1,p2,p3,p4,p5,p6],...
         {'Goundtruth for Robot1','Goundtruth for Robot2','Goundtruth for Robot3',...
          'Estimation for Robot1','Estimation for Robot2','Estimation for Robot3'...
          });
%   title('PF with Relative Measurements Fused')
%   title('EKF - Independent Update, all fused, 0.01*Q, (Toy Data)')

%     title('EKF - Landmark Measurements Only (Toy Data)')
%     title('EKF - Relative Measurements Only (Toy Data)')
%     title('EKF - Fused Measurements (Toy Data)')
    
%     title('EKF - Relative Measurements Only (Real Data)')
%     title('EKF - Relative Measurements Only (Real Data)')
    title('EKF - Fused Measurements (Real Data)')
    
% Stop for debugging
  stop = 0;
end
