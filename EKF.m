classdef EKF < handle
    properties
        Num             % Number of Robots
        
        mu;             % Pose Mean
        Sigma;          % Pose Covariance
        mu_pred;        % Predictied Mean after prediction step
        Sigma_pred;     % Predictied Covarince after prediction step
        
        gfun;           % Discrete Dynamical equations of motion
        Gfun;           % Motion Model Jacobian with respect to state
        Vfun;           % Motion Model Jacobian with respect to control inputs
        
        hfun_relative;  % Relative Measurement model equations
        hfun_landmark;  % Measurement model involves landmark
        Hfun;           % Measruement Model Jacobian
        
        M;              % Noise Covariance in input measruements
        Q;              % Sensor Noise Covariance

    end
    
    methods
        function obj = EKF(sys, init)
            % Identify the number of robots
              obj.Num = init.Num;
             
            % motion model
              obj.gfun = sys.gfun;
             
            % measurement model
              obj.hfun_relative = sys.hfun_relative;
              obj.hfun_landmark = sys.hfun_landmark;
            
            % Jocabian of motion model
              obj.Gfun = init.Gfun;
              obj.Vfun = init.Vfun;
              
            % Jacobian of measurement model
              obj.Hfun = init.Hfun;

            % motion noise covariance
              obj.M = sys.M;
            
            % measurement noise covariance
              obj.Q = sys.Q;
            
            % initial mean and covariance
              obj.mu = init.mu;
              obj.Sigma = init.Sigma;
        end
        
        %% Complete prediction function for all robots
        function prediction(obj, u)
            
           % Allocate storage for predicted mean
             obj.mu_pred =  zeros(3*obj.Num,1);
           % Allocate storage for Jacobians
             Gfun_local = zeros(3*obj.Num,3*obj.Num);
             Vfun_local = zeros(3*obj.Num,3*obj.Num);
             M_local = zeros(3*obj.Num,3*obj.Num);
             
           % Loop through all robots
             for i = 1 : obj.Num
               % Compute the predicted mean
                 obj.mu_pred(3*(i-1)+1:3*i,1) = obj.gfun(obj.mu(3*(i-1)+1:3*i,1), u(3*(i-1)+1:3*i,1));
               % Construct the Gfun, Vfun,and M necessary for the
               % calculation of predicted mean.
                 Gfun_local(3*(i-1)+1:3*i,3*(i-1)+1:3*i) = ...
                      obj.Gfun(obj.mu(3*(i-1)+1:3*i,1), u(3*(i-1)+1:3*i,1));
                 Vfun_local(3*(i-1)+1:3*i,3*(i-1)+1:3*i) = ...
                      obj.Vfun(obj.mu(3*(i-1)+1:3*i,1), u(3*(i-1)+1:3*i,1));
                 M_local(3*(i-1)+1:3*i,3*(i-1)+1:3*i) = obj.M(u(3*(i-1)+1:3*i,1)) ;
             end
            % Compute the predicted variance
            obj.Sigma_pred = Gfun_local * obj.Sigma * Gfun_local' ...
                           + Vfun_local * M_local * Vfun_local';
        end
        

        %% Complete prediction function for one robot
        function prediction_single(obj, u)
             
           % Compute the predicted mean
             obj.mu_pred= obj.gfun(obj.mu, u);
           % Construct the Gfun, Vfun,and M necessary for the
           % calculation of predicted mean.
             Gfun_local = obj.Gfun(obj.mu, u);
             Vfun_local = obj.Vfun(obj.mu, u);
             M_local = obj.M(u);

            % Compute the predicted variance
            obj.Sigma_pred = Gfun_local * obj.Sigma * Gfun_local' ...
                           + Vfun_local * M_local * Vfun_local';
        end
        
        
        %% Complete correction function - single robot
        function correction_relative_single(obj, z, relative_pos, Number)
            
            % Allocate the storage for z_hat, Jacobian;
            z_hat = zeros(2*(Number-1),1);
            H_local = zeros(2*(Number-1), 3);
            
            for i = 1:Number-1
                % Compute the anticipated measurements  
%                 z_hat(2*(i-1)+1:2*i,1) = ...
%                     obj.hfun_relative(obj.mu_pred, relative_pos(3*(i-1)+1:3*i,1));
                
                z_hat(2*(i-1)+1:2*i,1) = obj.hfun_landmark(obj.mu_pred(1), obj.mu_pred(2), relative_pos(3*(i-1)+1:3*i,1));
                
                % Construct the Jacobian H
                % Current version takes all relative measurements into account
                [H_bearing_i, H_bearing_j, H_range_i, H_range_j] = ...
                     H_relative(obj.mu_pred, relative_pos(3*(i-1)+1:3*i,1));
                 
                H_local(2*(i-1)+1, :) = H_local(1, :) + H_bearing_i;
                H_local(2*i, :) = H_local(2, :) + H_bearing_j;
            end
            
            Q_stack = zeros(2*(Number-1),2*(Number-1));
            for i = 1 : Number-1
                Q_stack(2*(i-1)+1:2*i , 2*(i-1)+1:2*i) = obj.Q;
            end

            % innovation covariance
            S = H_local * obj.Sigma_pred * H_local' + Q_stack;
            
            % compute innovation statistics  
            v = z - z_hat;
            
            % Wrap the bearing to the range [-pi,+pi]
            for i = 1 : Number-1
                v(2*(i-1)+1) = wrapToPi(v(2*(i-1)+1));
            end
            
            % filter gain
            K = obj.Sigma_pred * H_local' * (S \ eye(size(S)));
            
            % correct the predicted state statistics
            obj.mu = obj.mu_pred + K * v;

            % Wrap the angle to the range [0,+pi]
            for i = 1 : obj.Num
                obj.mu(3*i) = wrapToPi(obj.mu(3*i));
            end
            
            % Joseph update form
            i = eye(length(obj.mu));
            obj.Sigma = (i - K * H_local) * obj.Sigma_pred * (i - K * H_local)' ...
                    + K * Q_stack * K'; 
                
        end
        
        
        
        %% Complete correction function - robot
        function correction_relative_single_all(obj, z)
                    
            % Allocate the storage for z_hat and Jacobian H
            z_hat = zeros(2*obj.Num, 1); 
            z_relevant = zeros(2*obj.Num, 1); 
            H_local = zeros(2*obj.Num, 3*obj.Num);
            
            iter = 1;
            % Loop through all pairs of robots
            for i = 1 : obj.Num
                j = mod(i + iter, obj.Num);
                if j == 0
                    j = obj.Num;
                end
                
                % Compute z_relevant
                if i<j
                    z_pos = 2*((i-1)*(obj.Num-1)+(j-1)-1);
                else
                    z_pos = 2*((i-1)*(obj.Num-1)+(j)-1);
                end
                
                z_relevant(2*(i-1)+1:2*i, 1) = z(z_pos+1 : z_pos+2,1);
                
                % Compute the anticipated measurements  
                z_hat(2*(i-1)+1 : 2*i, 1) = ...
                    obj.hfun_relative(obj.mu_pred(3*(i-1)+1:3*i,1), obj.mu_pred(3*(j-1)+1:3*j,1));

                % Construct the Jacobian H
                % Current version takes all relative measurements into account
                [H_bearing_i, H_bearing_j, H_range_i, H_range_j] = ...
                     H_relative(obj.mu_pred(3*(i-1)+1:3*i,1), obj.mu_pred(3*(j-1)+1:3*j,1));

                % Judge whether to fuse the current relative measurement
                flag_usage = determine_usage();
                if flag_usage                               
                    H_local(2*(i-1)+1, 3*(i-1)+1:3*i) = H_bearing_i;
                    H_local(2*i, 3*(i-1)+1:3*i) = H_range_i;
                    H_local(2*(i-1)+1, 3*(j-1)+1:3*j) = H_bearing_j;
                    H_local(2*i, 3*(j-1)+1:3*j) = H_range_j;
                end
            end
             
            % stack the measurement noise
            Q_stack = zeros(2*obj.Num, 2*obj.Num);
            for i = 1 : obj.Num
                Q_stack(2*(i-1)+1:2*i , 2*(i-1)+1:2*i) = obj.Q;
            end
            % innovation covariance
            S = H_local * obj.Sigma_pred * H_local' + Q_stack;
        
            % compute innovation statistics  
            v = z_relevant - z_hat;
            % Wrap the bearing to the range [0,+pi]
            for i = 1 : obj.Num
               v(2*(i-1)+1) = wrapToPi(v(2*(i-1)+1));
            end
            
            % filter gain
            K = obj.Sigma_pred * H_local' * (S \ eye(size(S)));
            
            % correct the predicted state statistics
            obj.mu = obj.mu_pred + K * v;

            % Wrap the angle to the range [0,+pi]
            for i = 1 : obj.Num
                obj.mu(3*i) = wrapToPi(obj.mu(3*i));
            end
            
            % Joseph update form
            i = eye(length(obj.mu));
            obj.Sigma = (i - K * H_local) * obj.Sigma_pred * (i - K * H_local)' ...
                    + K * Q_stack * K'; 
        end
        
        %% Complete correction function - robot
        function correction_relative_batch_all(obj, z)
                    
            % Allocate the storage for z_hat and Jacobian H
            z_hat = zeros(2*obj.Num*(obj.Num-1),1);
             
            H_local = zeros(2*obj.Num*(obj.Num-1), 3*obj.Num);
            % Loop through all pairs of robots
            
            for i = 1 : obj.Num
                jj = 0;
                 
                for j = 1 : obj.Num
                     
                    if i ~= j
                        
                        jj = jj + 1;
                        
                        % Compute the anticipated measurements  
                        z_hat(2*(obj.Num-1)*(i-1) + 2*(jj-1)+1 : 2*(obj.Num-1)*(i-1) + 2*jj,1) = ...
                            obj.hfun_relative(obj.mu_pred(3*(i-1)+1:3*i,1), obj.mu_pred(3*(j-1)+1:3*j,1));
                        
                        % Construct the Jacobian H
                        % Current version takes all relative measurements into account
                        [H_bearing_i, H_bearing_j, H_range_i, H_range_j] = ...
                             H_relative(obj.mu_pred(3*(i-1)+1:3*i,1), obj.mu_pred(3*(j-1)+1:3*j,1));
                         
                        % Judge whether to fuse the current relative measurement
                        flag_usage = determine_usage();
                        if flag_usage                               
                            H_local(2*(obj.Num-1)*(i-1)+2*(jj-1)+1, 3*(i-1)+1:3*i) = H_bearing_i;
                            H_local(2*(obj.Num-1)*(i-1)+2*(jj-1)+1, 3*(j-1)+1:3*j) = H_bearing_j;
                            H_local(2*(obj.Num-1)*(i-1)+2*jj, 3*(i-1)+1:3*i) = H_range_i;
                            H_local(2*(obj.Num-1)*(i-1)+2*jj, 3*(j-1)+1:3*j) = H_range_j;
                        end
                    end   
                 end
            end
%             obj.mu = obj.mu_pred;
%             obj.Sigma = obj.Sigma_pred;
             
            % stack the measurement noise
            Q_stack = zeros(2*obj.Num*(obj.Num-1),2*obj.Num*(obj.Num-1));
            for i = 1 : obj.Num*(obj.Num-1)
                Q_stack(2*(i-1)+1:2*i , 2*(i-1)+1:2*i) = obj.Q;
            end
            % innovation covariance
            S = H_local * obj.Sigma_pred * H_local' + Q_stack;
        
            % compute innovation statistics                
            v = z - z_hat;
            % Wrap the bearing to the range [0,+pi]
            for i = 1 : obj.Num*(obj.Num - 1)
               v(2*i) = wrapToPi(v(2*i));
            end
            
            % filter gain
            K = obj.Sigma_pred * H_local' * (S \ eye(size(S)));
            
            % correct the predicted state statistics
            obj.mu = obj.mu_pred + K * v;
            % Wrap the angle to the range [0,+pi]
            for i = 1 : obj.Num
                obj.mu(3*i) = wrapToPi(obj.mu(3*i));
            end
            
            % Joseph update form
            i = eye(length(obj.mu));
            obj.Sigma = (i - K * H_local) * obj.Sigma_pred * (i - K * H_local)' ...
                    + K * Q_stack * K'; 
        end
        
        
        
         %% Complete correction function - landmark, update robot number i
         function correction_landmark_single(obj, z, landmark)

            % Compute the anticipated measurements  (2, 1)
            z_hat = ...
                 obj.hfun_landmark(landmark(1),landmark(2),obj.mu_pred);
            % Construct the Jacobian H (2, 3)
            H_local = obj.Hfun(landmark(1),landmark(2), obj.mu_pred, z_hat);

          % innovation covariance
            S = H_local * obj.Sigma_pred * H_local' + obj.Q;
        
          % compute innovation statistics                
            v = z - z_hat;
          % Wrap the bearing to the range [0,+pi]
            v(1) = wrapToPi(v(1));
            
          % filter gain
            K = obj.Sigma_pred * H_local' * (S \ eye(size(S)));
            
          % correct the predicted state statistics
            obj.mu = obj.mu_pred + K * v;
          % Wrap the angle to the range [0,+pi]
            obj.mu(3) = wrapToPi(obj.mu(3));
            
          % Joseph update form
            I = eye(3);
            obj.Sigma = ...
                (I - K * H_local) * obj.Sigma_pred * (I - K * H_local)' ...
                    + K * obj.Q * K'; 

         end
         
         %%
%          %% Complete correction function - landmark, update robot number i
%          function correction_landmark_single(obj, robot_i, z, landmark)
% 
%             % Compute the anticipated measurements  (2, 1)
%             z_hat = ...
%                  obj.hfun_landmark(landmark(1),landmark(2),obj.mu_pred(3*(robot_i-1)+1:3*robot_i,1));
%             % Construct the Jacobian H (2, 3)
%             H_local = obj.Hfun(landmark(1),landmark(2),...
%                             obj.mu_pred(3*(robot_i-1)+1:3*robot_i,1),...
%                             z_hat);
% 
%           % innovation covariance
%             S = H_local * obj.Sigma_pred(3*(robot_i-1)+1:3*robot_i, 3*(robot_i-1)+1:3*robot_i) * H_local' + obj.Q;
%         
%           % compute innovation statistics                
%             v = z - z_hat;
%           % Wrap the bearing to the range [0,+pi]
%             v(1) = wrapToPi(v(1));
%             
%           % filter gain
%             K = obj.Sigma_pred(3*(robot_i-1)+1:3*robot_i, 3*(robot_i-1)+1:3*robot_i) * H_local' * (S \ eye(size(S)));
%             
%           % correct the predicted state statistics
%             obj.mu(3*(robot_i-1)+1:3*robot_i,1) = obj.mu_pred(3*(robot_i-1)+1:3*robot_i,1) + K * v;
%           % Wrap the angle to the range [0,+pi]
%             obj.mu(3*robot_i) = wrapToPi(obj.mu(3*robot_i));
%             
%           % Joseph update form
%             I = eye(3);
%             obj.Sigma(3*(robot_i-1)+1:3*robot_i, 3*(robot_i-1)+1:3*robot_i) = ...
%                 (I - K * H_local) * obj.Sigma_pred(3*(robot_i-1)+1:3*robot_i, 3*(robot_i-1)+1:3*robot_i) * (I - K * H_local)' ...
%                     + K * obj.Q * K'; 
% 
%          end
         
         %% Complete correction function - landmark
         function correction_landmark(obj, z, landmark)
            % Allocate the storage for z_hat and Jacobian H
             z_hat = zeros(2*obj.Num,1);
             H_local = zeros(2*obj.Num,3*obj.Num);
           % Loop through all robots
             for i = 1 : obj.Num
                % Compute the anticipated measurements  
                z_hat(2*(i-1)+1 : 2*i,1) = ...
                     obj.hfun_landmark(landmark(1),landmark(2),obj.mu_pred(3*(i-1)+1:3*i,1));
                % Construct the Jacobian H
                H_blcok = obj.Hfun(landmark(1),landmark(2),...
                                obj.mu_pred(3*(i-1)+1:3*i,1),...
                                z_hat(2*(i-1)+1 : 2*i,1));
                H_local(2*(i-1)+1:2*i, 3*(i-1)+1:3*i) = H_blcok;           
             end
             
          % stack the measurement noise
            Q_stack = zeros(2*obj.Num, 2*obj.Num);
            for i = 1 : obj.Num
                Q_stack(2*(i-1)+1:2*i , 2*(i-1)+1:2*i) = obj.Q;
            end
            
          % innovation covariance
            S = H_local * obj.Sigma_pred * H_local' + Q_stack;
        
          % compute innovation statistics                
%             v = landmark - z_hat;
            v = z - z_hat;
          % Wrap the bearing to the range [0,+pi]
            for i = 1 : obj.Num
               v(2*(i-1)+1) = wrapToPi(v(2*(i-1)+1));
            end
            
          % filter gain
            K = obj.Sigma_pred * H_local' * (S \ eye(size(S)));
            
          % correct the predicted state statistics
            obj.mu = obj.mu_pred + K * v;
          % Wrap the angle to the range [0,+pi]
            for i = 1 : obj.Num
                obj.mu(3*i) = wrapToPi(obj.mu(3*i));
            end
            
          % Joseph update form
            i = eye(length(obj.mu));
            obj.Sigma = (i - K * H_local) * obj.Sigma_pred * (i - K * H_local)' ...
                    + K * Q_stack * K'; 

         end
         
         
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
      
    end
end

