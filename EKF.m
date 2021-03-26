classdef EKF < handle
    properties
        mu;             % Pose Mean
        Sigma;          % Pose Covariance
        
        gfun;           % Discrete Dynamical equations of motion
        hfun;           % Measurement model equations
        Gfun;           % Motion Model Jacobian with respect to state
        Vfun;           % Motion Model Jacobian with respect to control inputs
        Hfun;           % Measruement Model Jacobian
        
        M;              % Noise Covariance in input measruements
        Q;              % Sensor Noise Covariance
        
        mu_pred;        % Predictied Mean after prediction step
        Sigma_pred;     % Predictied Covarince after prediction step
        Num             % Number of Robots
    end
    
    methods
        function obj = EKF(sys, init)
            % Identify the number of robots
             obj.Num = init.Num;
             
            % motion model
            obj.gfun = sys.gfun;
            
            % measurement model
            obj.hfun = sys.hfun;
            
            % Jocabian of motion model
            obj.Gfun = init.Gfun;
            obj.Vfun = init.Vfun;
            
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
        
        %% Complete correction function
        function correction(obj, z)
                    
           % Allocate the storage for z_hat and Jacobian H
             z_hat = zeros(2*obj.Num*(obj.Num-1),1);
             H_local = zeros(2*obj.Num*(obj.Num-1),3*obj.Num);
           % Loop through all pairs of robots
             for k = 1 : obj.Num 
                 for kk = 1 : obj.Num-1
                   % Compute the anticipated measurements  
                     z_hat(2*(obj.Num-1)*(k-1) + 2*(kk-1)+1 : 2*(obj.Num-1)*(k-1) + 2*kk,1) = ...
                         obj.hfun(obj.mu_pred(3*(k-1)+1,3*k), obj.mu_pred(3*(kk-1)+1,3*kk));
                   % Construct the Jacobian H
                   % Current version takes all relative measurements into account
                     [H_bearing_i,H_bearing_j,H_range_i,H_range_j] = ...
                         H_relative(obj.mu_pred(3*(k-1)+1,3*k),obj.mu_pred(3*(kk-1)+1,3*kk));
                     flag_usage = determine_usage();
                   % Judge whether to fuse the current relative measurement
                     if flag_usage
                        H_local(2*(obj.Num-1)*(k-1)+2*(kk-1)+1,3*(k-1)+1:3*k) =...
                                   H_bearing_i;
                        H_local(2*(obj.Num-1)*(k-1)+2*(kk-1)+1,3*kk+1:3*kk+3) =...
                                   H_bearing_j;
                        H_local(2*(obj.Num-1)*(k-1)+2*kk,3*(k-1)+1:3*k) =...
                                   H_range_i;
                        H_local(2*(obj.Num-1)*(k-1)+2*kk,3*kk+1:3*kk+3) =...
                                   H_range_j;       
                     end
                     
                 end
             end
             
          % stack the measurement noise
            Q_stack = zeros(2*obj.Num*(obj.Num-1),2*obj.Num*(obj.Num-1));
            for i = 1 : obj.Num*(obj.Num-1)
                Q_stack(2*(i-1)+1:2i , 2*(i-1)+1:2i) = obj.Q;
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
            I = eye(length(obj.mu));
            obj.Sigma = (I - K * H_local) * obj.Sigma_pred * (I - K * H_local)' ...
                    + K * Q_stack * K'; 
        end
        
        % The following function computes the block of jacobian caused by
        % relative bearing measurement
        function [H_bearing_i,H_bearing_j,H_range_i,H_range_j] = H_relative(pos_i,pos_j)
        % For details of the following equation, readers are refered to
        % eq.(11) in Agostino et al. 2005
          delta_x = pos_j(1) - pos_i(1);
          delta_y = pos_j(2) - pos_j(2);
          H_bearing_i = [delta_y/(delta_x^2 + delta_y^2), -delta_x/(delta_x^2 + delta_y^2),-1];
          H_bearing_j = [-delta_y/(detla_x^2 + delta_y^2),delta_x/(delta_x^2 + delta_y^2),0];
        % For details of the following equation, readers are refered to
        % eq.(12) in Agostino et al. 2005 
          H_range_i = [-delta_x/sqrt(delta_x^2+delta_y^2),-delta_y/sqrt(delta_x^2+delta_y^2),0];
          H_range_j = [delta_x/sqrt(delta_x^2+delta_y^2),delta_y/sqrt(delta_x^2+delta_y^2),0];
        end
        
        % The following function determines whether the relative
        % measurement is reliable or not
         function [flag] = determine_usage()
             flag = 1;
         end
        
    end
end

