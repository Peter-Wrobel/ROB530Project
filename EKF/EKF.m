classdef EKF < handle
    properties
        mu;             % Pose Mean
        Sigma;          % Pose Covariance
        gfun;           % Discrete Dynamical equations of motion
        hfun;           % Measurement model equations
        Gfun;           % Motion Model Jacobian with respect to state
        Vfun;           % Motion Model Jacobian with respect to control inputs
        Hfun;           % Measruement Model Jacobian
        M;              % Noise Covariance in input measruements(This is dynamical, changes with the input values)
        Q;              % Sensor Noise Covariance
        mu_pred;        % Predictied Mean after prediction step
        Sigma_pred;     % Predictied Covarince after prediction step
    end
    
    methods
        function obj = EKF(sys, init)
            % motion model
            obj.gfun = sys.gfun;
%             obj.gfun2 = sys.gfun2;
            % measurement model
            obj.hfun = sys.hfun;
            % Jocabian of motion model
            obj.Gfun = init.Gfun;
            obj.Vfun = init.Vfun;
            % Jocabian of measurement model
            obj.Hfun = init.Hfun;
            % motion noise covariance
            obj.M = sys.M;
            % measurement noise covariance
            obj.Q = sys.Q;
            % initial mean and covariance
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
        end
        
        function prediction(obj, u)
            %Evaluate G with mean and input
            G = obj.Gfun(obj.mu, u); 
            %Evaluate V with mean and input
            V = obj.Vfun(obj.mu, u); 
            %Propoagate mean through non-linear dynamics
            obj.mu_pred = obj.gfun(obj.mu, u);
            %Update covariance with G,V and M(u)
            obj.Sigma_pred = G * obj.Sigma * G' + V * obj.M(u) * V';
        end
        
        function correction(obj, z)
            global FIELDINFO;
            landmark_x = FIELDINFO.MARKER_X_POS(z(3));
            landmark_y = FIELDINFO.MARKER_Y_POS(z(3));
            landmark_x2 = FIELDINFO.MARKER_X_POS(z(6));
            landmark_y2 = FIELDINFO.MARKER_Y_POS(z(6));
            
            z_hat_1 = obj.hfun(landmark_x, landmark_y, obj.mu_pred);
            z_hat_2 = obj.hfun(landmark_x2, landmark_y2, obj.mu_pred);
            % Compute expected observation and Jacobian
            H_1 = obj.Hfun(landmark_x, landmark_y, obj.mu_pred, z_hat_1);
            H_2 = obj.Hfun(landmark_x2, landmark_y2, obj.mu_pred, z_hat_2);
            H = [H_1;H_2];
            % Innovation / residual covariance
            S = H*obj.Sigma_pred*H' + blkdiag(obj.Q,obj.Q);
            % Kalman gain
            K = obj.Sigma_pred * H' * S^-1;
            % Correction
            diff = [...
                wrapToPi(z(1) - z_hat_1(1));
                z(2) - z_hat_1(2);
                wrapToPi(z(4) - z_hat_2(1));
                z(5) - z_hat_2(2)];
            obj.mu = obj.mu_pred + K * diff;
            obj.mu(3) = wrapToPi(obj.mu(3));
            U = eye(length(obj.mu)) - K * H;
            obj.Sigma = U * obj.Sigma_pred * U' + K * blkdiag(obj.Q,obj.Q) * K';
        end
    end
end

