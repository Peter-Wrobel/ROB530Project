classdef PF_landmark< handle
    properties
        gfun;               % Motion model function
        hfun;               % Measurement Model Function
        Q;                  %Sensor Noise
        M;                  %Motion Model Noise (dynamical and function of input)
        n;                  %Number of Particles
        particles;          %Pose of particle
        particle_weight;    %Particle Weight
        mu;
        Sigma;
        
        Num                 % Number of robots 
    end
    
    methods
        function obj = PF(sys, init)
            % motion model
            obj.gfun = sys.gfun;
            % measurement model
            obj.hfun = sys.hfun;
            % motion noise covariance
            obj.M = sys.M;
            % measurement noise covariance
            obj.Q = sys.Q;
            % PF parameters
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
            obj.n = init.n;
            obj.particles = init.particles;
            obj.particle_weight = init.particle_weight;
            obj.Num = init.Num;
        end
        
        %% Complete prediction function
        function prediction(obj, u)
            M_local = zeros(size(3*obj.Num,3*obj.Num));
            for i = 1 : obj.Num 
                M_local(3*(i-1)+1:3i,3*(i-1)+1:3i) = obj.M(u(3*(i-1)+1:3i,1));
            end
            L = chol(M_local, 'lower');
          % Sample from motion model
            for i = 1 : obj.n    
                obj.particles(:,i) = obj.gfun(obj.particles(:,i),u+ L * randn(3*obj.Num ,1)) ;
            end
        end
        
        %% Complete correction function
        function correction(obj, z, landmark)
            
            weight = zeros(obj.n,1);
            z_hat_local = NaN .* ones();
            for j = 1:obj.n
              % Compute the measurement
                for i = 1 : obj.Num
                   z_hat_i = sys.hfun_landmark(landmark(1),landmark(2),obj.particles(3*(i-1):3*i,j));
                   z_hat_local = ;
                end
                z_hat1 = obj.hfun(landmark_x, landmark_y, obj.particles(:,j));
                z_hat2 = obj.hfun(landmark_x2, landmark_y2, obj.particles(:,j));
                v = [wrapToPi(z(1) - z_hat1(1));
                     z(2) - z_hat1(2);
                     wrapToPi(z(4) - z_hat2(1));
                     z(5) - z_hat2(2)];
                 
               % Get weight probability of difference in measurement
                 Q_stack = diag([diag(obj.Q);diag(obj.Q)]); 
                 weight(j) = mvnpdf(v, 0, 2.*Q_stack);
            end
            
          % Update Weights
            obj.particle_weight = obj.particle_weight .* weight;
            obj.particle_weight = obj.particle_weight./sum(obj.particle_weight);
            Neff = 1 / sum(obj.particle_weight.^2);
            if Neff < obj.n /5
                obj.resample();
            end
            obj.meanAndVariance();
        end 
         
        
        
        
        
        
        function resample(obj)
            newSamples = zeros(size(obj.particles));
            newWeight = zeros(size(obj.particle_weight));
            W = cumsum(obj.particle_weight);
            r = rand/obj.n;
            count = 1;
            for j = 1:obj.n
                u = r+(j-1)/obj.n;
                while u > W(count)
                    count = count+1;
                end
                newSamples(:,j) = obj.particles(:,count);
                newWeight(j) = 1/obj.n;
            end
            obj.particles = newSamples;
            obj.particle_weight = newWeight;
        end
        
        function meanAndVariance(obj)
            obj.mu = mean(obj.particles, 2); 
            % orientation is a bit more tricky.
            sinSum = 0;
            cosSum = 0;
            for s = 1:obj.n
                cosSum = cosSum + cos(obj.particles(3,s));
                sinSum = sinSum + sin(obj.particles(3,s));
            end
            obj.mu(3) = atan2(sinSum, cosSum);     
            % Compute covariance.
            zeroMean = obj.particles - repmat(obj.mu, 1, obj.n);
            for s = 1:obj.n
                zeroMean(3,s) = wrapTo2Pi(zeroMean(3,s));
            end
            
            obj.Sigma = zeroMean * zeroMean' / obj.n;
        end
    end
end