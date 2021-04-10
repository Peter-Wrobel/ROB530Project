classdef PF < handle
    properties
        gfun;               % Motion model function
        hfun_relative;      % Measurement Model Function (relative)
        hfun_landmark;      % Measurement Model Function (landmark)
        Q;                  % Sensor Noise
        M;                  % Motion Model Noise (dynamical and function of input)
        
        n;                  % Number of Particles
        particles;          % Pose of particle
        particle_weight;    % Particle Weight
        
        mu;
        Sigma;
        
        Num                 % Number of robots 
    end
    
    methods
        function obj = PF(sys, init)
            % motion model
            obj.gfun = sys.gfun;
            % measurement model using relative measurement
            obj.hfun_relative = sys.hfun_relative;
            % measurement model using landmark measurement
            obj.hfun_landmark = sys.hfun_landmark;
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
            L = chol(obj.M(u), 'lower');
          % Sample from motion model
            for i = 1 : obj.n    
                obj.particles(:,i) = obj.gfun(obj.particles(:,i),u+ L * randn(3,1)) ;
            end
            obj.meanAndVariance();
        end
        
        %% Complete correction function using realtive measurement
        function correction_relative(obj, z, relative_pos, Number)
          % Allocate the storage for z_hat;
            z_hat = zeros(2*(Number-1),1);
          % Initialize the weights  
            w = zeros(obj.n,1);
          % Loop through all pairs of robots
            for p = 1 : obj.n
              for j = 1 : Number - 1
                   % Compute the anticipated measurements  
                     z_hat(2*(j-1)+1:2*j,1) = ...
                         obj.hfun_relative(obj.particles(:,p),relative_pos(3*(j-1)+1:3*j,1));
              end
              
              v = z - z_hat;
            % Wrap the bearing to the range [-pi,+pi]
              for i = 1 : Number - 1
                  v(2*(i-1)+1) = wrapToPi(v(2*(i-1)+1));
              end
             
              Q_stack = zeros(2*(Number-1),2*(Number-1));
              for i = 1 : Number-1
                  Q_stack(2*(i-1)+1:2*i , 2*(i-1)+1:2*i) = obj.Q;
              end

              w(p) = mvnpdf(v, 0, 2.*Q_stack);
           end
            
           
            % update and normalize weights
            obj.particle_weight = obj.particle_weight .* w; % since we used motion model to sample
            obj.particle_weight = obj.particle_weight ./ sum(obj.particle_weight);
            
            % resampling every time step
              resample(obj);
              
            meanAndVariance(obj);
        end 
        
     %% Complete correction function using realtive measurement
      function correction_landmark(obj, z, landmark) 
            weight = zeros(obj.n,1);
            for j = 1:obj.n
              % Compute the measurement
                z_hat_local = ...
                     obj.hfun_landmark(landmark(1),landmark(2),obj.particles(:,j));
           
              % Compute the Innovation
                v = z - z_hat_local;
              % Wrap the bearing to [0 pi]
                v(1) = wrapToPi(v(1));
                 
               % Get weight probability of difference in measurement
                 weight(j) = mvnpdf(v, 0, 2.*obj.Q);
            end
            
          % Update Weights
            obj.particle_weight = obj.particle_weight .* weight;
            obj.particle_weight = obj.particle_weight./sum(obj.particle_weight);
            
          % resampling every time step
            obj.resample();     
            obj.meanAndVariance();
        end 
     
     
     
     %% Other Methods
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