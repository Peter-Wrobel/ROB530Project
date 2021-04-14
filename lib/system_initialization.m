function sys = system_initialization(alphas, beta, deltaT)

%% The following motion model comems from HW5
sys.gfun = @(mu, u) [...
    mu(1) + (-u(1) / u(2) * sin(mu(3)) + u(1) / u(2) * sin(mu(3) + u(2)*deltaT));
    mu(2) + ( u(1) / u(2) * cos(mu(3)) - u(1) / u(2) * cos(mu(3) + u(2)*deltaT));
    mu(3) + u(2)*deltaT + u(3)*deltaT];

%% The following relative measurement model comes from Agostino
%  And the measurement model involves landmark comes from HW5
sys.hfun_landmark = @(landmark_x, landmark_y, mu_pred) [...
    wrapToPi(atan2(landmark_y - mu_pred(2), landmark_x - mu_pred(1)) - mu_pred(3));
    sqrt((landmark_y - mu_pred(2))^2 + (landmark_x - mu_pred(1))^2)];

sys.hfun_relative = @(pos_i, pos_j) [...
    wrapToPi(atan((-sin(pos_i(3))*(pos_j(1) - pos_i(1))+cos(pos_i(3))*(pos_j(2) - pos_i(2)))...
                 /( cos(pos_i(3))*(pos_j(1) - pos_i(1))+sin(pos_i(3))*(pos_j(2) - pos_i(2)))));
    sqrt((pos_j(1) - pos_i(1))^2 + (pos_j(2) - pos_i(2))^2)];

%% Motion noise
sys.M = @(u) [...
    alphas(1)*u(1)^2+alphas(2)*u(2)^2, 0, 0;
    0, alphas(3)*u(1)^2+alphas(4)*u(2)^2, 0;
    0, 0, alphas(5)*u(1)^2+alphas(6)*u(2)^2];

%% Measurement noise

% % Work for EKF
% sys.Q = [...
%         beta^2,    0;
%         0,      25^2];
    
sys.Q = 200 * [...
        beta^2,    0;
        0,       0.1];


end