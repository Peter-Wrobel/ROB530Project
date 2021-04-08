function filter = filter_initialization(sys, initialStateMean,initialStateCov, filter_name,Num)
switch filter_name
    case "EKF"
        init.mu = initialStateMean;
        init.Sigma = initialStateCov;
        
        init.Gfun = @(mu, u) [...
            1, 0, (u(1) * cos(mu(3) + u(2)))/u(2) - (u(1) * cos(mu(3)))/u(2);
            0, 1, (u(1) * sin(mu(3) + u(2)))/u(2) - (u(1) * sin(mu(3)))/u(2);
            0, 0,                                                          1];
        
        init.Vfun = @(mu, u) [...
            sin(mu(3) + u(2))/u(2) - sin(mu(3))/u(2), (u(1)*cos(mu(3) + u(2)))/u(2) - (u(1)*sin(mu(3) + u(2)))/u(2)^2 + (u(1)*sin(mu(3)))/u(2)^2, 0;
            cos(mu(3))/u(2) - cos(mu(3) + u(2))/u(2), (u(1)*cos(mu(3) + u(2)))/u(2)^2 + (u(1)*sin(mu(3) + u(2)))/u(2) - (u(1)*cos(mu(3)))/u(2)^2, 0;
            0,                                                                                                                                 1, 1];
        
        init.Hfun = @(landmark_x, landmark_y, mu_pred, z_hat) [...
            (landmark_y - mu_pred(2))/(z_hat(2)^2), -(landmark_x - mu_pred(1))/(z_hat(2)^2), -1;
            -(landmark_x - mu_pred(1))/z_hat(2),     -(landmark_y - mu_pred(2))/z_hat(2),  0];
        
        init.Num = Num;
        
        filter = EKF(sys, init);
end
end