function kf = initCAKF(detection, Ts)
    v0 = detection.Measurement;              % velocity from detection
    x0 = [v0; 0];                            % Set initial state (zero acceleration)

    A  = [1 Ts; 0 1];                        % state transition matrix 
    H  = [1 0];                              % measurement matrix

    % White-jerk PSD (tune q): larger -> follows bends/true accels faster
    q  = 0.4;                                
    Q  = q * [Ts^3/3, Ts^2/2; Ts^2/2, Ts];   % process noise covariance

    R  = detection.MeasurementNoise;         % set from cluster spread
    P0 = diag([ (3*sqrt(R))^2, 4 ]);         % covariance matrix

    kf = trackingKF('State',x0, ...
        'StateTransitionModel',A, ...
        'MeasurementModel',H, ...
        'ProcessNoise',Q, ...
        'MeasurementNoise',R, ...
        'StateCovariance',P0, ...
        'EnableSmoothing',true);
end