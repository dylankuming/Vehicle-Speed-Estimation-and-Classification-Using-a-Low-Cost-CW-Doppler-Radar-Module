%% (d) Vehicle Speed Computation with Extrapolation (Manual Direction)
function [repSpeed, repIdx, updatedTrack] = ...
         computeVehicleSpeedWithExtrap(timeVec, speedVec, distanceThreshold, Ts, direction)

    % Extrapolates a 0 m/s sample at start (away) or end (towards).
    % Computes cumulative radial distance and returns:
    %   repIdx   – index of representative point at distance threshold
    %   repSpeed – speed at repIdx
    %   updatedTrack – [time, speed] with the extrapolated zero appended/prepended


    switch direction
        case "away"
            % Prepend a zero at t0 - Ts 
            t0 = timeVec(1);
            timeVec  = [t0 - Ts; timeVec];
            speedVec = [0;      speedVec];

            radialDist = cumtrapz(timeVec, speedVec);        % distance from start
            idx = find(radialDist >= distanceThreshold, 1, 'first');
            if isempty(idx), idx = numel(timeVec); end
            repIdx   = idx;
            repSpeed = speedVec(repIdx);

        case "towards"
            % Append a zero at tend + Ts 
            te = timeVec(end);
            timeVec  = [timeVec;  te + Ts];
            speedVec = [speedVec; 0];

            radialDist = cumtrapz(timeVec, speedVec);        % distance from start
            totalDist  = radialDist(end);

            if totalDist > distanceThreshold
                neededDist = totalDist - distanceThreshold;  % walk back from end
                idx = find(radialDist >= neededDist, 1, 'first');
                repIdx = idx;
            else
                repIdx = 1;                                   % fallback: first sample
            end
            repSpeed = speedVec(repIdx);

        otherwise
            error('direction must be "towards" or "away".');
    end

    updatedTrack = [timeVec, speedVec];
    
end
