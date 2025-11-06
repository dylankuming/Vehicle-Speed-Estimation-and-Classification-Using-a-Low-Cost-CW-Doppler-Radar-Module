function mergedTracks = mergeTracksSameVelocityGap(trackList, maxTimeGap, maxSpeedGap)

%   Merges pairs of tracks that do NOT overlap in time, but whose end/start
%   times are close (<= maxTimeGap) and whose end/start speeds are within
%   maxSpeedGap. This ensures short gaps don't cause separate tracks.

    mergesOccured = true;
    while mergesOccured
        mergesOccured = false;
        i = 1;
        
        while i < length(trackList)
            j = i + 1;
            merged = trackList{i};
            mergedIndex = i;
            
            while j <= length(trackList)
                [didMerge, newMerged] = tryMergeSameVelocity(merged, trackList{j}, ...
                                                             maxTimeGap, maxSpeedGap);
                if didMerge
                    % Remove the track we just merged (trackList{j})
                    merged = newMerged;
                    trackList(j) = [];
                    mergesOccured = true;
                else
                    j = j + 1;
                end
            end
            
            % Update the track in the cell array
            trackList{mergedIndex} = merged;
            i = i + 1;
        end
    end
    
    mergedTracks = trackList;
end

function [didMerge, mergedTrack] = tryMergeSameVelocity(trackA, trackB, maxTimeGap, maxSpeedGap)
% tryMergeSameVelocity:
%   Checks if trackA and trackB do NOT overlap in time, but are within maxTimeGap
%   in time. If so, merges them.

    didMerge = false;
    mergedTrack = trackA;
    
    % Sort both tracks by time (safeguard)
    trackA = sortrows(trackA, 1);
    trackB = sortrows(trackB, 1);
    
    % If either track has fewer than 16 samples, skip merging (16*Ts=0,75s)
    if size(trackA,1) < 16 || size(trackB,1) < 16
        return; % didMerge stays false
    end
    
    % Identify time/speed at the boundary
    tA_end = trackA(end, 1);
    tB_start = trackB(1, 1);
    timeGap = tB_start - tA_end;
    
    % Check basic time-gap condition (tracks should not overlap and be <= maxTimeGap apart)
    if timeGap <= 0 || timeGap > maxTimeGap
        return; % They overlap or gap is too big
    end

    
    
    % Extract the last 16 speeds from Track A
    last16A = trackA(end-15:end, 2);  % the last 16 speed values
    
    % Extract the first 16 speeds from Track B
    first16B = trackB(1:16, 2);       % the first 16 speed values



    % Check if ALL pairwise differences are within maxSpeedGap
    % In other words, for each i in [1..16], |last16A(i) - first16B(i)| <= maxSpeedGap
    speedDiff = abs(last16A - first16B);
    if all(speedDiff <= maxSpeedGap)
        % If they satisfy the velocity requirement, +merge them by concatenating
        didMerge = true;
        mergedTrack = [trackA; trackB];
    end
end