function mergedTracks = mergeTracksAllPassesAvg(trackHistories, speedThreshold, minOverlapDuration)

 
    % Repeatedly merges tracks if they overlap in time and their average speed 
    % difference (within the overlap) is below a specified threshold.
    
    % Inputs:
    %   trackHistories     - A map containing time-speed histories for each track ID.
    %   speedThreshold     - Maximum allowable average speed difference (within the overlap) for merging (0.5m/s).
    %   minOverlapDuration - Minimum required overlap duration (in seconds) to consider merging (1.5s).
    
    % Output:
    %   mergedTracks - A cell array containing the merged track histories.
    
    % Get all the keys (TrackIDs) and sort each track by time
    keysList = trackHistories.keys;
    trackList = cell(length(keysList),1);
    
    % Convert map data into a cell array, ensuring each track is sorted using the first column (time) as the key.
    % Sorting ensures that each track's timeline is strictly increasing, which is critical for correctly identifying 
    % overlaps and interpolating speeds.
    for idx = 1:length(keysList)
        trackList{idx} = sortrows(trackHistories(keysList{idx}), 1);
    end

    % Perform merging until no more merges occur
    mergesOccured = true;
    while mergesOccured
        mergesOccured = false;
        i = 1;
        
        % Compare each track with subsequent tracks to see if they can be merged
        while i < length(trackList)
            j = i + 1;
            merged = trackList{i};
            mergedIndex = i;
            
            % Attempt merging with all remaining tracks
            while j <= length(trackList)
                [didMerge, newMerged] = tryMergeAverage(merged, trackList{j}, ...
                                                        speedThreshold, minOverlapDuration);
                if didMerge
                    % If a merge occurred, replace 'merged' with the newly merged track
                    merged = newMerged;
                    trackList(j) = [];     % Remove the merged track from the list
                    mergesOccured = true;  % Flag that a merge occurred in this pass
                else
                    j = j + 1;  % Move on to the next track
                end
            end
            
            % Update the merged track in the list
            trackList{mergedIndex} = merged;
            i = i + 1;  % Move on to the next track
        end
    end

    % Return the final list of merged tracks
    mergedTracks = trackList;
end

function [didMerge, mergedTrack] = tryMergeAverage(trackA, trackB, speedThreshold, minOverlapDuration)
    
    % Checks if two tracks overlap in time for at least 'minOverlapDuration' 
    % and if their average speed difference within that overlap is below 'speedThreshold'.
    % If both conditions are met, the two tracks are merged by averaging speeds 
    % over the overlap region.
    
    % Inputs:
    %   trackA, trackB       - Nx2 matrices with columns [time, speed].
    %   speedThreshold       - Maximum average speed difference allowed for merging (0.5m/s).
    %   minOverlapDuration   - Minimum overlap duration in seconds for merging (1.5s).
    
    % Outputs:
    %   didMerge     - Logical flag indicating whether a merge was performed.
    %   mergedTrack  - The resulting merged track if didMerge is true; otherwise, trackA is returned.

    didMerge = false;
    mergedTrack = trackA;

    % Ensure both track histories are sorted by time
    trackA = sortrows(trackA,1);
    trackB = sortrows(trackB,1);

    % Determine the overlapping time interval between the two tracks
    t_start = max(min(trackA(:,1)), min(trackB(:,1)));
    t_end   = min(max(trackA(:,1)), max(trackB(:,1)));

    % Compute the overlap duration
    overlapTime = t_end - t_start;
    
    % Proceed only if there is a valid overlap meeting the minimum duration
    if t_start < t_end && (overlapTime >= minOverlapDuration)
        % Interpolate speeds at multiple points within the overlapping time to estimate average speed difference
        numInterpPoints = 20;  % Number of interpolation points
        commonTimes = linspace(t_start, t_end, numInterpPoints);
        
        speedsA = interp1(trackA(:,1), trackA(:,2), commonTimes, 'linear', NaN);
        speedsB = interp1(trackB(:,1), trackB(:,2), commonTimes, 'linear', NaN);

        % Calculate the mean absolute speed difference within the overlap
        avgDiff = mean(abs(speedsA - speedsB));

       % If the average speed difference is below threshold, merge the two tracks
        if avgDiff < speedThreshold
            didMerge = true;
            
            % Create a combined time axis from both tracks
            mergeTimeAxis = unique([trackA(:,1); trackB(:,1)]);
            mergedSpeeds = zeros(size(mergeTimeAxis));
            
            % For each time in the combined time list
            for k = 1:length(mergeTimeAxis)
                t = mergeTimeAxis(k);

                if t >= t_start && t <= t_end

                    % WITHIN THE OVERLAP
                    vA = interp1(trackA(:,1), trackA(:,2), t, 'linear', NaN);
                    vB = interp1(trackB(:,1), trackB(:,2), t, 'linear', NaN);
                    mergedSpeeds(k) = (vA + vB) / 2;  % Average speed in the overlap

                else
                    % OUTSIDE THE OVERLAP
                    % Try interpolating from both tracks:
                    vA = interp1(trackA(:,1), trackA(:,2), t, 'linear', NaN);
                    vB = interp1(trackB(:,1), trackB(:,2), t, 'linear', NaN);

                    if ~isnan(vA) && isnan(vB)
                        % Only Track A has valid data here
                        mergedSpeeds(k) = vA;
                    elseif isnan(vA) && ~isnan(vB)
                        % Only Track B has valid data here
                        mergedSpeeds(k) = vB;
                    elseif ~isnan(vA) && ~isnan(vB)
                        % Both are valid outside the overlap (rare).
                        mergedSpeeds(k) = (vA + vB) / 2;  % average them 
                    else
                        % Both are NaN => no track covers this time
                        mergedSpeeds(k) = NaN;
                    end
                end
            end
            
            % The merged track combines all times and the new averaged speeds
            mergedTrack = [mergeTimeAxis(:), mergedSpeeds(:)];
        end
    end
end