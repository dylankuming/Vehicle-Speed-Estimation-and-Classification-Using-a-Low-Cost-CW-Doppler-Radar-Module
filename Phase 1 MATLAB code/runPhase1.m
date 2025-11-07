clear all;
close all;

%% Set WAV file and direction paramaters
wavFile = '05_Control_2_Motorcycle_Car_towards.wav';
direction = "towards";           % <-- set manually: "towards" or "away"

%%  Load recording

[x, fs] = audioread(wavFile);
x = single(x);

%  Spectrogram parameters
frameLength      = 4096;
overlap          = round(0.5 * frameLength);
windowFunction   = hanning(frameLength);
% Compute power spectrogram (|STFT|^2)
[P, T, F] = powerSpectrogram(x, fs, frameLength, overlap, windowFunction);


% Doppler → speed (m/s)
lambda = 299792458 / 24e9;    % radar wavelength = c/f_c
v_mps  = (F * lambda) / 2;


% Crop to 10–70 km/h
mask  = (v_mps >= 10/3.6) & (v_mps <= 71/3.6);
P     = P(mask, :);
v_mps = v_mps(mask);


P_linear = P;
TimeAxis_s = T;  
Ts = mean(diff(TimeAxis_s));

% Normalise & convert to dB 
P_db = 10 * log10( P_linear ./ max(P_linear(:)) );


% Plot Spectrogram and convert to km/h
figure('Name','Power Spectrogram');
imagesc(T, v_mps*3.6, P_db, [-40 0]);
axis xy;
xlabel('Time (s)');
ylabel('Speed (km/h)');
title('Normalised Power Spectrogram (10–70 km/h)');
colormap jet;
colorbar;
% xlim([0,15]); 

% Parameters for OS-CFAR
PFA = 1e-6;             % Probability of False Alarm
RefWindow = 100;        % Number of reference cells on each side of CUT
NumGuardCells = 4;      % Number of guard cells on each side of CUT

NumRefCells = 2 * RefWindow;
k = floor(NumRefCells / 3);           % Order statistic index


warning off backtrace
prevWarn = warning('off','MATLAB:nchoosek:LargeCoefficient'); %  Suppress the large-coefficient warning from nchoosek
alpha_OS = solveAlphaOS(PFA, NumRefCells, k);  
warning(prevWarn); % Restore previous warning state 



% Initialise detection and threshold matrices
[Rows, Cols] = size(P_db);
Detection = zeros(Rows, Cols);
ThresholdMatrix = zeros(Rows, Cols);

% Precompute reference indices for each row (wrap-around)
lagIndicesAll = cell(Rows, 1);
leadIndicesAll = cell(Rows, 1);
for i = 1:Rows
    lagIndicesAll{i} = mod((i - RefWindow - NumGuardCells : i - NumGuardCells - 1) - 1, Rows) + 1;
    leadIndicesAll{i} = mod((i + NumGuardCells + 1 : i + NumGuardCells + RefWindow) - 1, Rows) + 1;
end


% Apply OS-CFAR to each column
for col = 1:Cols
    [Detection(:, col)] = oscfarDetector(P_linear(:, col), alpha_OS, k, lagIndicesAll, leadIndicesAll);
end

% Plot the spectrogram with detections (speed axis in km/h)
figure('Name','Spectrogram with OSCFAR detections');
imagesc(T, v_mps*3.6, P_db, [-40 0]);
axis xy;
xlabel('Time (s)');
ylabel('Speed (km/h)');
title('Spectrogram with OS-CFAR Detections');
colormap jet;
colorbar;
% xlim([0,15]);
 
hold on;

% Overlay detections with "x" symbols
[row_idx, col_idx] = find(Detection); % Find detection indices
plot(T(col_idx), v_mps(row_idx)*3.6, 'wx', 'MarkerSize', 3, 'DisplayName', 'Detections');
legend('Location', 'northeast', 'Color', 'k', 'TextColor', 'w');
hold off;


%% DBSCAN Clustering
finalSpeeds       = cell(1, Cols);  % Representative speeds per column
allClusterTimes   = [];             % For visualisation only
allClusterSpeeds  = [];             % For visualisation only
allNoiseTimes   = [];             % For visualisation only
allNoiseSpeeds  = [];             % For visualisation only

% DBSCAN parameters (1D in m/s) 
epsilon = ;  % 1.8 km/h
minPts  = 2;

for col = 1:Cols
    % CFAR hits in this time frame
    rowCandidates = find(Detection(:, col)); % gives indices of speed bins that contain detections
    if isempty(rowCandidates)
        finalSpeeds{col} = [];
        continue;
    end

    % Speeds (m/s) at CFAR-hit rows
    speedsInCol = v_mps(rowCandidates);
    ampsInCol   = P_linear(rowCandidates, col);
    

    % 1D DBSCAN over speed
    labels     = dbscan(speedsInCol(:), epsilon, minPts);

    % For visualisation: store noisy points
    noiseIdx = labels == -1;
    if any(noiseIdx)
        allNoiseTimes  = [allNoiseTimes;  repmat(T(col), sum(noiseIdx), 1)];
        noise_speeds = speedsInCol(noiseIdx);
        allNoiseSpeeds = [allNoiseSpeeds; noise_speeds(:)];
    end


    uniqueLabs = unique(labels(labels>0));  % exclude noise
    repVals    = [];

    for lab = uniqueLabs'
        inClust = labels == lab;
        sc      = speedsInCol(inClust);
        ampsCluster   = ampsInCol(inClust);

        % Make sure both are column vectors with matching length
        sc = sc(:);
        ampsCluster   =  ampsCluster (:);

        % For visualisation only: store all cluster points
        allClusterTimes  = [allClusterTimes;  repmat(T(col), sum(inClust), 1)];
        allClusterSpeeds = [allClusterSpeeds; sc(:)];

        % Representative value: arithmetic mean

        detectionVal = sum(sc .* ampsCluster) / sum(ampsCluster);
        repVals = [repVals, detectionVal];
    end

    finalSpeeds{col} = repVals;
end

% Gather representative detections
finalDetTimes  = [];
finalDetSpeeds = [];
for col = 1:Cols
    if ~isempty(finalSpeeds{col})
        finalDetTimes  = [finalDetTimes;  repmat(T(col), numel(finalSpeeds{col}), 1)];
        finalDetSpeeds = [finalDetSpeeds; finalSpeeds{col}(:)];
    end
end

% Plot DBSCAN Results in km/h
% Cluster points (green), noise points (blue), representative points (red)
figure('Name','DBSCAN Clustering and Chosen Detections');
hold on; grid on;
scatter(allClusterTimes, allClusterSpeeds*3.6, 5, 'g', 'filled', 'DisplayName','Cluster Points');
scatter(allNoiseTimes,  allNoiseSpeeds*3.6,  5, 'b', 'filled', 'DisplayName','Noise Points');
scatter(finalDetTimes,  finalDetSpeeds*3.6,  10, 'r', 'filled', 'DisplayName','Representative Points');
xlabel('Time (s)'); ylabel('Speed (km/h)');
title('DBSCAN Clustering, Representative and Noise Detections');
legend('Location','best'); 
% xlim([0,15]);
 
hold off;

% Plot representative points (red) only
figure('Name','Final DBSCAN Result');
scatter(finalDetTimes, finalDetSpeeds*3.6, 5, 'r', 'filled');
xlabel('Time (s)'); ylabel('Speed (km/h)');
title('Multiple Detections per Column (DBSCAN)');
% xlim([0,15]);
 
grid on;


%% Create TrackerJPDA with CA Kalman Filter
% Configure the multi-target tracker with JPDA + CA Kalman
numTracks=30;
tracker = trackerJPDA( ...
    'FilterInitializationFcn',  @(d) initCAKF(d, Ts), ...  % Use custom CA Kalman filter
    'MaxNumTracks', 30, ...                        % Maximum allowable concurrent tracks
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', 5, ...                   % Gating threshold in measurement space
    'TrackLogic', 'History', ...
    'DetectionProbability', 0.7, ...                % Probability of detection
    'ConfirmationThreshold', [4 6], ...  % require 4 hits in last 6 timesteps to start track
    'DeletionThreshold', [8 16]);     % if 8 of last 16 timesteps didn't have a detection then stop track
    


%%  Tracking Loop
% In summary, at each time step, the loop:
% * Identifies any detections that occur at that time.
% * Converts these detections into the required format (objectDetection objects).
% * Updates the tracker with these detections, either by updating existing tracks or initiating new ones.
% * Plots the estimated speeds from the tracker for visualisation.

allTimes = TimeAxis_s;             % Use the full spectrogram time axis
numSteps = length(allTimes);       % Number of distinct time steps
trackHistories = containers.Map('KeyType','int32','ValueType','any');


% Generate a distinct color map for visualizing different tracks (each TrackID gets a unique color)
colourMap = lines(numTracks);

% Set up the figure for visualizing tracking results
figure('Name','Tracking Results');
hold on; grid on;
xlabel('Time (s)'); ylabel('Speed (km/h)');
title('TrackerJPDA with CA Kalman Filter');
% xlim([0,15]);
ylim([10,70]);

R_meas = 0.04;       % measurement noise (variance) found empirically

% Find the first time step that has at least one detection:
if ~isempty(finalDetTimes)
    firstDetTime = min(finalDetTimes);
    startIdx = find(allTimes >= firstDetTime, 1, 'first');
else
    error('No detections found. The tracker cannot be initialised.');
end

% Loop over all time steps starting at the first detection time
for i = startIdx:numSteps
    currentTime = allTimes(i);              % Current time instant

    % Find indices of detections that occur at the current time.
    % Use a small tolerance to account for floating-point comparisons
   
    tol = Ts/10;
    idx = find(abs(finalDetTimes - currentTime) < tol);
    
    if isempty(idx)
        % If no detections at this time, send an empty cell array.
        detObjs = {};
    else
        % Otherwise, build objectDetection objects.
        detObjs = cell(length(idx), 1);
        for j = 1:length(idx)
            % Each detection is created with the current time, corresponding speed measurement,
            % and a predefined measurement noise R_meas
            detObjs{j} = objectDetection(currentTime, finalDetSpeeds(idx(j)), ...
                                         'MeasurementNoise', R_meas);
        end
    end

    % Update the tracker with the new detections at the current time
    tracks = tracker(detObjs, currentTime);
   

    % Loop through each track returned by the tracker at the current time and plot its state
    for k = 1:length(tracks)
      
    
        % history capture (m/s)
        tid = tracks(k).TrackID;
        row = [currentTime, tracks(k).State(1)];  % [time, speed_mps]
        if isKey(trackHistories, tid)
            trackHistories(tid) = [trackHistories(tid); row];
        else
            trackHistories(tid) = row;
        end

         % Plotting in km/h
        v_est_kmh = tracks(k).State(1) * 3.6;
        plot(currentTime, v_est_kmh, 'o', ...
            'Color', colourMap(mod(tracks(k).TrackID - 1, numTracks) + 1, :), ...
            'MarkerFaceColor', colourMap(mod(tracks(k).TrackID - 1, numTracks) + 1, :));
    end  
end

% Overlay all DBSCAN detections on the same plot for reference.
scatter(finalDetTimes, finalDetSpeeds*3.6, 10, 'ks', 'filled', ...
    'DisplayName', 'DBSCAN Detections');

hold off;

%% Merge Converged Tracks (Multi-Pass with Average Speed Difference)
% Merge partial or overlapping tracks that likely represent the same vehicle 
% if they have sufficient time overlap and minimal speed difference.

mergedTrackHistories_overlap = mergeTracksAllPassesAvg(trackHistories, , 1.5);

%% A) Plot Merged Tracks (Overlap-Based Only)

figure('Name','Merged Tracks (Overlap Only)');
hold on;
for i = 1:length(mergedTrackHistories_overlap)
    history = mergedTrackHistories_overlap{i};
    
    % Only plot tracks lasting more than 0.75 seconds
    if max(history(:,1)) - min(history(:,1)) > 0.75
        plot(history(:,1), history(:,2)*3.6, '-o', 'LineWidth', 1.5, ...
             'DisplayName', sprintf('Merged Track %d', i));
    end
end
% Overlay DBSCAN detection points for comparison
scatter(finalDetTimes, finalDetSpeeds*3.6, 10, 'ks', 'filled',...
    'DisplayName', 'DBSCAN Detections');

xlabel('Time (s)');
ylabel('Speed (km/h)');
title('Merged Tracks (Overlap-Based Only)');
grid on; legend('Location', 'best');
% xlim([0,15]);
 
hold off;

%% B) Merge Tracks Sharing the Same Velocity (Short Gaps)
% Merge tracks that do NOT overlap in time but share similar boundary velocities 
% and are close in time, preventing premature termination.

maxTimeGap  = 4;    % 4 seconds
maxSpeedGap = 0.5;  % 0.5 m/s difference
mergedTrackHistories_final = mergeTracksSameVelocityGap(mergedTrackHistories_overlap, maxTimeGap, maxSpeedGap);

%% Plot Merged Tracks (Overlap + Same-Velocity)

T_min = 2.25;  % discard tracks shorter than 2.25 seconds and update track list
mergedTrackHistories_final_masked = {};
for i = 1:length(mergedTrackHistories_final)
    history = mergedTrackHistories_final{i};
    duration = max(history(:,1)) - min(history(:,1));
    if duration >= T_min
        mergedTrackHistories_final_masked{end+1} = history;
    end
end

figure('Name','Merged Tracks (Overlap + Same-Velocity)');
hold on;
for i = 1:length(mergedTrackHistories_final_masked)
    history = mergedTrackHistories_final_masked{i};
    plot(history(:,1), history(:,2)*3.6, '-o', 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Merged Track %d', i));
end

% Overlay DBSCAN detection points for comparison
scatter(finalDetTimes, finalDetSpeeds*3.6, 10, 'ks', 'filled',...
    'DisplayName', 'Measurements');

xlabel('Time (s)');
ylabel('Speed (km/h)');
title('Merged Tracks (Including Same-Velocity Gap Merge)');
grid on; legend('Location', 'best');
% xlim([0,15]);
 
hold off;

%% Estimate Number of Vehicles and Representative Speeds 
distanceThreshold = 15;          % [m] radial distance threshold

N = numel(mergedTrackHistories_final_masked);
allSpeeds   = zeros(1, N);
validTracks = cell(1, N);
repIndices  = cell(1, N);

for i = 1:N
    rawTrack = mergedTrackHistories_final_masked{i};
    rawTrack = sortrows(rawTrack, 1);             % [time, speed] ascending time

    timeVec  = rawTrack(:,1);
    speedVec = rawTrack(:,2);

    [repSpeed, repIdx, trackWithExtrap] = ...
        computeVehicleSpeedWithExtrap(timeVec, speedVec, distanceThreshold, Ts, direction);

    D_perp = 2;                             % lateral offset (m)
    r = distanceThreshold + D_perp;         % LOS range
    R = sqrt(r^2 - D_perp^2);               % along-road distance
    cos_beta = R / r;                       % cosine factor

    allSpeeds(i)   = repSpeed/cos_beta;     % Representative true speed (m/s)
    repIndices{i}  = repIdx;                % Index of representative point in track
    validTracks{i} = trackWithExtrap;       % Extrapolated track with 0 m/s endpoint

end

numVehicles = N;
normWidth = zeros(1, numVehicles); 
detWidth = zeros(1, numVehicles); 

%% Re-index Vehicles by Their 0 m/s Crossing Time
% Whichever crosses speed=0 first becomes 'vehicle 1', next is 'vehicle 2', etc.
if numVehicles > 1
    crossTimes = zeros(numVehicles,1);
    for k = 1:numVehicles
        thisTrack = validTracks{k};
        zeroRow = find(thisTrack(:,2) == 0, 1, 'first');  % guaranteed by extrapolation
        crossTimes(k) = thisTrack(zeroRow, 1);
    end
    [~, sortIdx] = sort(crossTimes, 'ascend');
    validTracks = validTracks(sortIdx);
    repIndices  = repIndices(sortIdx);
    allSpeeds   = allSpeeds(sortIdx);
end

%% Plot Final Merged Tracks with Representative Radial Speed Point (km/h)
figure('Name','Final Merged Tracks');
hold on; grid on;
title(sprintf('Final Merged Tracks (Direction = %s)', direction));
xlabel('Time (s)'); ylabel('Speed (km/h)');

for k = 1:numVehicles
    thisTrack = validTracks{k};     % [time, speed] after extrapolation
    repIdx    = repIndices{k};

    t = thisTrack(:,1);
    v = thisTrack(:,2);

    plot(t, v*3.6, '-', 'LineWidth', 3, ...
        'DisplayName', sprintf('Vehicle %d (%s)', k, direction));

    if ~isempty(repIdx) && repIdx >= 1 && repIdx <= numel(t)
        plot(t(repIdx), v(repIdx)*3.6, 'kx', 'MarkerSize', 10, 'LineWidth', 2, ...
            'HandleVisibility', 'off');
    end
end

legend('Location','best');
% xlim([0,15]);
ylim([10,70]);
 
hold off;



%% Create a Binary Mask and Plot it
threshold_dB = -22;  % keep everything >= -22 dB
BinaryMask = (P_db >= threshold_dB);

figure('Name','Binary Mask >= -22 dB');
imagesc(T, v_mps*3.6, BinaryMask);
colormap("gray");
colorbar;
axis xy;
hold on;
xlabel('Time (s)');
ylabel('Speed (km/h)');
title('Binary Mask (Spectrogram >= -22 dB)');
% xlim([0,15]);
 

%% Vehicle Classification with Row-by-Row Detection Width & Single Plot

% STEP A: Loop over each tracked vehicle and classify
for k = 1:numVehicles

    %-----------------------------------------------------------------------
    % Retrieve track info and representative speed
    %-----------------------------------------------------------------------
    thisTrack = validTracks{k};     % Speed-time track after merging & extrapolation
    repSpeed  = allSpeeds(k);       % Representative true speed computed earlier

    % Identify the timestamp where speed is extrapolated to 0 m/s
    if direction == "away"
        zeroSpeedTime = thisTrack(1,1);
    else
        zeroSpeedTime = thisTrack(end,1);
    end

    %-----------------------------------------------------------------------
    % Define a 'ramp' bounding box in time & speed
    %-----------------------------------------------------------------------
    timeWindow = 1.5;       % +/- 1.5 s around zeroSpeedTime
    tMin = zeroSpeedTime - timeWindow;
    tMax = zeroSpeedTime + timeWindow;
    
    sMin = 3;  % Lower speed bound (m/s) for ramp detection
    sMax = 5;  % Upper speed bound (m/s)
    
    % Find indices in the time axis and speed axis that fall within the bounding box
    timeIdxRange  = find(T >= tMin & T <= tMax);
    speedIdxRange = find(v_mps >= sMin & v_mps <= sMax);

    if isempty(timeIdxRange) || isempty(speedIdxRange)
        % If no data exist in that bounding box, can't look for detections and thus can't classify
        vehicleClasses{k} = 'Unknown';
        normWidth(k)      = NaN; 
        fprintf('Vehicle %d: No detections in bounding box => Unknown class\n', k);
        continue;
    end

    %-----------------------------------------------------------------------
    % Extract Binary detections within the bounding box
    %-----------------------------------------------------------------------
    rampDetections = BinaryMask(speedIdxRange, timeIdxRange);
    
    % Remove small connected components in 'rampDetections' 
    % (any group with fewer than 8 pixels) using 8-pixel connectivity.
    rampDetections = bwareaopen(rampDetections, 8, 8);

    % Convert submatrix indices -> absolute time/speed
    [rRows, cCols] = find(rampDetections);

    if isempty(rRows)
        % If no actual detections remain after outlier removal, classification is impossible
        vehicleClasses{k} = 'Unknown';
        normWidth(k)      = NaN; 
        fprintf('Vehicle %d: No detections in ramp => Unknown\n', k);
        continue;
    end

    %-----------------------------------------------------------------------
    % Plot the bounding box and these binary detections
    %-----------------------------------------------------------------------


    actualTimesDetected  = T(timeIdxRange(cCols));
    actualSpeedsDetected = v_mps(speedIdxRange(rRows));
    plot(actualTimesDetected, actualSpeedsDetected*3.6, 'mo', 'MarkerSize',5, ...
         'DisplayName', sprintf('Vehicle %d Ramp Detections (Binary)', k));

    rectangle('Position', [tMin, sMin*3.6, (tMax - tMin), (sMax - sMin)*3.6], ...
          'EdgeColor','g','LineWidth',2);
    %-----------------------------------------------------------------------
    % Row-by-Row Time Span Calculation
    %-----------------------------------------------------------------------
    % For each speed bin (row), find earliest & latest detection times,
    % then compute (latest - earliest)
    % median these row-based widths to get detectionWidth_s.
    rowWidths = [];
    for rowLocal = 1:length(speedIdxRange) % for each column
        colsLocal = find(rampDetections(rowLocal, :));  % columns (within the submatrix) that have detections
          if ~isempty(colsLocal)
            % time span = (#last - #first) * Ts
            rowWidth_s = (max(colsLocal) - min(colsLocal)) * Ts;
            rowWidths(end+1,1) = rowWidth_s;
        end
    end

    if isempty(rowWidths)
        vehicleClasses{k} = 'Unknown';
        normWidth(k)      = NaN; 
        fprintf('Vehicle %d: No row-based detections => Unknown\n', k);
        continue;
    end

    % Median the row time spans
    detectionWidth_s = median(rowWidths);
    detWidth(k) = detectionWidth_s;

    %-----------------------------------------------------------------------
    % Multiply by representative speed => 'normalisedWidth'
    %-----------------------------------------------------------------------
    normalisedWidth = detectionWidth_s * repSpeed;
    
    normWidth(k) = normalisedWidth;

    %-----------------------------------------------------------------------
    % Classify vehicle based on normalisedWidth
    %-----------------------------------------------------------------------
    if normalisedWidth < 3
        vehClass = 'Motorbike/Bicycle';
    elseif normalisedWidth < 9.5
        vehClass = 'Car/Minibus';
    else
        vehClass = 'Bus/Truck';
    end

    vehicleClasses{k} = vehClass;


end 
hold off;


%% Print Final Classification Results (including True Speed and Normalised Width)

fprintf('\n==========================================================================================\n');
fprintf('                                     FINAL RESULTS\n');
fprintf('==========================================================================================\n\n');

fprintf('Total Vehicles Detected: %d\n\n', numVehicles);

% Column headers with improved spacing
fprintf('%-8s %-15s %-15s %-15s %-15s %-12s %-12s\n', ...
    'Vehicle', 'Speed (m/s)', 'Speed (km/h)','Time. Width (s)', 'Norm. Width (m)', 'Direction', 'Class');
fprintf('%s\n', repmat('-', 1, 100));  % Horizontal separator


% Print data for each vehicle with improved spacing
for k = 1:numVehicles
    fprintf('%-8d %-15.2f %-15.2f %-15.2f %-15.2f %-12s %-12s\n', ...
        k, allSpeeds(k),allSpeeds(k)*3.6, detWidth(k), normWidth(k), direction, vehicleClasses{k});
end


