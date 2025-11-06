function Detection = oscfarDetector(y_power, alpha_OS, k, lagIndices, leadIndices)
    % applies OS-CFAR on a 1D vector (one column) using precomputed reference cell indices.
    
    % Inputs:
    %   y_power     - Input power data (column vector)
    %   alpha_OS    - Scaling factor computed for OS-CFAR (precomputed externally)
    %   k           - Order statistic (k-th smallest value) to be used
    %   lagIndices  - Cell array of lagging reference cell indices for each CUT position
    %   leadIndices - Cell array of leading reference cell indices for each CUT position

    % Output:
    %   Detection  - Binary vector indicating detection (true if detection)
    
    N         = numel(y_power);
    Detection = false(N, 1);
    
    % Process each cell using the precomputed indices
    for i = 1:N
        % Concatenate lagging and leading reference cell values
        refCells = [y_power(lagIndices{i}); y_power(leadIndices{i})];
        % Sort to get the k-th smallest value
        refCellsSorted = sort(refCells, 'ascend');
        kthVal = refCellsSorted(k);
        % Compute threshold and decide detection
        Threshold = alpha_OS * kthVal;
        Detection(i) = y_power(i) > Threshold;
    end
end
