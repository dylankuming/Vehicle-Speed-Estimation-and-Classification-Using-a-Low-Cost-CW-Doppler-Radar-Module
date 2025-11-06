function [P, T, F] = powerSpectrogram(x, fs, frameLength, overlap, win)

% Inputs:
%   x           - signal (vector)
%   fs          - sampling rate (Hz)
%   frameLength - frame size (samples)
%   overlap     - overlap (samples)
%   win         - window (length = frameLength)

% Outputs:
%   P           - power spectrogram (freq bins × frames)
%   T           - time axis (s)
%   F           - frequency axis (Hz)


    hopsize    = frameLength - overlap;
    numFrames  = floor((numel(x) - frameLength) / hopsize) + 1;
    numBins    = frameLength/2 + 1;

    % Preallocate power matrix
    P = zeros(numBins, numFrames);

    for frameIdx = 1:numFrames
        idx   = (frameIdx-1)*hopsize + (1:frameLength);
        frame = x(idx) .* win;
        X     = fft(frame, frameLength);   
        mag   = abs(X(1:numBins));
        P(:,frameIdx) = mag.^2;                     % power spectrum
    end

    % Axes
    T = (0:numFrames-1) * hopsize / fs;             % time axis (s)
    F = (0:numBins-1)   * fs       / frameLength;   % frequency axis (Hz)
end
