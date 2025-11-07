function alpha_os = solveAlphaOS(PFA, N, k)
    % solveAlphaOS solves for the scaling factor alpha_os used in OS-CFAR.

    % Inputs:
    %   PFA - Desired probability of false alarm
    %   N   - Total number of reference cells (2*RefWindow)
    %   k   - Order statistic index (k-th smallest)
    
    % Output:
    %   alpha_os - Calculated scaling factor for the OS-CFAR threshold
    
    syms alpha_OS
    rhs = k * nchoosek(N, k) * (gamma(alpha_OS + N - k + 1) * gamma(k)) / gamma(alpha_OS + N + 1);
    alpha_os_sol = vpasolve(PFA == rhs, alpha_OS, [0, Inf]);
    if isempty(alpha_os_sol)
        error('No solution found for alpha_os.');
    else
        alpha_os = double(alpha_os_sol);
    end
end