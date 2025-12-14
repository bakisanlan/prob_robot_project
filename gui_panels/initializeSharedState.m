function sharedState = initializeSharedState()
    %INITIALIZESHAREDSTATE Create shared state structure for GUI panels
    
    sharedState = struct();
    sharedState.isRunning = false;
    sharedState.isPaused = false;
    sharedState.isFinished = false;
    sharedState.currentStep = 0;
    sharedState.X_samples = [];
    sharedState.X_gt = [];
    sharedState.U_list = [];
    sharedState.U_odom = [];
    sharedState.x0 = [];
    sharedState.dt = 0;
    sharedState.numSamples = 0;
    sharedState.alpha = [];
    sharedState.isDeadReckoning = true;
    sharedState.trajectoryType = 1;
    sharedState.occMap = [];
end
