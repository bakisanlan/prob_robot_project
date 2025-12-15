%% EKF Localization GUI Demonstration Script
% This script demonstrates how to use the EKF localization feature in the GUI
%
% Workflow:
%   1. MODEL Panel: Configure and run motion model simulation
%   2. SENSORS Panel: Add landmarks for EKF measurements
%   3. GNC Panel: Run EKF localization with uncertainty visualization
%
% Author: Robot Motion Model Simulator Team
% Date: December 2025

clear; clc; close all;

fprintf('========================================\n');
fprintf('EKF Localization GUI Demo\n');
fprintf('========================================\n\n');

%% Step 1: Launch GUI
fprintf('Step 1: Launching GUI...\n');
robotMotionModelGUI();
pause(2);  % Wait for GUI to initialize

%% Step 2: Instructions
fprintf('\n========================================\n');
fprintf('INSTRUCTIONS FOR EKF DEMO:\n');
fprintf('========================================\n\n');

fprintf('MODEL Panel (Current Active):\n');
fprintf('  1. Select "Dead Reckoning" or "Odometry" model\n');
fprintf('  2. Choose trajectory: "Rectangle with Obstacles" (recommended)\n');
fprintf('  3. Set Number of Samples: 1 (EKF uses single estimate, not particles)\n');
fprintf('  4. Adjust Alpha parameters for motion noise\n');
fprintf('  5. Uncheck "Live Simulation" for faster batch mode\n');
fprintf('  6. Click "Run" button\n\n');

fprintf('SENSORS & LANDMARKS Panel:\n');
fprintf('  1. Click "Sensors&Landmarks" tab\n');
fprintf('  2. Add landmarks using either method:\n');
fprintf('     a) Manual Entry: Enter X, Y coordinates (e.g., 5, 3)\n');
fprintf('     b) Click to Add: Click on plot to place landmarks\n');
fprintf('  3. Recommended: Add 4-6 landmarks spread across the workspace\n');
fprintf('  4. Example landmark positions for Rectangle trajectory:\n');
fprintf('     - Landmark 1: (5, 3)\n');
fprintf('     - Landmark 2: (15, 3)\n');
fprintf('     - Landmark 3: (15, 7)\n');
fprintf('     - Landmark 4: (5, 7)\n\n');

fprintf('GNC Panel (EKF Localization):\n');
fprintf('  1. Click "GNC" tab\n');
fprintf('  2. Configure Initial Covariance P₀:\n');
fprintf('     - σ²(x): 0.1 (uncertainty in x position)\n');
fprintf('     - σ²(y): 0.1 (uncertainty in y position)\n');
fprintf('     - σ²(θ): 0.05 (uncertainty in heading)\n');
fprintf('  3. Configure Measurement Noise R:\n');
fprintf('     - σ²(r): 0.05 (range measurement noise)\n');
fprintf('     - σ²(φ): 0.01 (bearing measurement noise)\n');
fprintf('  4. Visualization options:\n');
fprintf('     - [✓] Show EKF Estimate (blue trajectory)\n');
fprintf('     - [✓] Show Uncertainty Ellipsoid (shows position uncertainty)\n');
fprintf('     - [✓] Show Landmark Detections (magenta circles)\n');
fprintf('     - Confidence: 3 (for 3σ = 99.7%% confidence)\n');
fprintf('  5. Check "Live Update" for step-by-step visualization\n');
fprintf('  6. Click "Run EKF" button\n\n');

fprintf('========================================\n');
fprintf('INTERPRETING RESULTS:\n');
fprintf('========================================\n\n');

fprintf('Visualization Elements:\n');
fprintf('  - Green circles: Ground truth robot trajectory\n');
fprintf('  - Blue line: EKF estimated trajectory\n');
fprintf('  - Blue dashed ellipse: Position uncertainty (3σ)\n');
fprintf('  - Purple stars: All known landmarks\n');
fprintf('  - Magenta circles: Currently detected landmarks\n');
fprintf('  - Red points: Motion model samples (if > 1 sample)\n\n');

fprintf('Uncertainty Ellipsoid:\n');
fprintf('  - Size indicates position uncertainty\n');
fprintf('  - Ellipse shrinks when landmarks are detected (measurement update)\n');
fprintf('  - Ellipse grows during dead reckoning (prediction step)\n');
fprintf('  - Orientation shows correlation between x and y errors\n\n');

fprintf('Expected Behavior:\n');
fprintf('  - EKF estimate (blue) should closely follow ground truth (green)\n');
fprintf('  - Uncertainty grows when no landmarks are visible\n');
fprintf('  - Uncertainty shrinks when landmarks are detected and measured\n');
fprintf('  - With good landmarks, ellipsoid should stay small\n\n');

fprintf('========================================\n');
fprintf('TROUBLESHOOTING:\n');
fprintf('========================================\n\n');

fprintf('If EKF shows errors:\n');
fprintf('  1. "Run MODEL simulation first" → Go to MODEL panel and click Run\n');
fprintf('  2. "No landmarks defined" → Go to SENSORS panel and add landmarks\n');
fprintf('  3. Large estimation errors → Check:\n');
fprintf('     - Alpha parameters (motion noise) not too large\n');
fprintf('     - Measurement noise R values are reasonable\n');
fprintf('     - Landmarks are visible during trajectory\n');
fprintf('  4. No uncertainty ellipsoid → Check "Show Uncertainty Ellipsoid"\n\n');

fprintf('========================================\n');
fprintf('READY TO START!\n');
fprintf('========================================\n\n');

fprintf('Follow the instructions above to run the EKF demo.\n');
fprintf('The GUI is now open and waiting for your input.\n\n');
