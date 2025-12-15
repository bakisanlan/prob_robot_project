%% QUICK START GUIDE: EKF Localization in GUI
% This script provides a minimal working example for EKF localization
%
% WORKFLOW:
%   1. Launch GUI
%   2. MODEL Panel: Run simulation (samples=1)
%   3. SENSORS Panel: Add landmarks
%   4. GNC Panel: Run EKF
%
% See demoEKF_GUI.m for detailed instructions
% See EKF_IMPLEMENTATION_SUMMARY.md for complete documentation

%% STEP 1: Launch GUI
robotMotionModelGUI();

%% STEP 2: MODEL Panel (Already active)
% ┌────────────────────────────────────────┐
% │ Motion Model:    [●] Dead Reckoning    │
% │ Trajectory:      Rectangle w/ Obstacles│
% │ Samples:         1  ← IMPORTANT!       │
% │ Alpha params:    Use defaults          │
% │ Live Simulation: [ ] (faster)          │
% │ [Run Button]                           │
% └────────────────────────────────────────┘

fprintf('MODEL PANEL INSTRUCTIONS:\n');
fprintf('1. Select motion model\n');
fprintf('2. Choose trajectory (Rectangle with Obstacles recommended)\n');
fprintf('3. Set "Number of Samples" to 1\n');
fprintf('4. Click RUN button\n\n');

%% STEP 3: SENSORS & LANDMARKS Panel
% ┌────────────────────────────────────────┐
% │ Click: [Sensors&Landmarks] tab         │
% │                                        │
% │ Add Landmarks:                         │
% │   Manual Entry:  X: 5    Y: 3  [Add]  │
% │   Or: [Add by Click]                  │
% │                                        │
% │ Recommended for Rectangle trajectory:  │
% │   - (5, 3)   - (15, 3)                │
% │   - (5, 7)   - (15, 7)                │
% └────────────────────────────────────────┘

fprintf('SENSORS PANEL INSTRUCTIONS:\n');
fprintf('1. Click "Sensors&Landmarks" tab\n');
fprintf('2. Add 4 landmarks at: (5,3), (15,3), (5,7), (15,7)\n');
fprintf('   - Use "Manual Entry" and type coordinates\n');
fprintf('   - OR use "Add by Click" and click on plot\n\n');

%% STEP 4: GNC Panel
% ┌────────────────────────────────────────┐
% │ Click: [GNC] tab                       │
% │                                        │
% │ Initial Covariance P₀:                 │
% │   σ²(x): 0.1    σ²(y): 0.1            │
% │   σ²(θ): 0.05                         │
% │                                        │
% │ Measurement Noise R:                   │
% │   σ²(r): 0.05   σ²(φ): 0.01           │
% │                                        │
% │ Visualization:                         │
% │   [✓] Show EKF Estimate               │
% │   [✓] Show Uncertainty Ellipsoid      │
% │   [✓] Show Landmark Detections        │
% │   Confidence: 3 (99.7%)               │
% │                                        │
% │ [✓] Live Update    Pace: 0.1          │
% │                                        │
% │ [Run EKF Button]                      │
% └────────────────────────────────────────┘

fprintf('GNC PANEL INSTRUCTIONS:\n');
fprintf('1. Click "GNC" tab\n');
fprintf('2. Use default parameters (already set correctly)\n');
fprintf('3. Check all visualization options\n');
fprintf('4. Enable "Live Update" to see step-by-step\n');
fprintf('5. Click RUN EKF button\n\n');

fprintf('EXPECTED RESULT:\n');
fprintf('- Blue line (EKF estimate) follows green circles (ground truth)\n');
fprintf('- Blue dashed ellipse shows position uncertainty\n');
fprintf('- Ellipse shrinks when landmarks detected (magenta circles)\n');
fprintf('- Ellipse grows during dead reckoning\n\n');

fprintf('═══════════════════════════════════════════════\n');
fprintf('READY! Follow the steps above in the GUI.\n');
fprintf('═══════════════════════════════════════════════\n');
