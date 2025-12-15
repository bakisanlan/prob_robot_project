# Copilot Instructions for Robot Motion Model Simulator

## Project Overview
MATLAB-based probabilistic robot motion model simulator implementing Thrun's motion models from *Probabilistic Robotics*. The GUI supports both particle-based uncertainty propagation and Extended Kalman Filter (EKF) localization with range-bearing landmark measurements.

## Architecture

### Directory Structure
```
prob_robot_project/
├── robotMotionModelGUI.m          # Main entry point (orchestrator)
├── demoEKF_GUI.m                  # EKF demonstration script with instructions
├── testEKF.m                      # Standalone EKF test script
├── gui/
│   ├── panels/                    # Panel UI creation
│   │   ├── createModelPanel.m     # MODEL panel components
│   │   ├── createSensorsPanel.m   # SENSORS & LANDMARKS panel (2D Lidar + Landmarks)
│   │   └── createGNCPanel.m       # GNC panel (EKF localization)
│   ├── callbacks/                 # Event handlers
│   │   ├── ModelCallbacks.m       # MODEL panel callback class
│   │   ├── SensorsCallbacks.m     # SENSORS & LANDMARKS panel callback class
│   │   └── GNCCallbacks.m         # GNC panel callback class (EKF)
│   └── utils/                     # GUI utilities
│       └── resizeGUI.m            # Window resize handler (auto-adjusts all UI elements)
├── functions/                     # Motion model & EKF implementations
│   ├── sample*.m                  # Noisy sampling functions
│   ├── GT*.m                      # Ground truth propagation
│   ├── pdf*.m                     # Probability density functions
│   ├── EKF_localization.m         # Extended Kalman Filter main function
│   ├── predictMeasurement.m       # Measurement model prediction with Jacobian
│   ├── detectLandmarksFromScan.m  # Ray-circle intersection for landmark detection
│   └── associateMeasurements.m    # Data association for EKF update
└── trajectories/                  # Trajectory generators
    ├── generateCircularTrajectory.m
    ├── generateRectangleTrajectory.m
    └── generateRectangleObstacleTrajectory.m
```

### Core Components
- **[robotMotionModelGUI.m](../robotMotionModelGUI.m)**: Main entry point; creates figure, axes, panel buttons, and orchestrates sub-components. Run with `robotMotionModelGUI` in MATLAB.
- **[gui/panels/](../gui/panels/)**: Panel creation functions returning UI components and handles
- **[gui/callbacks/ModelCallbacks.m](../gui/callbacks/ModelCallbacks.m)**: Handle class managing all MODEL panel interactions and simulation state
- **[gui/callbacks/SensorsCallbacks.m](../gui/callbacks/SensorsCallbacks.m)**: Handle class managing 2D Lidar sensor configuration, raycasting, and landmark management
- **[gui/callbacks/GNCCallbacks.m](../gui/callbacks/GNCCallbacks.m)**: Handle class managing EKF localization, uncertainty visualization, and state estimation
- **[gui/utils/resizeGUI.m](../gui/utils/resizeGUI.m)**: Automatic window resize handler that proportionally adjusts all UI elements based on figure dimensions
- **[functions/](../functions/)**: Motion model and EKF implementations following consistent patterns
- **[trajectories/](../trajectories/)**: Standalone trajectory generation functions

### Data Flow
**Particle-based Simulation (MODEL Panel):**
1. `robotMotionModelGUI.m` creates figure and instantiates panel creators
2. `createModelPanel.m` returns UI handles to `ModelCallbacks` class
3. `ModelCallbacks` handles user interactions and manages simulation state
4. Trajectories generated via `trajectories/generate*.m` functions
5. Ground truth computed via `GT*` functions, particles via `sample*` functions
6. Visualization updates axes with particle cloud and ground truth path

**EKF Localization (GNC Panel):**
1. Requires MODEL panel simulation data (trajectory, controls, ground truth)
2. Requires SENSORS panel landmarks (purple stars on plot)
3. `GNCCallbacks.runEKF()` executes EKF main loop:
   - **Prediction**: Uses motion model Jacobians from `EKF_localization.m`
   - **Detection**: Uses `detectLandmarksFromScan.m` with ray-circle intersection
   - **Association**: Matches detections to landmarks via `associateMeasurements.m`
   - **Update**: Computes measurement Jacobian and updates state/covariance
4. Visualization shows EKF estimate (blue line) and uncertainty ellipsoid (blue dashed)

## Key Conventions

### Pose Representation
All poses are `[3×1]` column vectors: `[x; y; theta]` where theta is in radians, wrapped to `[-π, π]` via `wrapToPi()`.

### Motion Model Interfaces
```matlab
% Dead Reckoning (velocity-based)
x_next = sampleDeadReckoningMotionModel(x, u, alpha, dt)        % u = [v; w]
x_next = sampleDeadReckoningMotionModel(x, u, alpha, dt, occMap) % with collision

% Odometry (pose-based)  
x_next = sampleOdometryMotionModel(x, u, alpha)                 % u = [6×1] odom poses
x_next = sampleOdometryMotionModel(x, u, alpha, occMap)         % with collision
```

### Collision Handling Pattern
When `occMap` is provided, functions return `[NaN; NaN; NaN]` for collisions. NaN samples are automatically excluded from plots. Always check with:
```matlab
if checkOccupancy(occMap, [x_new, y_new])
    x_next = [NaN; NaN; NaN];
end
```

### Alpha Parameters
- **Dead Reckoning**: 6 parameters `alpha(1:6)` controlling velocity noise
- **Odometry**: 4 parameters `alpha(1:4)` controlling rotation/translation noise
- Default range: `[0, 0.01]` with typical value `0.001`

### 2D Lidar Sensor & Raycasting
The SENSORS & LANDMARKS panel uses MATLAB's built-in `lidarScan` and `raycast` functions:
```matlab
% Perform raycasting against occupancy map
poseRow = [x, y, theta];  % Row vector format
ranges = raycast(occMap, poseRow, angles, maxRange);

% Create lidar scan
scan = lidarScan(ranges, angles);
```
**Configurable parameters:**
- `maxRange`: Maximum sensor range in meters (default: 10m)
- `fov`: Field of view in degrees (default: 90°)
- `resolution`: Angular resolution in degrees (default: 2°)
- `rangeNoise`: Range measurement std deviation (default: 0.1m)
- `angularNoise`: Angular measurement std deviation (default: 0.5°)

**Raycasting:**
- Uses built-in `raycast(occMap, pose, angles, maxRange)` for obstacle detection
- Automatically detects obstacles from the robot's current pose and orientation
- Integrates with MODEL panel's occupancy map (e.g., Rectangle with Obstacles trajectory)

### Landmarks Management
Landmarks stored as `Nx3` matrix: `[id, x, y]`
- **Add Manually**: Enter X, Y coordinates directly
- **Add by Click**: Use `ginput(1)` to select position on plot
- **Remove Selected**: Delete selected landmark from list
- **Clear All**: Remove all landmarks
- **Visualization**: Red star markers with ID labels, excluded from legend using `'HandleVisibility', 'off'`

## Dependencies
- MATLAB R2019b+
- Robotics System Toolbox (`occupancyMap`, `checkOccupancy`, `lidarScan`)
- Statistics and Machine Learning Toolbox (`randn`)

## GUI Implementation Notes
- Uses pixel-based positioning with reference size 1200×700, scales to screen
- Panel switching hides/shows UI groups via `Visible` property
- Live simulation uses `pause()` and `drawnow` for animation
- `ModelCallbacks` class encapsulates simulation state as properties
- `SensorsCallbacks` class manages lidar configuration and visualization
- Panel creators return `[components, handles]` for visibility and callback binding
- Cross-panel data sharing via `setappdata/getappdata` on figure handle

### Auto-Resize System
**[gui/utils/resizeGUI.m](../gui/utils/resizeGUI.m)** automatically adjusts all UI elements when window is resized:
- Uses `SizeChangedFcn` callback on main figure
- Updates positions proportionally based on current figure dimensions
- Separate update functions for each panel: `updateModelPanelPositions()`, `updateSensorsPanelPositions()`, `updateGNCPanelPositions()`
- Finds UI elements using `Tag` properties for reliable identification
- All UI components must have unique `Tag` properties for resize to work correctly

**Adding new UI elements:**
1. Assign a unique `Tag` property to the UI control in `createXxxPanel.m`
2. Add position update code in corresponding `updateXxxPanelPositions()` function in `resizeGUI.m`
3. Use relative positioning (e.g., `figHeight*0.50`, `panelWidth*0.45`) for proportional scaling

**Example:**
```matlab
% In createModelPanel.m
handles.editSamples = uicontrol(...
    'Tag', 'editSamples', ...  % Unique tag
    'Position', [panelX, figHeight*0.594, panelWidth*0.4, figHeight*0.036]);

% In resizeGUI.m updateModelPanelPositions()
editSamp = findobj(src, 'Tag', 'editSamples');
if ~isempty(editSamp)
    set(editSamp, 'Position', [panelX, figHeight*0.594, panelWidth*0.4, figHeight*0.036]);
end
```

## EKF Localization System

### Overview
The GNC panel implements Extended Kalman Filter (EKF) localization with range-bearing measurements to circular landmarks. Unlike the particle-based approach in MODEL panel, EKF maintains a single state estimate with Gaussian uncertainty.

### Key Components

**[functions/EKF_localization.m](../functions/EKF_localization.m)**:
- Main EKF filter implementing prediction and update steps
- Supports both Dead Reckoning and Odometry motion models
- Computes motion Jacobians (G) and process noise (Q)
- Handles measurement updates with Jacobian (H)

**[functions/predictMeasurement.m](../functions/predictMeasurement.m)**:
- Predicts range-bearing measurements to landmarks
- Filters landmarks based on sensor limits (maxRange, FOV)
- Returns measurement Jacobian H for EKF update
- Output: `[z_pred, H, validLandmarks]`

**[functions/detectLandmarksFromScan.m](../functions/detectLandmarksFromScan.m)**:
- Detects circular landmarks using ray-circle intersection
- Simulates actual sensor measurements from ground truth pose
- Handles cases where no landmarks are visible (returns empty arrays with correct dimensions)
- Output: `[detectedLandmarks, measurements]` with .ranges, .bearings, .rayIdx, .hitPoints

**[functions/associateMeasurements.m](../functions/associateMeasurements.m)**:
- Matches detected landmarks with predicted measurements
- Uses ID-based data association via `intersect()`
- Returns matched measurements and Jacobians for EKF update
- Early return for empty detections or predictions

### Usage Workflow

1. **MODEL Panel**: Run simulation to generate ground truth trajectory
   - Note: Set "Number of Samples" to 1 for EKF (single estimate, not particles)
   
2. **SENSORS Panel**: Add landmarks (4-6 recommended)
   - Landmarks visualized as purple stars
   - Place strategically for good observability
   
3. **GNC Panel**: Configure and run EKF
   - Set initial covariance P₀ (typically 0.1 for x,y and 0.05 for θ)
   - Set measurement noise R (0.05 for range, 0.01 for bearing)
   - Enable "Show Uncertainty Ellipsoid" to visualize position uncertainty
   - Run with "Live Update" for step-by-step visualization

### Visualization Elements

- **Blue Line**: EKF estimated trajectory
- **Blue Dashed Ellipse**: Position uncertainty (nσ confidence region)
  - Size indicates uncertainty magnitude
  - Shrinks during measurement updates
  - Grows during prediction steps
  - Ellipse eigenvalues from position covariance Σ(1:2, 1:2)
- **Magenta Circles**: Currently detected landmarks
- **Purple Stars**: All known landmarks
- **Green Circles**: Ground truth trajectory

### Parameters

**Initial Covariance P₀**:
- σ²(x): X position variance (default: 0.1)
- σ²(y): Y position variance (default: 0.1)
- σ²(θ): Heading variance (default: 0.05)

**Measurement Noise R**:
- σ²(r): Range measurement variance (default: 0.05)
- σ²(φ): Bearing measurement variance (default: 0.01)

**Sensor Parameters** (from SENSORS panel):
- Max Range: Maximum sensor range (default: 10m)
- FOV: Field of view in degrees (default: 90°)
- Landmark Radius: Circular landmark radius (default: 0.05m)

### EKF Algorithm Details

**Prediction Step**:
```matlab
% Motion model: x_next = f(x, u)
% Jacobian: G = ∂f/∂x
% Process noise: Q = G_u * M * G_u'
mu_pred = f(mu, u)
Sigma_pred = G * Sigma * G' + Q
```

**Update Step** (per detected landmark):
```matlab
% Measurement model: z = h(x, landmark)
% Innovation: y = z_actual - h(mu_pred, landmark)
% Jacobian: H = ∂h/∂x
% Innovation covariance: S = H * Sigma_pred * H' + R
% Kalman gain: K = Sigma_pred * H' * inv(S)
mu = mu_pred + K * y
Sigma = (I - K * H) * Sigma_pred
```

## Adding New Features

### New Panel (e.g., SENSORS or GNC)
1. Create `gui/panels/createNewPanel.m` following `createModelPanel.m` pattern
2. Create `gui/callbacks/NewCallbacks.m` handle class for panel logic
3. Update `robotMotionModelGUI.m` to instantiate and wire up the new panel
4. Update `gui/utils/resizeGUI.m` with position update function `updateNewPanelPositions()`
5. Ensure all UI elements have unique `Tag` properties for auto-resize

### New Motion Model
1. Create `functions/sampleNewModel.m` and `functions/GTnewModel.m`
2. Follow existing signature patterns (pose in, pose out, optional occMap)
3. Add radio button in `createModelPanel.m`
4. Update `ModelCallbacks.runSimulation()` to handle new model

### New Trajectory
1. Create `trajectories/generateNewTrajectory.m`
2. Return `[X_gt, U_list, x0, dt]` (add `occMap` if obstacles needed)
3. Add option to `popupTrajectory` dropdown in `createModelPanel.m`
4. Update `ModelCallbacks.runSimulation()` switch statement

### New Sensor Type
1. Add UI controls in `createSensorsPanel.m` with unique `Tag` properties
2. Add sensor logic methods in `SensorsCallbacks.m`
3. Update `resizeGUI.m` `updateSensorsPanelPositions()` to handle new UI elements
4. Use relative positioning for all new elements (e.g., `figHeight*0.XX`, `panelWidth*0.YY`)
