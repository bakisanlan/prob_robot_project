# Robot Motion Model Simulator GUI

A MATLAB-based interactive GUI for visualizing and comparing probabilistic robot motion models with support for multiple trajectories and obstacle environments.

## Overview

This simulator implements two classic probabilistic motion models from robotics:
- **Dead Reckoning (Velocity) Model**: Based on control velocities (v, ω)
- **Odometry Model**: Based on odometry pose measurements

The GUI allows real-time visualization of particle propagation with configurable noise parameters and supports both batch and live simulation modes.

## Features

### Motion Models
- **Dead Reckoning Model**: 6 noise parameters (α1-α6) for velocity-based motion
- **Odometry Model**: 4 noise parameters (α1-α4) for pose-based motion
- Ground truth trajectory visualization
- Sample-based uncertainty representation

### Trajectories
1. **Circular**: 8-step circular path
2. **Rectangle**: Simple rectangular path
3. **Rectangle with Obstacles**: Rectangular path with 2 square obstacles (4×4m)
   - Obstacle 1: centered at (2, 4)
   - Obstacle 2: centered at (14, 4)
   - Resolution: 0.5m occupancy grid

### Simulation Modes
- **Batch Mode**: Run entire simulation instantly, display all samples at once
- **Live Mode**: Step-by-step visualization with configurable pace
  - Pause/Resume capability
  - Reset to initial state
  - Adjustable simulation speed (pace in seconds)

### Visualization
- Progressive ground truth plotting (live mode)
- Particle cloud visualization (samples vanish when colliding with obstacles)
- Occupancy map overlay (obstacle environments)
- Monitor-independent GUI scaling

## Requirements

- MATLAB R2019b or later
- Robotics System Toolbox (for occupancyMap)
- Statistics and Machine Learning Toolbox (for random sampling)

## Installation

1. Clone or download the repository to your local machine:
   ```
   c:\Users\[username]\Desktop\github_repos\prob_robot_project\
   ```

2. Ensure all required MATLAB toolboxes are installed

3. Add the project folder to your MATLAB path

## Usage

### Quick Start

1. Open MATLAB and navigate to the project directory

2. Run the main GUI:
   ```matlab
   robotMotionModelGUI
   ```

3. The GUI window will open with the MODEL panel active

### Configuration Steps

1. **Select Motion Model**
   - Choose between "Dead Reckoning" or "Odometry"
   - Appropriate noise parameters (sliders) will appear

2. **Choose Trajectory**
   - Select from dropdown: Circular, Rectangle, or Rectangle with Obstacles

3. **Set Parameters**
   - Number of Samples: Enter desired particle count (default: 1000)
   - Alpha Parameters: Adjust noise levels using sliders (range: 0-0.01)

4. **Run Simulation**
   - **Batch Mode**: Simply click "Run" button
   - **Live Mode**: 
     - Check "Live Simulation" checkbox
     - Set pace (delay between steps in seconds)
     - Click "Run" to start
     - Use "Stop" to pause, "Continue" to resume, "Reset" to restart

### Panel Navigation

The GUI has three main panels (top buttons):
- **MODEL**: Motion model configuration and simulation
- **SENSORS**: (Placeholder for future sensor models)
- **GNC**: (Placeholder for future guidance/navigation/control)

## File Structure

```
prob_robot_project/
├── robotMotionModelGUI.m                  # Main GUI application
├── sampleDeadReckoningMotionModel.m       # Dead reckoning motion model
├── sampleOdometryMotionModel.m            # Odometry motion model
├── GTdeadReckoningMotionModel.m           # Ground truth dead reckoning
├── GTodometryMotionModel.m                # Ground truth odometry
├── createOccupancyMapWithObstacles.m      # Obstacle map generator
└── README.md                              # This file
```

## Key Functions

### Motion Models

**Dead Reckoning Model**
```matlab
x_next = sampleDeadReckoningMotionModel(x, u, alpha, dt)
x_next = sampleDeadReckoningMotionModel(x, u, alpha, dt, occMap)
```
- `x`: Current pose [x; y; theta]
- `u`: Control [v; w] (velocities)
- `alpha`: 6×1 noise parameters
- `dt`: Time step
- `occMap`: (optional) Occupancy map for collision detection

**Odometry Model**
```matlab
x_next = sampleOdometryMotionModel(x, u, alpha)
x_next = sampleOdometryMotionModel(x, u, alpha, occMap)
```
- `x`: Current pose [x; y; theta]
- `u`: [6×1] Odometry poses [x̄, ȳ, θ̄, x̄', ȳ', θ̄']
- `alpha`: 4×1 noise parameters
- `occMap`: (optional) Occupancy map for collision detection

## Technical Details

### Noise Parameters

**Dead Reckoning (6 parameters)**
- α1, α2: Translational velocity noise
- α3, α4: Rotational velocity noise
- α5, α6: Additional drift noise

**Odometry (4 parameters)**
- α1: Rotation-to-rotation noise
- α2: Translation-to-rotation noise
- α3: Translation-to-translation noise
- α4: Rotation-to-translation noise

### Collision Handling

When obstacles are present:
- Samples that collide with obstacles return `[NaN; NaN; NaN]`
- NaN values are not plotted, causing particles to "vanish"
- Provides realistic visualization of obstacle constraints

### GUI Scaling

The GUI automatically scales to fit different monitor sizes:
- Maximum size: 1200×700 pixels (reference)
- Adapts to 90% of screen width, 85% of screen height
- Maintains 1200:700 aspect ratio
- Centers on screen

## Tips

1. **Performance**: Reduce sample count for faster simulation in live mode
2. **Noise Tuning**: Start with default values (0.001) and adjust as needed
3. **Visualization**: Use live mode with pace ~0.1s for smooth animation
4. **Obstacles**: Try Rectangle with Obstacles trajectory to see collision effects

## Troubleshooting

**GUI doesn't appear**
- Check MATLAB version (R2019b+)
- Ensure project folder is in MATLAB path

**Occupancy map errors**
- Verify Robotics System Toolbox is installed
- Check: `ver('robotics')`

**Slow simulation**
- Reduce number of samples
- Increase pace value in live mode
- Use batch mode for large sample counts

## Future Enhancements

- Sensor models (range, bearing)
- Particle filter implementation
- Custom trajectory designer
- Multi-robot scenarios
- Data export functionality

## References

Based on probabilistic motion models from:
- Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

## License

This project is for educational and research purposes.

## Contact

For questions or contributions, please refer to the project repository.
