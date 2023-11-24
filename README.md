
# Visual Servoing Implementation for a 2-DOF Robot

This repository contains the implementation of a visual servoing algorithm for a 2-DOF robot.

## Steps

### Step 1: Object Placement and Image Capture

1. **Spawn Object:** 
   - Spawn the object created in the previous homework on the ground within the robot's workspace.

2. **Move Robot:**
   - Move the robot via the position controller to ensure the entire object is visible in the image.

3. **Image Capture:**
   - Capture an image and obtain the coordinates of the 4 circle centers.

4. **Output:**
   - Present the captured image from the virtual camera.
   - Display the detected circle centers.
  

### Step 2: Robot Relocation and Image Capture

1. **Relocate Robot:**
   - Move the robot to a different location using the position controller.

2. **Ensure Visibility:**
   - Ensure the entire object is still visible by the virtual camera.

3. **New Image Capture:**
   - Capture a new image and obtain the coordinates of the 4 circle centers.

4. **Output:**
   - Present the new captured image from the virtual camera.
   - Display the detected circle centers in the new location.


### Step 3: Visual Servoing Algorithm Implementation 

1. **Algorithm Implementation:**
   - Implement a visual servoing algorithm using the four point features (centers of the circles).

2. **Controller Switch:**
   - Switch the controller to velocity mode to execute the velocity command.

3. **Image Jacobian:**
   - In the image Jacobian, use f and Z, both set to 1.

4. **Note:**
   - All parameters are multiplied with a gain (lambda) in this 2D example and do not have a direct effect.

5. **Output:**
   - Record the locations (x, y coordinates) of all features over time during visual servoing.
   - Plot the trajectories of the features in the XY plane.

**Note:** Adjust parameters, gains, and other specifics based on your implementation details.
