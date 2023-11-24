
## Overview

This repository contains MATLAB code focusing on analyzing the minimum singular value, volume of the ellipsoid in the wrench space, and grasp isotropy index for a 2D grasping scenario.

## Steps to Reproduce the Analysis

### 1. Literature Review

- Read the first four pages of the paper "Grasp quality measures: review and performance" to gain insights into different grasp quality metrics.

### 2. Grasp Matrix Derivation

- Derive a grasp matrix specifically for the planar case, where the twist is represented by a 3-dimensional vector.

### 3. MATLAB Function for Grasp Matrix Generation

- Implement a MATLAB function to generate the grasp matrix for a given object center and contact locations, considering the hard finger contact model.

### 4. Grasp Quality Metrics Analysis

- Utilize the implemented grasp matrix function to analyze the grasp quality metrics for a specific object shape.
- Sample the location of the 5th force vector in the planar scenario.
- Evaluate three grasp quality metrics:
  1. **3.1.1 Minimum Singular Value of G:**
      - Calculate the minimum singular value of the grasp matrix for the current configuration.
      - Move the 5th vector along the object edges in 0.1 cm increments, recording the grasp quality values.
      - Plot the grasp quality value vs. vector location.
      - Identify and visualize the best location for the 5th vector on the object.
  2. **3.1.2 Volume of the Ellipsoid in the Wrench Space:**
      - Repeat the process for the grasp quality metric in 3.1.2.
  3. **3.1.3 Grasp Isotropy Index:**
      - Repeat the process for the grasp quality metric in 3.1.3.
- Compare the locations of the 5th vector obtained from the different grasp quality metrics and discuss any differences observed.

## Instructions

1. **Run the Code:**
   - Open the MATLAB script file `VBM_Assignment2.m`.
   - Execute the script to run the analysis.

2. **Results:**
   - The code generates plots for the minimum singular value (QMSV), volume of the ellipsoid in the wrench space (QVEW), and grasp isotropy index (QGII) for different object locations and orientations.

3. **Customization:**
   - You can uncomment specific lines (79-98 for QMSV, 80-99 for QVEW, 81-100 for QGII) to selectively visualize the desired metric.

4. **Plotting Functions:**
   - The script includes plotting functions to visualize the results. You can find functions for QMSV (`Qmsv_value`), QVEW (`Qvew_value`), and QGII (`Qgii_value`).

## File Structure

- `VBM_Assignment2.m`: Main MATLAB script containing the implementation.
- `partial_grasp_matrix.m`: Function to calculate the partial grasp matrix for a given contact point.

## Note

- The code is configured to analyze the grasping metrics for different object positions and orientations.
- Customize the code based on specific requirements or experimentations.

## Instructions for Plotting

1. **QMSV Plot:**
   - Uncomment lines 79, 98, 120, 139.

2. **QVEW Plot:**
   - Uncomment lines 80, 99, 121, 140.

3. **QGII Plot:**
   - Uncomment lines 81, 100, 122, 141.


