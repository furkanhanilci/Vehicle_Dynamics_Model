# Vehicle_Dynamics_Model


- [Parameters](#parameters)
- [How to Run](#how-to-run)




#### Parameters 

- **Gravity (gravity):** Represents the gravitational acceleration, commonly denoted as 'g', measured in meters per second squared (m/s²). It affects the weight of the vehicle and influences its dynamics, especially during acceleration, braking, and cornering.

- **Air Density (rho_air):** Denotes the density of air, measured in kilograms per cubic meter (kg/m³). Air density affects aerodynamic forces acting on the vehicle, influencing its speed and fuel consumption.

- **Front Axle Distance from CoG (lf):** Indicates the distance from the vehicle's center of gravity (CoG) to the front axle, measured in meters. This parameter defines the longitudinal weight distribution of the vehicle, affecting its handling characteristics.

- **Rear Axle Distance from CoG (lr):** Represents the distance from the vehicle's center of gravity (CoG) to the rear axle, measured in meters. Similar to 'lf', this parameter affects the longitudinal weight distribution and hence the vehicle's handling.

- **Track Width (lw):** Denotes the distance between the centerlines of the vehicle's left and right wheels, measured in meters. Track width influences the vehicle's stability during cornering and its resistance to rollover.

- **Wheel Radius (r0):** Refers to the radius of the vehicle's wheels, measured in meters. Wheel radius affects the vehicle's speed, acceleration, and overall performance.

- **Effective Radius of Wheels (r_eff):** Represents the effective radius of the wheels, often slightly smaller than the actual radius due to tire deformation under load. It is measured in meters and influences the vehicle's speed and acceleration.

- **Height of Center of Gravity (h_CoG):** Denotes the height of the vehicle's center of gravity (CoG) above the ground, measured in meters. This parameter affects the vehicle's stability and susceptibility to rollover.

- **Vehicle Mass (mass):** Represents the total mass of the vehicle, including occupants, cargo, and other components, measured in kilograms (kg). Vehicle mass influences its acceleration, braking, and overall dynamics.

- **Suspension Mass (mass_susp):** Indicates the mass of the vehicle's suspension system, measured in kilograms (kg). Suspension mass affects the vehicle's ride comfort, handling, and response to road irregularities.

- **Moment of Inertia around Z-axis (Iz):** Represents the moment of inertia of the vehicle around its vertical (Z) axis, measured in kilograms square meters (kg·m²). This parameter determines the vehicle's resistance to rotation about the vertical axis, affecting its stability and handling.

- **Moment of Inertia of Wheels (Ir):** Denotes the moment of inertia of the vehicle's wheels, measured in kilograms square meters (kg·m²). It affects the rotational dynamics of the wheels and influences the vehicle's acceleration and handling.

- **Moment of Inertia around X-axis (Ix):** Represents the moment of inertia of the vehicle around its longitudinal (X) axis, measured in kilograms square meters (kg·m²). This parameter affects the vehicle's pitching motion, particularly during acceleration and braking.

- **Moment of Inertia around Y-axis (Iy):** Denotes the moment of inertia of the vehicle around its lateral (Y) axis, measured in kilograms square meters (kg·m²). This parameter affects the vehicle's rolling motion, particularly during cornering.

- **Front Suspension Stiffness (k_susp_f):** Indicates the stiffness of the front suspension system, measured in Newtons per meter (N/m). Higher stiffness results in firmer suspension and affects the vehicle's handling, especially during cornering and braking.

- **Front Suspension Damping (d_susp_f):** Represents the damping coefficient of the front suspension system, measured in Newtons per meter per second (N·s/m). Damping controls the rate of suspension oscillations and influences the vehicle's ride comfort and stability.

- **Rear Suspension Stiffness (k_susp_r):** Refers to the stiffness of the rear suspension system, calculated based on the ratio of front to rear axle distances squared and the front suspension stiffness (k_susp_f). It affects the vehicle's rear-end behavior and handling characteristics.

- **Rear Suspension Damping (d_susp_r):** Denotes the damping coefficient of the rear suspension system, calculated based on the ratio of front to rear axle distances squared and the front suspension damping (d_susp_f). It influences the vehicle's rear-end stability and ride comfort.

- **Frontal Surface Area (front_surf):** Represents the frontal surface area of the vehicle, measured in square meters (m²). This parameter affects the aerodynamic drag force experienced by the vehicle, influencing its speed and fuel efficiency.

- **Drag Coefficient (drag_coef):** Denotes the drag coefficient of the vehicle, representing its aerodynamic efficiency. It is a dimensionless quantity and affects the aerodynamic drag force experienced by the vehicle.

- **Aerodynamic Coefficient (aero_coef):** Represents the aerodynamic coefficient of the vehicle, calculated as half the product of air density, frontal surface area, and drag coefficient. It quantifies the aerodynamic drag force acting on the vehicle.

- **Maximum Velocity (V_max):** Indicates the maximum velocity the vehicle can achieve, measured in meters per second (m/s). It represents the upper limit of the vehicle's speed under normal operating conditions.

- **Maximum Angular Velocity (OmegaMax):** Represents the maximum angular velocity of the vehicle, calculated as the maximum velocity divided by the effective radius of the wheels (r_eff). It quantifies the maximum rotational speed of the wheels.

- **Maximum Roll Angle (rollMax):** Denotes the maximum allowable roll angle of the vehicle, calculated as ten times the mathematical constant pi divided by 180 radians. It represents the maximum tilt angle of the vehicle body during cornering.

- **Maximum Pitch Angle (pitchMax):** Represents the maximum allowable pitch angle of the vehicle, calculated as ten times the mathematical constant pi divided by 180 radians. It quantifies the maximum upward or downward tilt angle of the vehicle body.

- **Maximum Motor Torque (C_mot_max):** Indicates the maximum torque that the vehicle's motor can produce, measured in Newton meters (N·m). It represents the upper limit of torque output for the vehicle's propulsion system.

- **Minimum Friction Torque (C_fr_min):** Denotes the minimum friction torque required to overcome resistance in the vehicle's drivetrain, measured in Newton meters (N·m). It represents the lower limit of torque required to maintain vehicle motion.

- **Maximum Acceleration (A_max):** Represents the maximum acceleration that the vehicle can achieve, measured in meters per second squared (m/s²). It quantifies the vehicle's ability to increase its speed within a given time frame.

- **Maximum Deceleration (D_max):** Denotes the maximum deceleration (braking) that the vehicle can achieve, measured in meters per second squared (m/s²). It quantifies the vehicle's ability to decrease its speed within a given time frame.

- **Collision Direction (ColDirection):** Indicates the direction of potential collisions, measured in degrees. This parameter is used to calculate steering angles during collision avoidance maneuvers.

- **Maximum Steering Angle (delta_max):** Represents the maximum allowable steering angle of the vehicle, calculated based on a formula involving the collision direction. It quantifies the maximum angle through which the vehicle's wheels can be turned.

- **Minimum Steering Angle (delta_min):** Denotes the minimum allowable steering angle of the vehicle, calculated based on a formula involving the collision direction. It quantifies the minimum angle through which the vehicle's wheels can be turned.

- **Braking Steering Angle (delta_braq):** Indicates the steering angle applied during braking maneuvers, calculated based on a formula involving the collision direction. It quantifies the steering input required for effective braking and collision avoidance.

- **Tire Parameters (Nested in params::tires):** Represents a set of parameters related to the vehicle's tires, including shape factors, peak values, variations with load, camber, and slip for both longitudinal and lateral forces, longitudinal and lateral slip stiffness, and aligning moment factors and their variations. These parameters collectively define the tire behavior and influence the vehicle's traction, handling, and stability.

- **Shape Factors (Nested in params::tires):** These factors describe the physical shape of the tire, including its tread pattern, sidewall stiffness, and overall geometry. The shape of the tire influences its contact patch with the road surface, which affects traction, grip, and handling characteristics.

- **Peak Values (Nested in params::tires):** Peak values refer to the maximum achievable levels of tire performance in various aspects, such as traction, cornering grip, and braking force. These values are often determined through testing and represent the tire's optimal performance under specific conditions.

- **Variations with Load (Nested in params::tires):** Tire behavior varies depending on the load imposed on them. Load variations affect parameters like tire pressure, contact patch size, and deformation characteristics. Understanding how tires respond to different loads is essential for maintaining optimal performance and safety.

- **Camber Variation (Nested in params::tires):** Camber refers to the angle of the tire relative to the vertical axis of the vehicle. Camber variation describes how tire performance changes with different camber angles. Proper camber adjustment is critical for maximizing tire contact with the road surface and optimizing handling and cornering grip.

- **Slip Variation (Nested in params::tires):** Slip refers to the difference between the speed of the tire's rotation and the speed of the vehicle. Longitudinal slip occurs during acceleration or braking, while lateral slip occurs during cornering. Understanding how tire characteristics vary with slip is crucial for predicting vehicle behavior and optimizing traction and stability.

- **Longitudinal and Lateral Slip Stiffness (Nested in params::tires):** Slip stiffness describes how tire forces vary with changes in slip. Longitudinal slip stiffness relates to acceleration and braking forces, while lateral slip stiffness relates to cornering forces. High slip stiffness indicates a more responsive tire with better grip and handling capabilities.

- **Aligning Moment Factors (Nested in params::tires):**  Aligning moments are generated when the tire is subjected to lateral forces during cornering. Aligning moment factors describe how these moments vary with changes in slip angle and load. Proper alignment and tuning of aligning moment factors are essential for optimizing steering response and cornering stability.

#### Requirements
1. ** libboost-python-dev:**
   - Install the libboost-python-dev package using the following command:
     ```
     sudo apt-get update
     sudo apt-get install libboost-python-dev
     ```
2. ** Confirm Library Location:**
   - Confirm the location of the library using the following command:
     ```
     locate libboost_python38.so
     ```
     ```
     export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/lib
     ```
     
3. ** Check Version Mismatch:**
   - Check for version mismatch using the following command:
     ```
     ls /usr/lib/x86_64-linux-gnu/ | grep libboost_python
     ```
     
#### How to Run

1. **Clone the Repository:**
   - Clone the repository to your local machine using the following command:
     ```
     git clone https://github.com/furkanhanilci/Vehicle_Dynamics_Model.git
     ```

2. **Run the setup.py file:**
   - Run the setup.py file to install the required dependencies:
     ```
     python3 setup.py build
     python3 setup.py install
     ```
   
3. **Run the Simulation File:**
   - Run the simulation file to execute the vehicle dynamics model:
     ```
     python3 simulation.py  
     ```    
     