import numpy as np
import vehiclemodel
import matplotlib.pyplot as plt

# Simulator function
def get_simulation(x=0, vx=10, y=0, vy=0, z=0.715, vz=0, r=0, vr=0, p=0, vp=0, yaw=0, vyaw=0, T=1200., amplitude=0.1,
                   frequency=1, sim_duration=200):
    """
    Parameters

    :param x:   Initial x position of the vehicle
    :param vx:  Initial x velocity of the vehicle
    :param y:   Initial y position of the vehicle
    :param vy:  Initial y velocity of the vehicle
    :param z:   Initial z position of the vehicle
    :param vz:  Initial z velocity of the vehicle
    :param r:   Initial roll angle of the vehicle
    :param vr:  Initial angular velocity of the vehicle's roll
    :param p:   Initial pitch angle of the vehicle
    :param vp:   Initial angular velocity of the vehicle's pitch
    :param yaw:     Initial yaw angle of the vehicle
    :param vyaw:    Initial angular velocity of the vehicle's yaw
    :param T:   Throttle control input; represents the torque applied to the wheels
    :param amplitude:   Amplitude of the sinusoidal steering angle
    :param frequency:   Frequency of the sinusoidal steering angle
    :param sim_duration:    Duration of the simulation in milliseconds
    :return:    _run: numpy.ndarray of the vehicle state at each timestep
    :param:sim_dt:    Time step used in the simulation, assumed to be 1 ms
    :param:state:     Array representing the initial state of the vehicle, including position, velocity, and orientation.
    :param:controls:  Array storing the control inputs for each time step, including throttle and steering angle.
    :param:t:         Array representing time points throughout the simulation, generated using np.linspace.
    :param:delta:     Array representing the sinusoidal steering angle, calculated based on amplitude, frequency, and t.
    :param:mu:        Array representing the friction coefficients for the vehicle's wheels.
    :param:_run:      Array storing the simulation results, including vehicle state variables such as position, velocity, and orientation.

    """

    """
    Returns
    -------
    _run: numpy.ndarray of the vehicle state at each timestep
          shape: (sim_duration, 12)
    """

    sim_dt = 0.001  # in s, assuming a timestep of 1 ms
    sim_duration = int(sim_duration / sim_dt)  # Convert sim_duration from milliseconds to timesteps

    # Vehicle initial state
    state = np.array([x, vx, y, vy, z, vz, r, vr, p, vp, yaw, vyaw]).astype(np.double)

    # Initialize the controls array
    controls = np.zeros((sim_duration, 5))

    # Time array for the simulation period
    t = np.linspace(0, sim_dt * sim_duration, sim_duration)

    # Sinusoidal steering angle
    delta = amplitude * np.sin(2 * np.pi * frequency * t)

    # Set controls for each timestep
    for i in range(sim_duration):
        controls[i] = [
            T,  # Constant throttle for all wheels
            T,  # Constant throttle for all wheels
            0,  # Rear wheels do not contribute to steering
            0,  # Rear wheels do not contribute to steering
            delta[i]  # Sinusoidal steering angle
        ]

    mu = np.array([1., 1., 1., 1.])  # Constant friction coefficients

    # Run sim
    _run = vehiclemodel.run(sim_duration, sim_dt, state, controls, mu)

    # Drop the first column (time)
    _run = _run[:, 1:]

    # Shape: (sim_duration, 12).
    #        Rows format: x vx y vy z vz roll droll pitch dpitch yaw dyaw
    _run = _run[:, 0:12]

    return _run

# Visualization function
def plot_trajectory(demo_state_trajectory):
    t = np.linspace(0, 0.001 * len(demo_state_trajectory), len(demo_state_trajectory))  # Time array in seconds
    plt.figure(figsize=(10, 12))  # Increase the figure size for better visibility

    plt.subplot(411)  # Change to a 4-row plot configuration
    plt.plot(t, demo_state_trajectory[:, 0], label='x')
    plt.plot(t, demo_state_trajectory[:, 2], label='y')
    plt.plot(t, demo_state_trajectory[:, 4], label='z')
    plt.ylabel('Position (meters)')
    plt.legend()

    plt.subplot(412)
    plt.plot(t, demo_state_trajectory[:, 1], label='vx')
    plt.plot(t, demo_state_trajectory[:, 3], label='vy')
    plt.plot(t, demo_state_trajectory[:, 5], label='vz')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    plt.subplot(413)
    plt.plot(t, demo_state_trajectory[:, 6], label='roll')
    plt.plot(t, demo_state_trajectory[:, 8], label='pitch')
    plt.plot(t, demo_state_trajectory[:, 10], label='yaw')
    plt.ylabel('Angle (radians)')
    plt.legend()

    plt.subplot(414)  # New subplot for the trajectory
    plt.plot(demo_state_trajectory[:, 0], demo_state_trajectory[:, 2], label='Trajectory')
    plt.xlabel('x Position (meters)')
    plt.ylabel('y Position (meters)')
    plt.legend()
    plt.title('Vehicle Trajectory')

    plt.tight_layout()  # Adjust subplots to fit into the figure area.
    plt.show()

# Main Execution
if __name__ == "__main__":
    # Drive the vehicle in a sinusoidal trajectory
    demo_state_trajectory = get_simulation(vx=15, amplitude=0.1, frequency=0.01, sim_duration=200)
    plot_trajectory(demo_state_trajectory)
