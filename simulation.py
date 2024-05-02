import numpy
import vehiclemodel
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------------------
def get_simulation(x=0, vx=10, y=0, vy=0, z=0.715, vz=0, r=0, vr=0, p=0, vp=0, yaw=0, vyaw=0, T=1200., amplitude=0.1,
                   frequency=1):
    """
    Returns
    -------
    _run: numpy.ndarray of the vehicle state at each timestep
          shape: (sim_duration=100, 12)
    """

    sim_dt = 0.001  # in s
    sim_duration = 100  # in ms, since dt=0.001s

    # vehicle initial state
    state = numpy.array([x, vx, y, vy, z, vz, r, vr, p, vp, yaw, vyaw]).astype(numpy.double)

    # Initialize the controls array
    controls = numpy.zeros((sim_duration, 5))

    # Time array for the simulation period
    t = numpy.linspace(0, sim_dt * sim_duration, sim_duration)

    # Sinusoidal steering angle
    delta = amplitude * numpy.sin(2 * numpy.pi * frequency * t)

    # Set controls for each timestep
    for i in range(sim_duration):
        controls[i] = [
            T,  # Constant throttle for all wheels
            T,  # Constant throttle for all wheels
            0,  # Rear wheels do not contribute to steering
            0,  # Rear wheels do not contribute to steering
            delta[i]  # Sinusoidal steering angle
        ]

    mu = numpy.array([1., 1., 1., 1.])  # Constant friction coefficients

    # Run sim
    # Note: sim columns: dt x vx y vy z vz r vr p vp yaw vyaw om_fl om_fr om_rl om_rr ax ay sr1 sr2 sr3 sr4 sa1 sa2 sa3 sa4
    _run = vehiclemodel.run(sim_duration, sim_dt, state, controls, mu)

    # Drop the first column (time)
    _run = _run[:, 1:]

    # Shape: (sim_duration, 12).
    #        Rows format: x vx y vy z vz roll droll pitch dpitch yaw dyaw
    _run = _run[:, 0:12]

    return _run


# ---------------------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------------------
def plot_trajectory(demo_state_trajectory):
    t = numpy.linspace(0, 0.1, 100)  # time from 0 to 0.1 seconds
    plt.figure()
    plt.subplot(311)
    plt.plot(t, demo_state_trajectory[:, 0], label='x')
    plt.plot(t, demo_state_trajectory[:, 2], label='y')
    plt.plot(t, demo_state_trajectory[:, 4], label='z')
    plt.ylabel('Position (meters)')
    plt.legend()

    plt.subplot(312)
    plt.plot(t, demo_state_trajectory[:, 1], label='vx')
    plt.plot(t, demo_state_trajectory[:, 3], label='vy')
    plt.plot(t, demo_state_trajectory[:, 5], label='vz')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    plt.subplot(313)
    plt.plot(t, demo_state_trajectory[:, 6], label='roll')
    plt.plot(t, demo_state_trajectory[:, 8], label='pitch')
    plt.plot(t, demo_state_trajectory[:, 10], label='yaw')
    plt.ylabel('Angle (radians)')
    plt.xlabel('Time (s)')
    plt.legend()

    plt.show()


# ---------------------------------------------------------------------------------------
# Main Execution
# ---------------------------------------------------------------------------------------
if __name__ == "__main__":
    # Drive the vehicle in a sinusoidal trajectory
    demo_state_trajectory = get_simulation(amplitude=0.1, frequency=0.5)  # amplitude and frequency can be adjusted
    assert demo_state_trajectory.shape == (100, 12), "Unexpected shape of the simulation output"

    # Plot the simulation results
    plot_trajectory(demo_state_trajectory)
