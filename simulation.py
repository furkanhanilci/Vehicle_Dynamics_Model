import numpy
import vehiclemodel

# ---------------------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------------------
def get_simulation(x=0, vx=0, y=0, vy=0, z=0.715, vz=0, r=0, vr=0, p=0, vp=0, yaw=0, vyaw=0, T=1200., delta=0.):

    """
    Returns
    -------
    _run: numpy.ndarray of the vehicle state at each timestep
          shape: (sim_duration=100, 12) 
    """

    sim_dt = 0.001      # in s
    sim_duration = 100  # in ms, since dt=0.001s
    
    # vehicle initial state
    state = numpy.array([x, vx, y, vy, z, vz, r, vr, p, vp, yaw, vyaw]).astype(numpy.double)

    # controls
    #    T     = couple,         in N.m
    #    delta = steering angle, in rad
    controls = numpy.array([
        T,  # front left wheel
        T,  # front right wheel
        T if T < 0 else 0.,  # rear left wheel. can only brake and not accelerate
        T if T < 0 else 0.,  # rear right wheel. can only brake and not accelerate
        delta
    ])
    mu = numpy.array([1., 1., 1., 1.])
    
    # Run sim
    # Note: sim columns: dt x vx y vy z vz r vr p vp yaw vyaw om_fl om_fr om_rl om_rr ax ay sr1 sr2 sr3 sr4 sa1 sa2 sa3 sa4
    _run = vehiclemodel.run(sim_duration, sim_dt, state, controls, mu)        

    # Drop the first column (time)
    _run = _run[:, 1:]
    
    # Shape: (sim_duration, 12).
    #        Rows format: x vx y vy z vz roll droll pitch dpitch yaw dyaw
    _run = _run[:, 0:12]

    return _run


# Drive the vehicle at 10m/s for 100ms:
demo_state_trajectory = get_simulation(0., 10., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., .0)
assert demo_state_trajectory.shape == (100, 12)
