import time
import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from matplotlib.animation import FuncAnimation

import numpy as np
import pure_pursuit


def vehicle_dynamics(state, control, wheelbase, dt):
    def dx_dt(t, state, control):
        x, y, theta, v = state
        udv, delta = control

        dtheta = v * np.tan(delta) / wheelbase
        dv = udv  # acceleration input

        dx = v * np.cos(theta)
        dy = v * np.sin(theta)

        return [dx, dy, dtheta, dv]

    sol = integrate.solve_ivp(dx_dt, [0, dt], state, args=(control,), method='RK45')
    next_state = sol.y[:, -1]
    next_state[2] = (next_state[2] + np.pi) % (2 * np.pi) - np.pi
    return next_state

if __name__ == "__main__":
    num_points = 20
    radius = 10
    center = (0, 0)
    x = np.linspace(0, 2*np.pi * 10, num_points)
    y = np.cos(x/5) - x/10
    psi = -np.sin(x/5)/5 - 1/10

    waypoints = np.column_stack((x, y, psi))
    initial_state = [1, 1, 0, 0]  # x, y, theta, v
    dt = 0.1
    max_time = 30.0

    params = pure_pursuit.PurePursuitParams(
        lookahead_distance=3.0,
        wheelbase=0.17,
        desired_vel=2.0
    )
    state_history = np.array(initial_state)
    target_history = np.array(initial_state[:3])

    current_state = initial_state

    time = 0.0
    while time < max_time:
        control_input, lookahead_point, _ = pure_pursuit.pure_pursuit(waypoints, current_state, params)
        next_state = vehicle_dynamics(current_state, control_input, params.wheelbase, dt)
        state_history = np.vstack((state_history, next_state))
        target_history = np.vstack((target_history, lookahead_point))
        current_state = next_state
        time += dt

    fig, ax = plt.subplots()
    plt.plot(waypoints[:, 0], waypoints[:, 1], color='red', linestyle='--', marker='s', label='Waypoints')
    lookahead_circle = patches.Circle((state_history[0, 0], state_history[0, 1]), params.lookahead_distance, color='green', fill=False, linestyle='--')
    ax.add_patch(lookahead_circle)
    veh_path, = ax.plot(state_history[:, 0], state_history[:, 1], color='blue', label='Vehicle Path')
    target_pt, = ax.plot([], [], 'go')

    def update(t):
        lookahead_circle.center = (state_history[t, 0], state_history[t, 1])

        veh_path.set_data(state_history[:t,0], state_history[:t,1])
        target_pt.set_data([target_history[t,0]], [target_history[t,1]])
        return veh_path, target_pt, lookahead_circle
    ani = FuncAnimation(fig, update, frames=state_history.shape[0], interval=50, blit=True)

    plt.title("Pure Pursuit Path Tracking")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid()
    ani.save('animation.gif', writer='pillow', fps=10)
    # plt.show()