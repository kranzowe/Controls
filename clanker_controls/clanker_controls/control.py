import numpy as np

def get_pwm_vel_from_vel(ol_model, vel):
    if vel >= ol_model.velocity.ol_velocities[-1]:
        return ol_model.velocity.pwms[-1]
    elif vel <= ol_model.velocity.ol_velocities[0]:
        return ol_model.velocity.pwms[0]
    else:
        h_vel_idx = np.where(ol_model.velocity.ol_velocities >= vel)[0][0]
        l_vel_idx = h_vel_idx - 1
        h_vel = ol_model.velocity.ol_velocities[h_vel_idx]
        l_vel = ol_model.velocity.ol_velocities[l_vel_idx]
        m = (ol_model.velocity.pwms[h_vel_idx] - ol_model.velocity.pwms[l_vel_idx]) / (h_vel - l_vel)
        b = ol_model.velocity.pwms[l_vel_idx] - m * l_vel
        return m * vel + b

def get_pwm_steer_from_turn_radius(ol_model, turn_radius):
    if np.abs(turn_radius) >= ol_model.steering.ol_radius[ol_model.steering.zero_idx]:
        return ol_model.steering.pwms[ol_model.steering.zero_idx]
    h_rad_idx, l_rad_idx, h_radius, l_radius, m, b = None, None, None, None, None, None
    if turn_radius > 0:
        if turn_radius <= ol_model.steering.ol_radius[-1]:
            return ol_model.steering.pwms[-1]
        l_rad_idx = int(np.where(ol_model.steering.ol_radius >= turn_radius)[-1][0])
        h_rad_idx = l_rad_idx + 1
        h_radius = ol_model.steering.ol_radius[h_rad_idx]
    elif turn_radius < 0:
        if turn_radius >= ol_model.steering.ol_radius[0]:
            return ol_model.steering.pwms[0]
        h_rad_idx = int(np.where(ol_model.steering.ol_radius <= turn_radius)[0][0])
        l_rad_idx = h_rad_idx + 1
        h_radius = ol_model.steering.ol_radius[h_rad_idx]
        if h_rad_idx == ol_model.steering.zero_idx:
            h_radius = -h_radius
    l_radius = ol_model.steering.ol_radius[l_rad_idx]
    m = np.abs((ol_model.steering.pwms[h_rad_idx] - ol_model.steering.pwms[l_rad_idx]) / (h_radius - l_radius))
    b = ol_model.steering.pwms[l_radius]
    return b + m * (turn_radius - l_radius)