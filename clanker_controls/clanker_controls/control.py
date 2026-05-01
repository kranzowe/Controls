import numpy as np
from ament_index_python import get_package_share_directory
import yaml
import os
from types import SimpleNamespace

OL_MODEL_SUBPATH = "resource/ol_data.yaml"

def load_ol_model(logger=None):

        #get the robo rover share directory
        robo_share_dir = get_package_share_directory("robo_rover")
        ol_path = os.path.join(robo_share_dir, OL_MODEL_SUBPATH)

        with open(ol_path, "r") as file:
            try:
                model_raw = yaml.safe_load(file)

            except:
                if logger is not None:
                    self.get_logger().error("Failed to open ol model!")
                return None
            
        ol_model = SimpleNamespace()
        ol_model.velocity = SimpleNamespace()
        ol_model.steering = SimpleNamespace()
        ol_model.time_constant = model_raw["time_constant"]
        ol_model.velocity.ol_velocities = np.array(model_raw["velocity"]["steady_state_velocity"])
        ol_model.velocity.pwms = np.array(model_raw["velocity"]["pwm_values"])
        ol_model.steering.ol_radius = np.array(model_raw["angular"]["turn_radius_values"])
        ol_model.steering.pwms = np.array(model_raw["angular"]["pwm_values"])
        ol_model.steering.zero_idx = np.argmax(np.abs(ol_model.steering.ol_radius))
        return ol_model

def get_pwm_vel_from_vel(ol_model, vel):
    if vel <= ol_model.velocity.ol_velocities[0]:
        return ol_model.velocity.pwms[0]
    elif vel >= ol_model.velocity.ol_velocities[-1]:
        return ol_model.velocity.pwms[-1]
    else:
        h_vel_idx = np.where(ol_model.velocity.ol_velocities >= vel)[0][0]
        l_vel_idx = h_vel_idx + 1
        h_vel = ol_model.velocity.ol_velocities[h_vel_idx]
        l_vel = ol_model.velocity.ol_velocities[l_vel_idx]
        m = (ol_model.velocity.pwms[h_vel_idx] - ol_model.velocity.pwms[l_vel_idx]) / (h_vel - l_vel)
        b = ol_model.velocity.pwms[l_vel_idx] - m * l_vel
        return m * vel + b

def get_pwm_steer_from_turn_radius(ol_model, turn_radius):
    center_idx = ol_model.steering.zero_idx
    center_radius = np.abs(ol_model.steering.ol_radius[center_idx])
    center_pwm = ol_model.steering.pwms[center_idx]

    if not np.isfinite(turn_radius) or turn_radius == 0.0:
        return center_pwm

    abs_radius = np.abs(turn_radius)
    if abs_radius >= center_radius:
        return center_pwm

    if turn_radius > 0:
        # Right turns: sort by |radius| so interpolation x-axis is monotonic.
        radii = np.abs(ol_model.steering.ol_radius[center_idx + 1:])
        pwms = ol_model.steering.pwms[center_idx + 1:]
        sort_idx = np.argsort(radii)
        radii = radii[sort_idx]
        pwms = pwms[sort_idx]
    else:
        # Left turns are already ordered by increasing |radius| in the model.
        radii = np.abs(ol_model.steering.ol_radius[:center_idx])
        pwms = ol_model.steering.pwms[:center_idx]

    # Add center point so large radii naturally interpolate to straight steering.
    radii = np.append(radii, center_radius)
    pwms = np.append(pwms, center_pwm)
    return float(np.interp(abs_radius, radii, pwms))
