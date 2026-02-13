#include "dynamics.h"

void DoubleTrackCar::load_parameters(){

    //calculate derived wheel position values
    wheels_br = trackwidth / 2 - com_position(1);
    wheels_bl = trackwidth / 2 + com_position(1);
    wheels_lf = wheelbase / 2 - com_position(0);
    wheels_lr = wheelbase / 2 + com_position(0);

    //calculate the tire distribution matrix via psuedoinverse
    V3f4 tire_loading_governing_equations;
    tire_loading_governing_equations << 
        1, 1, 1, 1, // tires should be loaded equal to the downward force
        1, -1, 1, -1, // balance of roll torque
        -1, -1, 1, 1; // balance of pithc torque
    tire_distribution_matrix = (tire_loading_governing_equations.transpose() * tire_loading_governing_equations).inverse() * tire_loading_governing_equations.transpose();
    // should be used as
    //    tdm * [Fz, Troll, Tpitch]^T = [Fl, Fr, Rl, Rr]^T
    // note the loads here are the force and torques that should be supplied by the tires as a group

}

void DoubleTrackCar::update_adjusted_com(){
    //account for the motion of the sprung mass

    adjusted_com(0) = 0;
    adjusted_com(1) = com_position(1) - sin(state(6)) * height_roll_mass * (sprung_mass / mass);
    adjusted_com(2) = com_position(2) - (1 - cos(state(6))) * height_roll_mass * (sprung_mass / mass);
}

void DoubleTrackCar::distribute_tire_loadidng(float roll_angle_acceleration){
    //determine the z axis force being exerted on each tire
    V3f total_loading;

    //calculate the z force required for the change in sprung mass
    total_loading(0) = -mass * GRAVITY + height_roll_mass * (-roll_angle_acceleration * sin(state(6)) - state(7) * state(7) * cos(state(6))) * sprung_mass;
    
    //calculate the roll torque that must be exerted by the tires
    // from my understanding of dynamics (which I'm not entirely sure on this point) but only the torque exerted by the "damper" or "spring" is seen by the tires 
    total_loading(1) = mass * adjusted_com(1) - roll_damping * state(7) - roll_stiffness * state(6);

    //calculate the pitch torque that must be exerted by the tires - this is static because pitch roll is being ignored
    total_loading(2) = -mass * com_position(0);

    //get the load on each tire
    tire_loading = tire_distribution_matrix * total_loading;
}


V10f DoubleTrackCar::solve_eom(float drive_motor_torque, float steering_motor_velocity){
    //state of the car [X, dX, Y, DY, yaw, dyaw, roll, droll, steering, dsteering]
    V10f d_state;

    //update the com 
    update_adjusted_com();
    
    //roll dynamics - it may be advantageous to calculate this based on the cg of the sprung mass... (if car has large static lean)
    d_state(6) = state(7);
    d_state(7) = (
        sprung_mass * height_roll_mass * (state(3) + state(1) * state(5)) +  // torque due to the acceleration of the sprung mass due to lateral motion
        sprung_mass * height_roll_mass * GRAVITY * (state(6)) - // due to the offset of the sprung mass caused by roll itself
        roll_damping * state(7) - // roll damping
        roll_stiffness * state(6) //spring action of the roll
    );

    //calculate the steering 

    //calculate the load on each tire
    distribute_tire_loadidng(d_state(7));

    //calculate the velocities at each tire
    V2f4 tire_velocities;

    //front left
    tire_velocities(0, 0) = state(1) - state(5) * wheels_bl * cos(state(8)) + sin(state(8)) * state(3) - state(5) * wheels_lf;
    tire_velocities(0, 1) = -state(1) - state(5) * wheels_bl * sin(state(8)) + cos(state(8)) * state(3) - state(5) * wheels_lf;

    //front right
    tire_velocities(1, 0) = state(1) - state(5) * wheels_bl * cos(state(8)) + sin(state(8)) * state(3) - state(5) * wheels_lf;
    tire_velocities(1, 1) = -state(1) - state(5) * wheels_bl * sin(state(8)) + cos(state(8)) * state(3) - state(5) * wheels_lf;
    
    //back left
    tire_velocities(2, 0) = state(1) - state(5) * wheels_bl;
    tire_velocities(2, 1) = state(3) + state(5) * wheels_lr;

    //brack right
    tire_velocities(3, 0) = state(1) + state(5) * wheels_br;
    tire_velocities(3, 1) = state(3) + state(5) * wheels_lr;

    //get the forces each tire exerts
    int counter = 0;
    for(auto it = tires.begin(); it != tires.end(); ++it){
        max_tire_forces.col(counter) = it->get_tire_wrench(tire_velocities(counter, 0), tire_velocities(counter, 1),tire_loading(counter));
        counter++;
    }

    //
}


V3f PacejkaTire::get_tire_wrench(float tire_vx, float tire_vy, float loading){
    
    // calculate the slip angle
    float slip = atan2(tire_vx, tire_vy);

    //calculate the percent slip
    float percent_slip = (tire_vx - tire_vy) / tire_vx;

    V3f tire_wrench; // x y yaw

    //calculate the lateral tire force
    tire_wrench(1) = d_lat * sin(c_lat * atan(b_lat * slip - e_lat * (b_lat * slip - atan(b_lat * slip))));

    //calculate the longitudal tire force
    tire_wrench(0) = d_lon * sin(c_lon * atan(b_lon * percent_slip - e_lon * (b_lon * percent_slip - atan(b_lon * percent_slip))));

    //calculate the aligning tire force
    //may want to force this to zero... may not be a large factor for the rc car
    tire_wrench(2) = d_m * sin(c_m * atan(b_m * percent_slip - e_m* (b_m * percent_slip - atan(b_m * percent_slip))));

    //factor in the loading
    tire_wrench = tire_wrench * loading;

    return tire_wrench;
}
