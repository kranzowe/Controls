#pragma once

#include <math.h>
#include <list>
#include <stdlib.h>
#include <Eigen/Dense>


typedef Eigen::Matrix<float, 3, 1> V3f;
typedef Eigen::Matrix<float, 4, 1> V4f;
typedef Eigen::Matrix<float, 10, 1> V10f;

typedef Eigen::Matrix<float, 4, 3> V4f3;
typedef Eigen::Matrix<float, 3, 4> V3f4;
typedef Eigen::Matrix<float, 2, 4> V2f4;

#define GRAVITY -9.8115 //  the gravitational constant

class DoubleTrackCar{

    //interest amazon article
    //https://builder.aws.com/content/2wmc5dBRDe3jBPmu7WSJsZuaxa5/aws-deepracer-car-modifications

    //taking dynamics from here: assuming a flat track... 
    //file:///home/hollis/Downloads/Advanced_Estimation_Techniques_for_Vehicle_System_.pdf

    //part of the theory here is to handle all the calculations upfront to speed up the execution
    public:
        DoubleTrackCar();


    private:

        void load_parameters(); // load in the parameters and perform initial calculations

        V10f solve_eom(float drive_motor_torque, float steering_motor_velocity);
        void update_adjusted_com();
        void distribute_tire_loadidng(float roll_angle_acceleration);

        //state of the car [X, dX, Y, DY, yaw, dyaw, roll, droll, steering, dsteering]
        V10f state;

        V3f adjusted_com; // com of the car adjusted for the roll dynamics

        V4f tire_loading; // the loading of each car tire
        V3f4 max_tire_forces; // the maximum frictional vector (and centering moment) that is provided by each tire


    //Mass parameters -------------------------------------

        float mass;
        float sprung_mass; // mass of the car that is subject to normal roll motion (not accounting for lifting a wheel)
        float Izz; // yaw axis inertia
        float Ixx; // roll axis inertia

        //no steering mass - think the best way is set the steering velocity with the HAND OF GOD

    // Stiffness & Damping Parameters ---------------------------------

        float roll_damping; // the damping coefficent for the roll axis
        float roll_stiffness; // the stiffness of the roll axis

        float coefficent_drag; // the drag coefficent (in the automotive sense, x axis)
        float front_area; // the frontal area of the car

    // Kinematic Parameters ------------------------------

        float wheelbase; // the wheel base of the wheels of the car, measured from axel to axel, steering at nominal
        float trackwidth; // the car's track with, measured from center to center of rear wheeels
        float height_roll_center; // the heigh off the ground to the "axis" of pivot of the rolling mass
        float height_roll_mass; // the height of the center of the rolling mass from the center center

        V3f com_position; // measured from the center of the wheelbase and trackwidth, z is the height off the ground, at nominal

        // these are derrived parameters from the com position, wheel base and trackwidth
        float wheels_br;
        float wheels_bl;
        float wheels_lf;
        float wheels_lr;

    // Environmental parameters -----------------------------

        float air_density; // density of the surrounding air

    // Tire Objects -----------------------------------------

        std::list<PacejkaTire> tires;
        V4f3 tire_distribution_matrix;
        float required_pitch_torque;

};

class PacejkaTire{

    //model used
    //http://www-cdr.stanford.edu/dynamic/bywire/tires.pdf

    //this may be helpful in guessimating parameters...
    //https://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/

    //helpful diagram
    //http://www-cdr.stanford.edu/dynamic/bywire/tires.pdf

    public:

        PacejkaTire();

        V3f get_tire_wrench(float tire_vx, float tire_vy, float loading); // albeit a 2d wrench

    private:

        void load_parameters();

    //compound parameters ------------------------------------------------------------

        //generally
        //  C - shape factor
        //  D - peak factor
        //  B - stiffness factor
        //  E - curvature factor

        //lateral
        float c_lat;
        float d_lat;
        float b_lat;
        float e_lat;

        //longitudal
        float c_lon;
        float d_lon;
        float b_lon;
        float e_lon;

        //aligning moment
        float c_m;
        float d_m;
        float b_m;
        float e_m;
};