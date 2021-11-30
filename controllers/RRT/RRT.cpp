// File:          RRT.cpp
// Date:          October 17th, 2021
// Description:
// Author:        Angelo Espinoza
// Modifications:

/* Webots libraries */
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>

/* C++ libraries */
#include <iomanip>
#include <cmath>

/* Own libraries */
#include <DH.hpp>

/* Macros */
#define PI 3.141592

using namespace webots;

/* Protofunctions */
float degrees_to_radians(float degrees);
float radians_to_degrees(float radians);
void print_matrix(float **Tij);

void move_link_1_left(Motor *link_1);
void move_link_1_right(Motor *link_1);
void move_link_2_up(Motor *link_2);
void move_link_2_down(Motor *link_2);
void move_link_3_up(Motor *link_3);
void move_link_3_down(Motor *link_3);
void move_link_4_up(Motor *link_4);
void move_link_4_down(Motor *link_4);
void stop_links(Motor *link_1, Motor *link_2, Motor *link_3, Motor *link_4);

void move_forward_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3);
void move_backward_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3);
void move_left_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3);
void move_right_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3);
void turn_around_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3);
void stop_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3);

void turn_left_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3);
void turn_right_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3);
double get_bearing_in_degrees(Compass *compass);

float calculate_angle_in_degrees(float position_x, float position_z, float destination_x, float destination_z);
float bearing_to_heading(double heading);
double cartesian_calculation(double heading, double destination);

int main(int argc, char **argv) 
{
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int TIME_STEP = (int)robot->getBasicTimeStep();

  /* Link motors */
  Motor *motor_link_1 = robot->getMotor("rotational motor link 1");
  Motor *motor_link_2 = robot->getMotor("rotational motor link 2");
  Motor *motor_link_3 = robot->getMotor("rotational motor link 3");
  Motor *motor_link_4 = robot->getMotor("rotational motor link 4");

  /* Wheel motors */
  Motor *motor_wheel_1 = robot->getMotor("wheel motor 1");
  Motor *motor_wheel_2 = robot->getMotor("wheel motor 2");
  Motor *motor_wheel_3 = robot->getMotor("wheel motor 3");

  /* Link position sensors */
  PositionSensor *ps_link_1 = robot->getPositionSensor("position sensor link 1");
  PositionSensor *ps_link_2 = robot->getPositionSensor("position sensor link 2");
  PositionSensor *ps_link_3 = robot->getPositionSensor("position sensor link 3");
  PositionSensor *ps_link_4 = robot->getPositionSensor("position sensor link 4");

  /* Link positions sensors */
  motor_link_1->setPosition(INFINITY);
  motor_link_2->setPosition(INFINITY);
  motor_link_3->setPosition(INFINITY);
  motor_link_4->setPosition(INFINITY);

  /* Initialize links all in 0 to avoid wheel ovedump */
  motor_link_1->setVelocity(0);
  motor_link_2->setVelocity(0);
  motor_link_3->setVelocity(0);
  motor_link_4->setVelocity(0);

  /* Wheel position sensors */
  motor_wheel_1->setPosition(INFINITY);
  motor_wheel_2->setPosition(INFINITY);
  motor_wheel_3->setPosition(INFINITY);

  /* Initialize wheels all in 0 to avoid wheel overdump*/
  motor_wheel_1->setVelocity(0);
  motor_wheel_2->setVelocity(0);
  motor_wheel_3->setVelocity(0);

  /* Keyboard */
  Keyboard keyboard;

  /* GPS */
  GPS *gps = robot->getGPS("gps");

  /* Compass */
  Compass *compass = robot->getCompass("compass");

  /* Enabling position sensors */
  ps_link_1->enable(TIME_STEP);
  ps_link_2->enable(TIME_STEP);
  ps_link_3->enable(TIME_STEP);
  ps_link_4->enable(TIME_STEP);

  /* Enabling keyboard  */
  keyboard.enable(TIME_STEP);

  /* Enabling GPS */
  gps->enable(TIME_STEP);

  /* Enabling Compass */
  compass->enable(TIME_STEP);  

  /* Homogenous transformation matrices */
  float **A01, **A12, **A23, **A34; // Matrices
  float **A02, **A03, **A04; // Resultant matrices

  int desired_x = -1;
  int desired_z = -2;

  while (robot->step(TIME_STEP) != -1) 
  {
    int key = keyboard.getKey();

    /* Rotations around z Rot_z(theta_i) */
    float theta_1 = ps_link_1->getValue() + degrees_to_radians(90);
    float theta_2 = ps_link_2->getValue();
    float theta_3 = ps_link_3->getValue();
    float theta_4 = ps_link_4->getValue();

    /* Robot position coordinates */
    const double position_x = gps->getValues()[0];
    const double position_y = gps->getValues()[1];
    const double position_z = gps->getValues()[2];

    /* Robot compass angles */
    const double compass_x = compass->getValues()[0];
    const double compass_y = compass->getValues()[1];
    const double compass_z = compass->getValues()[2];

    /* Calculates the needed angle beta that the robot needs to rotate 
       to align it with the desired position */
    float beta = calculate_angle_in_degrees(position_x, position_z, desired_x, desired_z);

    /* Current heading of the robot */
    double current_compass = get_bearing_in_degrees(compass); // Recommended by Webots

    /* Translations in z Trans_z(d_i) */
    float d1 = 0.715;  // [m]
    float d2 = 0.150;  // [m]
    float d3 = -0.050; // [m]
    float d4 = -0.050; // [m]

    /* Translations in x Trans_x(a_i) */
    float a1 = 0.000;  // [m]
    float a2 = 0.300;  // [m]
    float a3 = 0.250;  // [m]
    float a4 = 0.100;  // [m]

    /* Rotations arround x Rot_x(alpha_i) */
    float alpha_1 = degrees_to_radians(90);
    float alpha_2 = 0;
    float alpha_3 = 0;
    float alpha_4 = 0;

    A01 = dh(theta_1, d1, a1, alpha_1);
    A12 = dh(theta_2, d2, a2, alpha_2);
    A23 = dh(theta_3, d3, a3, alpha_3);
    A34 = dh(theta_4, d4, a4, alpha_4);

    A02 = multiply_matrices(A01, A12);
    A03 = multiply_matrices(A02, A23);
    A04 = multiply_matrices(A03, A34);

    std::cout << "\t---MATRIX A01---" << std::endl;
    print_matrix(A01);
    std::cout << "\t---MATRIX A12---" << std::endl;
    print_matrix(A12);
    std::cout << "\t---MATRIX A23---" << std::endl;
    print_matrix(A23);
    std::cout << "\t---MATRIX A34---" << std::endl;
    print_matrix(A34);


    std::cout << "\t---RESULT (A02)---" << std::endl;
    print_matrix(A02);

    std::cout << "\t---RESULT (A03)---" << std::endl;
    print_matrix(A03);

    std::cout << "\t---RESULT (A04)---" << std::endl;
    print_matrix(A04);

    std::cout << "Position sensor link 1: " << radians_to_degrees(ps_link_1->getValue()) << std::endl;
    std::cout << "Position sensor link 2: " << radians_to_degrees(ps_link_2->getValue()) << std::endl;
    std::cout << "Position sensor link 3: " << radians_to_degrees(ps_link_3->getValue()) << std::endl;
    std::cout << "Position sensor link 4: " << radians_to_degrees(ps_link_4->getValue()) << std::endl;

    std::cout << "Robot position in x: " << position_x << std::endl;
    std::cout << "Robot position in y: " << position_y << std::endl;
    std::cout << "Robot position in z: " << position_z << std::endl;

    std::cout << "Robot heading in x: " << compass_x << std::endl;
    std::cout << "Robot heading in y: " << compass_y << std::endl;
    std::cout << "Robot heading in z: " << compass_z << std::endl;

    switch (key)
    {
      case keyboard.UP: move_forward_robot(motor_wheel_1, motor_wheel_2, motor_wheel_3); break;
      case keyboard.DOWN: move_backward_robot(motor_wheel_1, motor_wheel_2, motor_wheel_3); break;
      case keyboard.LEFT: move_left_robot(motor_wheel_1, motor_wheel_2, motor_wheel_3); break;
      case keyboard.RIGHT: move_right_robot(motor_wheel_1, motor_wheel_2, motor_wheel_3); break;
      case 80: turn_around_robot(motor_wheel_1, motor_wheel_2, motor_wheel_3); break;
      case 89: move_link_1_left(motor_link_1); break;  // Y
      case 85: move_link_1_right(motor_link_1); break; // U
      case 73: move_link_2_up(motor_link_2); break;    // I
      case 79: move_link_2_down(motor_link_2); break;  // O
      case 74: move_link_3_up(motor_link_3); break;    // J
      case 75: move_link_3_down(motor_link_3); break;  // K
      case 76: move_link_4_up(motor_link_4); break;    // L
      case 59: move_link_4_down(motor_link_4); break;  // ;

      default:
        stop_robot(motor_wheel_1, motor_wheel_2, motor_wheel_3);
        stop_links(motor_link_1, motor_link_2, motor_link_3, motor_link_4); 
        break;
    }

    std::cout << "Needed angle to reach desired position: " << beta << "°" << std::endl;
    std::cout << "Current angle to reach desired position: " << current_compass << "°" << std::endl;

    if (current_compass >= 359) // Avoids initial angle 
    {
      current_compass = 0;
    }
    
    if (beta > current_compass)
    {
      turn_right_robot(motor_wheel_1, motor_wheel_2, motor_wheel_3);
    }
    else
    {
      stop_robot(motor_wheel_1, motor_wheel_2, motor_wheel_3);
    }
  };

  // Enter here exit cleanup code.
  delete robot;
  return 0;
}


float degrees_to_radians(float degrees)
{
  return (degrees / 180) * PI;
}

float radians_to_degrees(float radians)
{
  return (radians / PI) * 180;;
}

void move_link_1_left(Motor *link_1)
{
  link_1->setVelocity(1.5);
}

void move_link_1_right(Motor *link_1)
{
  link_1->setVelocity(-1.5);
}

void move_link_2_up(Motor *link_2)
{
  link_2->setVelocity(1.5);
}

void move_link_2_down(Motor *link_2)
{
  link_2->setVelocity(-1.5);
}

void move_link_3_up(Motor *link_3)
{
  link_3->setVelocity(1.5);
}

void move_link_3_down(Motor *link_3)
{
  link_3->setVelocity(-1.5);
}

void move_link_4_up(Motor *link_4)
{
  link_4->setVelocity(1.5);
}

void move_link_4_down(Motor *link_4)
{
  link_4->setVelocity(-1.5);
}

void stop_links(Motor *link_1, Motor *link_2, Motor *link_3, Motor *link_4)
{
  link_1->setVelocity(0);
  link_2->setVelocity(0);
  link_3->setVelocity(0);
  link_4->setVelocity(0);
}

void move_forward_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3)
{
  wheel_1->setVelocity(-3);
  wheel_2->setVelocity(-3);
  wheel_3->setVelocity(0);
}

void move_backward_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3)
{
  wheel_1->setVelocity(3);
  wheel_2->setVelocity(3);
  wheel_3->setVelocity(0);
}

void move_left_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3)
{
  wheel_1->setVelocity(-3);
  wheel_2->setVelocity(3);
  wheel_3->setVelocity(-6);
}

void move_right_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3)
{
  wheel_1->setVelocity(3);
  wheel_2->setVelocity(-3);
  wheel_3->setVelocity(6);
}

void turn_around_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3)
{
  wheel_1->setVelocity(3);
  wheel_2->setVelocity(-3);
  wheel_3->setVelocity(-3); 
}

void stop_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3)
{
  wheel_1->setVelocity(0);
  wheel_2->setVelocity(0);
  wheel_3->setVelocity(0);
}

void print_matrix(float **Tij)
{
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      std::cout << std::fixed << std::setprecision(3) << Tij[i][j] << "|\t";
    }
    std::cout << std::endl;
  }
}

float calculate_angle_in_degrees(float position_x, float position_z, float destination_x, float destination_z)
{
  float adjacent_cathetus = destination_x - position_x;
  float opposite_cathetus = destination_z - position_z;

  float angle = radians_to_degrees(atan2(opposite_cathetus, adjacent_cathetus));

  if (angle >= 360)
  {
    return angle = angle - 360;
  }
  else if (angle < 0)
  {
    return angle += 360;
  }
  else
  {
    return angle;
  }
}

void turn_left_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3)
{
  wheel_1->setVelocity(3);
  wheel_2->setVelocity(-3);
  wheel_3->setVelocity(-3);
}

void turn_right_robot(Motor *wheel_1, Motor *wheel_2, Motor *wheel_3)
{
  wheel_1->setVelocity(-3);
  wheel_2->setVelocity(3);
  wheel_3->setVelocity(3);
}

double get_bearing_in_degrees(Compass *compass)
{
  const double *north = compass->getValues();
  double rad = atan2(north[0], north[2]);
  double bearing = radians_to_degrees((rad - 1.5708));

  if (bearing < 0)
  {
    bearing = bearing + 360;
  }

  return bearing;
}

float bearing_to_heading(double heading)
{
  heading = 360 - heading;

  heading = heading + 90;

  if (heading > 360)
  {
    heading = heading - 360;
  }

  return heading;
}

double cartesian_calculation(double heading, double destination) {
    double theta_dot = destination - heading;

    if (theta_dot > 180)
        theta_dot = -(360-theta_dot);
    else if (theta_dot < -180)
        theta_dot = (360+theta_dot);

    return theta_dot;
}



// TODO: Autonomously rotate the robot until the desired angle is 0 (it is achieved)
// Create a function to do so.
// Create function that computes the euclidean distance.
// Make the robot moves to the desired position.

// TODO: Create obstacles and match them with the python script in size.
// Read an input file with coordinates separeted (try to put them in array preferably
// when reading them).
// e.g.
// 0 0 
// 2 3
// 4 1
// Write an ouput file in Python with info about the node coordinates, and obstacles
// coordinates.
// Rescale the node coordinates and obstacle coordinates since the Python script 
// is in dimensions of (600, 600) and the webots world is in dimensions of (6, 6)
// Therefore, try to divide the ouput file info by 100.