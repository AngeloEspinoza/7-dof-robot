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
// #include <webots/GPS.hpp>

/* C++ libraries */
#include <iomanip>

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
  // GPS *gps = robot->getGPS("gps");

  /* Enabling position sensors */
  ps_link_1->enable(TIME_STEP);
  ps_link_2->enable(TIME_STEP);
  ps_link_3->enable(TIME_STEP);
  ps_link_4->enable(TIME_STEP);


  /* Enabling GPS */
  // gps->enable(TIME_STEP);



  /* Homogenous transformation matrices */
  float **A01, **A12, **A23, **A34; // Matrices
  float **A02, **A03, **A04; // Resultant matrices

  while (robot->step(TIME_STEP) != -1) 
  {
    int key = keyboard.getKey();

    /* Rotations around z Rot_z(theta_i) */
    float theta_1 = ps_link_1->getValue() + degrees_to_radians(90);
    float theta_2 = ps_link_2->getValue();
    float theta_3 = ps_link_3->getValue();
    float theta_4 = ps_link_4->getValue();

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

    // std::cout << "GPS: " << gps->getValues() << std::endl;

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


    std::cout << key << std::endl;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}


float degrees_to_radians(float degrees)
{
  return (degrees / 360) * 2*PI;
}

float radians_to_degrees(float radians)
{
  radians = (radians / PI) * 180;
  return radians;
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
