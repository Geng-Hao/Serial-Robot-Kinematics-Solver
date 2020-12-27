#ifndef IK_Solver_H
#define IK_Solver_H

#include"Vec.h"
#include"Matrix.h"
#include<iostream>
#include<cmath>

#define joint_number 6

constexpr auto PI = 3.14159265358979323846;


class IK_Solver{

 private:
 
 /*DH Parameter*/
 
 double alpha[joint_number];
 double a[joint_number];
 double d[joint_number];
 double theta[joint_number];
 double l[joint_number];
 double u[joint_number];
 
 
 
 /* private function */
 Matrix getT(int);
 Vec xaxis(int);
 Vec yaxis(int);
 Vec zaxis(int);
 Vec joint_position(int);
 void CCD(const Vec&, const Vec&, const Vec&, const Vec&, double*, double*);
 
 
 public: 
 
 IK_Solver();
 IK_Solver(double*,double*,double*,double*,double*,double*);

 Vec end_effector_position(double*); // forward kinematics position
 Vec end_effector_orientationX(double*); // forward kinematics orientation x 
 Vec end_effector_orientationY(double*); // forward kinematics orientation y 
 Vec end_effector_orientationZ(double*); // forward kinematics orientation z 
 
 void Inv_Kine(const Vec&, double, double, double, double, double, double); //inverse kinematics
 
 double* getTheta(); // obtain the current robot pose
 
 void print(); // print the DH parameters of robot manipulator
};

#endif
