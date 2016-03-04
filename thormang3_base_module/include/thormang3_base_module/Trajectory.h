/*
 * trajectory.h
 *
 *  Created on: Jul 13, 2015
 *      Author: sch
 */

#ifndef BASE_MODULE_TRAJECTORY_H_
#define BASE_MODULE_TRAJECTORY_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <Eigen/Dense>

namespace ROBOTIS_BASE
{

// minimum jerk trajectory

Eigen::MatrixXd minimum_jerk_tra( double pos_start , double vel_start , double accel_start,
                                  double pos_end ,   double vel_end ,   double accel_end,
                                  double smp_time ,  double mov_time );

Eigen::MatrixXd minimum_jerk_tra_vian_qdqddq( int n,
                                              double x0 , double v0 , double a0 ,
                                              Eigen::MatrixXd x,  Eigen::MatrixXd dx, Eigen::MatrixXd ddx,
                                              double xf, double vf, double af,
                                              double smp, Eigen::MatrixXd t, double tf );

}

#endif /* TRAJECTORY_H_ */
