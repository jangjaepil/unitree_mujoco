#pragma once
// pinocchio
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass-derivatives.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
//qpOASES
//#include "qpOASES.hpp"

//EIGEN
#include <Eigen/Geometry>
#include <unordered_set>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Cholesky>

//std
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
