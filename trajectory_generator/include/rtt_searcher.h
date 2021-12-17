#ifndef _DCK_RRT_SEARCHER_H
#define _DCK_RRT_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#include "node.h"

#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ValidityChecker;

class RttSearcher {
private:
	uint8_t * data;
	Eigen::Vector3i goalIdx;
	int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
	int GLXYZ_SIZE, GLYZ_SIZE;

	double resolution, inv_resolution;
	double gl_xl, gl_yl, gl_zl;
	double gl_xu, gl_yu, gl_zu;

    Eigen::Vector3d map_lower_;
    Eigen::Vector3d map_upper_;
    ob::StateSpacePtr space_;
    ob::SpaceInformationPtr si_;
    std::shared_ptr<ValidityChecker> checker_;

public:
    RttSearcher(Eigen::Vector3d map_lower, Eigen::Vector3d map_upper);
    ~RttSearcher();
    std::vector<Eigen::Vector3d> RttPathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt);
    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);
    void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l,
                                  Eigen::Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id);
    void setObs(const double coord_x, const double coord_y, const double coord_z);
    bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);

    std::vector<Eigen::Vector3d> pathSimplify(const std::vector<Eigen::Vector3d> &path, double path_resolution);
    int safeCheck(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time);
    Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t);
};


// Our collision checker. For this demo, our robot's state space
class ValidityChecker : public ob::StateValidityChecker {
    private:
        RttSearcher *grid_checker_;
    public:
        ValidityChecker(const ob::SpaceInformationPtr& si, RttSearcher *checker_ptr) :
            ob::StateValidityChecker(si){
                grid_checker_ = checker_ptr;
        }
        bool isValid(const ob::State* state) const {
            const ob::RealVectorStateSpace::StateType* state3D =
                state->as<ob::RealVectorStateSpace::StateType>();
            Eigen::Vector3d pt;
            Eigen::Vector3i idx;

            pt(0) = state3D->values[0];
            pt(1) = state3D->values[1];
            pt(2) = state3D->values[2];
            idx = grid_checker_->coord2gridIndex(pt);

            return grid_checker_->isFree(idx(0), idx(1), idx(2));
        }
};

#endif