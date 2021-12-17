#include "rtt_searcher.h"

using namespace std;
using namespace Eigen;

RttSearcher::RttSearcher(Eigen::Vector3d map_lower, Eigen::Vector3d map_upper)
{
  space_ = std::make_shared<ob::RealVectorStateSpace>(3);
  map_lower_ = map_lower;
  map_upper_ = map_upper;
  // Set the bounds of space to be in [0,1].
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, map_lower_(0) - 10);
  bounds.setLow(1, map_lower_(1) - 10);
  bounds.setLow(2, map_lower_(2) - 1);

  bounds.setHigh(0, map_upper_(0) + 10);
  bounds.setHigh(1, map_upper_(1) + 10);
  bounds.setHigh(2, map_upper_(2) + 1);
  space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  si_ = std::make_shared<ob::SpaceInformation>(space_);

  checker_ = std::make_shared<ValidityChecker>(si_, this);
  si_->setStateValidityChecker(checker_);
  si_->setup();
}

RttSearcher::~RttSearcher()
{
}

void RttSearcher::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void RttSearcher::setObs(const double coord_x, const double coord_y,
                         const double coord_z)
{
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
      idx_y == GLY_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
  else
  {
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
  }
}

inline bool RttSearcher::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3i RttSearcher::coord2gridIndex(const Vector3d &pt)
{
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

ob::OptimizationObjectivePtr RttSearcher::getPathLengthObjective(const ob::SpaceInformationPtr &si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  // auto obj = std::shared_ptr<ob::PathLengthOptimizationObjective>(si);
  // obj->setCostThreshold(ob::Cost(1.51)); // 设置阈值
  return obj;
}

std::vector<Eigen::Vector3d> RttSearcher::RttPathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt)
{
  ob::ScopedState<> start(space_);
  start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_pt.x();
  start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_pt.y();
  start->as<ob::RealVectorStateSpace::StateType>()->values[2] = start_pt.z();

  ob::ScopedState<> goal(space_);
  goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = target_pt.x();
  goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = target_pt.y();
  goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = target_pt.z();

  ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si_);
  pdef->setStartAndGoalStates(start, goal);
  pdef->setOptimizationObjective(getPathLengthObjective(si_));

  ob::PlannerPtr optimizingPlanner = std::make_shared<og::RRTstar>(si_);
  optimizingPlanner->setProblemDefinition(pdef);
  optimizingPlanner->setup();

  ob::PlannerStatus solved = optimizingPlanner->solve(0.1);

  std::vector<Eigen::Vector3d> path_points;

  if (solved)
  {
    og::PathGeometric *path = pdef->getSolutionPath()->as<og::PathGeometric>();

    for (size_t path_idx = 0; path_idx < path->getStateCount(); path_idx++)
    {
      const ob::RealVectorStateSpace::StateType *state = path->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>();
      path_points.emplace_back(state->values[0], state->values[1], state->values[2]);
    }
  }
  return path_points;
}

vector<Vector3d> RttSearcher::pathSimplify(const vector<Vector3d> &path, double path_resolution)
{
  vector<Vector3d> subPath;
  /**
   *
   * STEP 2.1:  implement the RDP algorithm
   *
   * **/
  double d_max = 0.0;
  int index = 0;

  int n = path.size();
  // base case
  if (n <= 2)
    return path;

  // find the farthest point
  Vector3d A = path[0];
  Vector3d B = path[n - 1];
  Vector3d AB = B - A;
  for (int i = 1; i < n - 1; ++i)
  {
    Vector3d P = path[i];
    Vector3d AP = P - A;
    Vector3d AB_cross_AP = AB.cross(AP);
    double d = AB_cross_AP.norm() / AB.norm();
    if (d > d_max)
    {
      d_max = d;
      index = i;
    }
  }

  // Divide & conquer
  if (d_max > path_resolution)
  {
    vector<Vector3d> path1, path2;
    for (int i = 0; i <= index; ++i)
      path1.push_back(path[i]);
    for (int i = index; i < n; ++i)
      path2.push_back(path[i]);
    vector<Vector3d> subPath1 = pathSimplify(path1, path_resolution);
    vector<Vector3d> subPath2 = pathSimplify(path2, path_resolution);
    for (const auto &p : subPath1)
      subPath.push_back(p);
    subPath.pop_back();
    for (const auto &p : subPath2)
      subPath.push_back(p);
  }
  else
  {
    subPath.push_back(A);
    subPath.push_back(B);
  }
  return subPath;
}

int RttSearcher::safeCheck(MatrixXd polyCoeff, VectorXd time)
{
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe
  /**
   *
   * STEP 3.3:  finish the sareCheck()
   *
   * **/
  for (int i = 0; i < time.size(); i++)
  {
    for (double t = 0.0; t < time(i); t += 0.01)
    {
      Vector3d pt = getPosPoly(polyCoeff, i, t);
      Vector3i idx = coord2gridIndex(pt);
      if (!isFree(idx(0), idx(1), idx(2)))
        return i;
    }
  }
  return unsafe_segment;
}

Vector3d RttSearcher::getPosPoly(MatrixXd polyCoeff, int k, double t)
{
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++)
  {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}