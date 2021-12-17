#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint()
{
}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for (int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

Eigen::VectorXd TrajectoryGeneratorWaypoint::GetCoeff(int n, int d, double t)
{ // 返回n阶多项式求k次导数后的系数 在时间t时的系数
    Eigen::VectorXd c(n + 1);
    c.setZero();
    for (int i = d; i <= n; ++i)
    {
        c(i) = Factorial(i) / Factorial(i - d) * pow(t, i - d);
    }
    return c;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order
    // derivatives, just assume them be 0;
    int p_order = 2 * d_order - 1; // the order of polynomial
    int p_num1d = p_order + 1;     // the number of variables in each segment

    int m = Time.size();
    Eigen::MatrixXd PolyCoeff(m, 3 * p_num1d);

    /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
    // minimum snap trajectory generation
    int number_of_variables = p_num1d * m;                                 // 要优化变量的个数 段数*每段参数的数量
    int number_of_constraints = 2 * d_order + (m - 1) + d_order * (m - 1); // 约束的个数  头尾约束+中间点约束+导数约束

    Eigen::VectorXd Px = Eigen::VectorXd::Zero(number_of_variables); // xyz参数的列向量
    Eigen::VectorXd Py = Eigen::VectorXd::Zero(number_of_variables);
    Eigen::VectorXd Pz = Eigen::VectorXd::Zero(number_of_variables);

    ROS_INFO("use QP");
    Eigen::SparseMatrix<double> Q;                                      // Q
    Eigen::VectorXd f = Eigen::VectorXd::Zero(number_of_variables);     // f
    Eigen::SparseMatrix<double> Aeq;                                    // A
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(number_of_constraints); // b

    Q.resize(number_of_variables, number_of_variables);
    for (int k = 0; k < m; ++k)
    {
        double T = Time(k);
        for (int i = d_order; i < p_num1d; ++i)
        {
            for (int j = d_order; j < p_num1d; ++j)
            {
                Q.insert(p_num1d * k + i, p_num1d * k + j) = Factorial(i) / Factorial(i - d_order) * Factorial(j) / Factorial(j - d_order) * pow(T, i + j - 2 * d_order + 1) / (i + j - 2.0 * d_order + 1.0);
            }
        }
    }

    // int dim = 0; // 当前操作第几个维度
    Aeq.resize(number_of_constraints, number_of_variables);

    int cnt = 0; // 当前的约束

    // constraint in start
    Eigen::Vector4d start_cond;
    start_cond(0) = Path(0, 0);
    start_cond(1) = Vel(0, 0);
    start_cond(2) = Acc(0, 0);
    start_cond(3) = 0;
    for (int i = 0; i < d_order; ++i)
    { // p,v,a,j 约束
        Eigen::VectorXd c = GetCoeff(p_order, i, 0);
        for (int k = 0; k < p_num1d; ++k)
        {
            if (0 != c(k))
                Aeq.insert(cnt, k) = c(k);
        }
        beq(cnt) = start_cond(i);
        ++cnt;
    }

    // constraint in end
    Eigen::Vector4d end_cond;
    end_cond(0) = Path(m, 0);
    end_cond(1) = Vel(1, 0);
    end_cond(2) = Acc(1, 0);
    end_cond(3) = 0;
    for (int i = 0; i < d_order; ++i)
    { // p,v,a,j 约束
        Eigen::VectorXd c = GetCoeff(p_order, i, Time(m - 1));
        for (int k = 0; k < p_num1d; ++k)
        {
            if (0 != c(k))
                Aeq.insert(cnt, p_num1d * (m - 1) + k) = c(k);
        }
        beq(cnt) = end_cond(i);
        ++cnt;
    }

    // position constrain in all middle waypoints
    for (int i = 0; i < m - 1; ++i)
    {
        Eigen::VectorXd c = GetCoeff(p_order, 0, 0);
        for (int k = 0; k < p_num1d; ++k)
        {
            if (0 != c(k))
                Aeq.insert(cnt, p_num1d * (i + 1) + k) = c(k);
        }
        beq(cnt) = Path(i + 1, 0);
        ++cnt;
    }

    // position velocity acceleration jerk continuity constrain between each 2 segments
    for (int d = 0; d < d_order; ++d)
    {
        for (int i = 0; i < m - 1; ++i)
        {
            Eigen::VectorXd c = GetCoeff(p_order, d, Time(i)); // 上一段结束
            for (int k = 0; k < p_num1d; ++k)
            {
                if (0 != c(k))
                    Aeq.insert(cnt, p_num1d * i + k) = c(k);
            }
            c = GetCoeff(p_order, d, 0); // 下一段开始
            for (int k = 0; k < p_num1d; ++k)
            {
                if (0 != c(k))
                    Aeq.insert(cnt, p_num1d * (i + 1) + k) = -c(k);
            }
            // beq(cnt) = 0;
            ++cnt;
        }
    }
    // ROS_INFO("cnt is:%d", cnt);
    // Aeq.makeCompressed(); // 压缩矩阵

    // std::cout << Q << '\n';

    // std::cout << f.transpose() << '\n';

    // std::cout << Aeq << '\n';

    // std::cout << beq.transpose() << '\n';

    // instantiate the solver
    OsqpEigen::Solver solver;
    // solver.settings()->setVerbosity(true);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(number_of_variables);
    solver.data()->setNumberOfConstraints(number_of_constraints);

    if (!solver.data()->setHessianMatrix(Q))
        ROS_ERROR("set Q error");
    if (!solver.data()->setGradient(f))
        ROS_ERROR("set f error");
    if (!solver.data()->setLinearConstraintsMatrix(Aeq))
        ROS_ERROR("set Aeq error");
    // if(!solver.data()->setBounds(beq, beq)) ROS_ERROR("set beq error");
    if (!solver.data()->setLowerBound(beq))
        ROS_ERROR("set LowerBound error");
    if (!solver.data()->setUpperBound(beq))
        ROS_ERROR("set UpperBound error");

    // instantiate the solver
    if (!solver.initSolver())
        ROS_ERROR("init solver error");

    // solve the QP problem
    if (!solver.solve())
        ROS_ERROR("solver qp error");
    // 获取参数
    Px = solver.getSolution();
    // solver.getSolution();
    // ROS_WARN("x axis is ok!!!");

    cnt = 0; // 参数更新只用更新beq
    // constraint in start
    start_cond(0) = Path(0, 1);
    start_cond(1) = Vel(0, 1);
    start_cond(2) = Acc(0, 1);
    start_cond(3) = 0;
    for (int i = 0; i < d_order; ++i)
    { // p,v,a,j 约束
        beq(cnt) = start_cond(i);
        ++cnt;
    }

    // constraint in end
    end_cond(0) = Path(m, 1);
    end_cond(1) = Vel(1, 1);
    end_cond(2) = Acc(1, 1);
    end_cond(3) = 0;
    for (int i = 0; i < d_order; ++i)
    { // p,v,a,j 约束
        beq(cnt) = end_cond(i);
        ++cnt;
    }

    // position constrain in all middle waypoints
    for (int i = 0; i < m - 1; ++i)
    {
        beq(cnt) = Path(i + 1, 1);
        ++cnt;
    }
    if (!solver.updateBounds(beq, beq))
        ROS_ERROR("set beq error");
    // solve the QP problem
    if (!solver.solve())
        ROS_ERROR("solver qp error");
    Py = solver.getSolution();
    // ROS_WARN("y axis is ok!!!");

    cnt = 0; // 参数更新只用更新beq
    // constraint in start
    start_cond(0) = Path(0, 2);
    start_cond(1) = Vel(0, 2);
    start_cond(2) = Acc(0, 2);
    start_cond(3) = 0;
    for (int i = 0; i < d_order; ++i)
    { // p,v,a,j 约束
        beq(cnt) = start_cond(i);
        ++cnt;
    }

    // constraint in end
    end_cond(0) = Path(m, 2);
    end_cond(1) = Vel(1, 2);
    end_cond(2) = Acc(1, 2);
    end_cond(3) = 0;
    for (int i = 0; i < d_order; ++i)
    { // p,v,a,j 约束
        beq(cnt) = end_cond(i);
        ++cnt;
    }

    // position constrain in all middle waypoints
    for (int i = 0; i < m - 1; ++i)
    {
        beq(cnt) = Path(i + 1, 2);
        ++cnt;
    }
    if (!solver.updateBounds(beq, beq))
        ROS_ERROR("set beq error");
    // solve the QP problem
    if (!solver.solve())
        ROS_ERROR("solver qp error");
    Pz = solver.getSolution();
    // ROS_WARN("z axis is ok!!!");

    for (int i = 0; i < m; ++i)
    {
        (PolyCoeff.row(i)).segment(0 * p_num1d, p_num1d) = Px.segment(p_num1d * i, p_num1d);
        (PolyCoeff.row(i)).segment(1 * p_num1d, p_num1d) = Py.segment(p_num1d * i, p_num1d);
        (PolyCoeff.row(i)).segment(2 * p_num1d, p_num1d) = Pz.segment(p_num1d * i, p_num1d);
    }

    return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective()
{
    _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
                _Pz.transpose() * _Q * _Pz)(0);
    return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t)
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

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t)
{
    Vector3d ret;
    int _poly_num1D = (int)polyCoeff.cols() / 3;
    for (int dim = 0; dim < 3; dim++)
    {
        VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
        VectorXd time = VectorXd::Zero(_poly_num1D);

        for (int j = 0; j < _poly_num1D; j++)
            if (j == 0)
                time(j) = 0.0;
            else
                time(j) = j * pow(t, j - 1);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t)
{
    Vector3d ret;
    int _poly_num1D = (int)polyCoeff.cols() / 3;
    for (int dim = 0; dim < 3; dim++)
    {
        VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
        VectorXd time = VectorXd::Zero(_poly_num1D);

        for (int j = 0; j < _poly_num1D; j++)
            if (j == 0 || j == 1)
                time(j) = 0.0;
            else
                time(j) = j * (j - 1) * pow(t, j - 2);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}
