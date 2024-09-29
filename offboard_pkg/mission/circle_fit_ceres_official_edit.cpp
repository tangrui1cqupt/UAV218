// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// This fits circles to a collection of points, where the error is related to
// the distance of a point from the circle. This uses auto-differentiation to
// take the derivatives.
//
// The input format is simple text. Feed on standard in:
//
//   x_initial y_initial r_initial
//   x1 y1
//   x2 y2
//   y3 y3
//   ...
//
// And the result after solving will be printed to stdout:
//
//   x y r
//
// There are closed form solutions [1] to this problem which you may want to
// consider instead of using this one. If you already have a decent guess, Ceres
// can squeeze down the last bit of error.
//
//   [1] http://www.mathworks.com/matlabcentral/fileexchange/5557-circle-fit/content/circfit.m  // NOLINT
#include <cstdio>
#include <vector>
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
DEFINE_double(robust_threshold,
              0.0,
              "Robust loss parameter. Set to 0 for normal squared error (no "
              "robustification).");
// The cost for a single sample. The returned residual is related to the
// distance of the point from the circle (passed in as x, y, m parameters).
//
// Note that the radius is parameterized as r = m^2 to constrain the radius to
// positive values.把半径r写成m的平方是说为了保证半径是个正值
class DistanceFromCircleCost {
 public:
  DistanceFromCircleCost(double xx, double yy) : xx_(xx), yy_(yy) {}
  //DistanceFromCircleCost输入的参数是观测值,我这里观测值有三个  x y t 
  //DistanceFromCircleCost(double xx, double yy) : xx_(xx), yy_(yy), tt_(tt) {}
  template <typename T>
  bool operator()(const T* const x,
                  const T* const y,
                  const T* const m,  // r = m^2
                  T* residual) const {
    // Since the radius is parameterized as m^2, unpack m to get r.
    T r = *m * *m;
    // Get the position of the sample in the circle's coordinate system.
    T xp = xx_ - *x;
    T yp = yy_ - *y;
    // It is tempting to use the following cost:
    //
    //   residual[0] = r - sqrt(xp*xp + yp*yp);
    //
    // which is the distance of the sample from the circle. This works
    // reasonably well, but the sqrt() adds strong nonlinearities to the cost
    // function. Instead, a different cost is used, which while not strictly a
    // distance in the metric sense (it has units distance^2) it produces more
    // robust fits when there are outliers. This is because the cost surface is
    // more convex.
    residual[0] = r * r - xp * xp - yp * yp;
    //residual[0] = x - (xc + 1*cos(a*t+b))   //这里面应该就两种值，观测值和估计值，DistanceFromCircleCost输入的参数是观测值，输入的参数是估计值
    //residual[0] = y - (yc + 1*sin(a*t+b))
    return true;
  }
 private:
  // The measured x,y coordinate that should be on the circle.
  double xx_, yy_;
};

class DistanceFromCircleTrajectoryCost {
 public:
  //DistanceFromCircleCost(double xx, double yy) : xx_(xx), yy_(yy) {}
  //DistanceFromCircleCost输入的参数是观测值,我这里观测值有三个  x y t 
  DistanceFromCircleTrajectoryCost(double xx, double yy, double tt) : xx_(xx), yy_(yy), tt_(tt) {}
  template <typename T>
  bool operator()(const T* const x,
                  const T* const y,
                  const T* const a, 
                  const T* const b, 
                  T* residual) const {
    // Since the radius is parameterized as m^2, unpack m to get r.
    //T r = *m * *m;
    // Get the position of the sample in the circle's coordinate system.
    //T xp = xx_ - *x;
    //T yp = yy_ - *y;
    // It is tempting to use the following cost:
    //
    //   residual[0] = r - sqrt(xp*xp + yp*yp);
    //
    // which is the distance of the sample from the circle. This works
    // reasonably well, but the sqrt() adds strong nonlinearities to the cost
    // function. Instead, a different cost is used, which while not strictly a
    // distance in the metric sense (it has units distance^2) it produces more
    // robust fits when there are outliers. This is because the cost surface is
    // more convex.
    //residual[0] = r * r - xp * xp - yp * yp;
    //residual[0] = x - (xc + 1*cos(a*t+b))   //这里面应该就两种值，观测值和估计值，DistanceFromCircleCost输入的参数是观测值，输入的参数是估计值
    //注意下面式子中不要出现如1这种int类型的数字，编译会报错，得是double类型的，比如0.5就OK
    residual[0] = xx_ - (*x + 0.5*cos(*a * tt_ + *b));
    residual[1] = yy_ - (*y + 0.5*sin(*a * tt_ + *b));
    //residual[0] = y - (yc + 1*sin(a*t+b))
    return true;
  }
 private:
  // The measured x,y coordinate that should be on the circle.
  double xx_, yy_, tt_;
};

int main(int argc, char** argv) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  double x, y, a ,b;
  if (scanf("%lg %lg %lg %lg", &x, &y, &a, &b) != 4) {
    fprintf(stderr, "Couldn't read first line.\n");
    return 1;
  }
  fprintf(stderr, "Got x, y, a, b %lg, %lg, %lg, %lg\n", x, y, a, b);
  // Save initial values for comparison.
  double initial_x = x;
  double initial_y = y;
  double initial_a = a;
  double initial_b = b;
  // Parameterize r as m^2 so that it can't be negative.
  //double m = sqrt(r);
  Problem problem;
  // Configure the loss function.
  LossFunction* loss = nullptr;
  if (CERES_GET_FLAG(FLAGS_robust_threshold)) {
    loss = new CauchyLoss(CERES_GET_FLAG(FLAGS_robust_threshold));
  }
  // Add the residuals.
  double xx, yy, tt;
  int num_points = 0;
  //下面 2, 1, 1, 1, 1，其中2是残差的维度，1 1 1 1是四个优化参数的维度
  while (scanf("%lf %lf %lf\n", &xx, &yy, &tt) == 3) {
    CostFunction* cost =
        new AutoDiffCostFunction<DistanceFromCircleTrajectoryCost, 2, 1, 1, 1, 1>(
            new DistanceFromCircleTrajectoryCost(xx, yy, tt));
    problem.AddResidualBlock(cost, loss, &x, &y, &a, &b);
    num_points++;
  }
  std::cout << "Got " << num_points << " points.\n";
  // Build and solve the problem.
  Solver::Options options;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  // Recover r from m.
  //r = m * m;
  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x << " -> " << x << "\n";
  std::cout << "y : " << initial_y << " -> " << y << "\n";
  std::cout << "a : " << initial_a << " -> " << a << "\n";
  std::cout << "b : " << initial_b << " -> " << b << "\n";
  return 0;
}
