/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Mark Moll (OMPL). Distance-only adaptation; no segment execution.
// Upstream: ompl/ompl 1.4.2, c7fc1452bb77d5dee417e25a6ce2b2c5feb6fd5c
// src/ompl/base/spaces/src/ReedsSheppStateSpace.cpp
// Local changes: retain only path lengths and expose normalized SE(2) distance;
// remove OMPL state allocation, motion validation and interpolation interfaces.

#include "reeds_shepp_distance.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace ugv_reset_safety {
namespace {
// Only the metric is consumed; segment types/interpolation are omitted.
struct PathLength {
    explicit PathLength(double t = std::numeric_limits<double>::max(), double u = 0.0,
                        double v = 0.0, double w = 0.0, double x = 0.0)
        : total(std::abs(t) + std::abs(u) + std::abs(v) + std::abs(w) + std::abs(x)) {}
    double length() const {
        return total;
    }
    double total;
};

// The comments, variable names, etc. use the nomenclature from the Reeds & Shepp paper.

constexpr double pi = 3.14159265358979323846;
const double twopi = 2. * pi;
const double RS_EPS = 1e-6;
const double ZERO = 10 * std::numeric_limits<double>::epsilon();

inline double mod2pi(double x) {
    double v = fmod(x, twopi);
    if (v < -pi) {
        v += twopi;
    } else if (v > pi) {
        v -= twopi;
    }
    return v;
}
inline void polar(double x, double y, double& r, double& theta) {
    r = sqrt(x * x + y * y);
    theta = atan2(y, x);
}
inline void tauOmega(double u, double v, double xi, double eta, double phi, double& tau,
                     double& omega) {
    double delta = mod2pi(u - v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1.;
    double t1 = atan2(eta * A - xi * B, xi * A + eta * B),
           t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
    tau = (t2 < 0) ? mod2pi(t1 + pi) : mod2pi(t1);
    omega = mod2pi(tau - u + v - phi);
}

// formula 8.1 in Reeds-Shepp paper
inline bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v) {
    polar(x - sin(phi), y - 1. + cos(phi), u, t);
    if (t >= -ZERO) {
        v = mod2pi(phi - t);
        if (v >= -ZERO) {
            assert(fabs(u * cos(t) + sin(phi) - x) < RS_EPS);
            assert(fabs(u * sin(t) - cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t + v - phi)) < RS_EPS);
            return true;
        }
    }
    return false;
}
// formula 8.2
inline bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v) {
    double t1, u1;
    polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
    u1 = u1 * u1;
    if (u1 >= 4.) {
        double theta;
        u = sqrt(u1 - 4.);
        theta = atan2(2., u);
        t = mod2pi(t1 + theta);
        v = mod2pi(t - phi);
        assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < RS_EPS);
        assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
        return t >= -ZERO && v >= -ZERO;
    }
    return false;
}
void CSC(double x, double y, double phi, PathLength& path) {
    double t, u, v, Lmin = path.length(), L;
    if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
        path = PathLength(t, u, v);
        Lmin = L;
    }
    if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
        path = PathLength(-t, -u, -v);
        Lmin = L;
    }
    if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
        path = PathLength(t, u, v);
        Lmin = L;
    }
    if (LpSpLp(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    {
        path = PathLength(-t, -u, -v);
        Lmin = L;
    }
    if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
        path = PathLength(t, u, v);
        Lmin = L;
    }
    if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
        path = PathLength(-t, -u, -v);
        Lmin = L;
    }
    if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
        path = PathLength(t, u, v);
        Lmin = L;
    }
    if (LpSpRp(-x, -y, phi, t, u, v) &&
        Lmin > (fabs(t) + fabs(u) + fabs(v))) {  // timeflip + reflect
        path = PathLength(-t, -u, -v);
    }
}
// formula 8.3 / 8.4  *** TYPO IN PAPER ***
inline bool LpRmL(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
    polar(xi, eta, u1, theta);
    if (u1 <= 4.) {
        u = -2. * asin(.25 * u1);
        t = mod2pi(theta + .5 * u + pi);
        v = mod2pi(phi - t + u);
        assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < RS_EPS);
        assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t - u + v - phi)) < RS_EPS);
        return t >= -ZERO && u <= ZERO;
    }
    return false;
}
void CCC(double x, double y, double phi, PathLength& path) {
    double t, u, v, Lmin = path.length(), L;
    if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
        path = PathLength(t, u, v);
        Lmin = L;
    }
    if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
        path = PathLength(-t, -u, -v);
        Lmin = L;
    }
    if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
        path = PathLength(t, u, v);
        Lmin = L;
    }
    if (LpRmL(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    {
        path = PathLength(-t, -u, -v);
        Lmin = L;
    }

    // backwards
    double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
    if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
        path = PathLength(v, u, t);
        Lmin = L;
    }
    if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
        path = PathLength(-v, -u, -t);
        Lmin = L;
    }
    if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
        path = PathLength(v, u, t);
        Lmin = L;
    }
    if (LpRmL(-xb, -yb, phi, t, u, v) &&
        Lmin > (fabs(t) + fabs(u) + fabs(v))) {  // timeflip + reflect
        path = PathLength(-v, -u, -t);
    }
}
// formula 8.7
inline bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = .25 * (2. + sqrt(xi * xi + eta * eta));
    if (rho <= 1.) {
        u = acos(rho);
        tauOmega(u, -u, xi, eta, phi, t, v);
        assert(fabs(2 * (sin(t) - sin(t - u) + sin(t - 2 * u)) - sin(phi) - x) < RS_EPS);
        assert(fabs(2 * (-cos(t) + cos(t - u) - cos(t - 2 * u)) + cos(phi) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t - 2 * u - v - phi)) < RS_EPS);
        return t >= -ZERO && v <= ZERO;
    }
    return false;
}
// formula 8.8
inline bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = (20. - xi * xi - eta * eta) / 16.;
    if (rho >= 0 && rho <= 1) {
        u = -acos(rho);
        if (u >= -.5 * pi) {
            tauOmega(u, u, xi, eta, phi, t, v);
            assert(fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x) < RS_EPS);
            assert(fabs(-4 * cos(t) + 2 * cos(t - u) + cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
            return t >= -ZERO && v >= -ZERO;
        }
    }
    return false;
}
void CCCC(double x, double y, double phi, PathLength& path) {
    double t, u, v, Lmin = path.length(), L;
    if (LpRupLumRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v))) {
        path = PathLength(t, u, -u, v);
        Lmin = L;
    }
    if (LpRupLumRm(-x, y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
    {
        path = PathLength(-t, -u, u, -v);
        Lmin = L;
    }
    if (LpRupLumRm(x, -y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
    {
        path = PathLength(t, u, -u, v);
        Lmin = L;
    }
    if (LpRupLumRm(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
    {
        path = PathLength(-t, -u, u, -v);
        Lmin = L;
    }

    if (LpRumLumRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v))) {
        path = PathLength(t, u, u, v);
        Lmin = L;
    }
    if (LpRumLumRp(-x, y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
    {
        path = PathLength(-t, -u, -u, -v);
        Lmin = L;
    }
    if (LpRumLumRp(x, -y, -phi, t, u, v) &&
        Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
    {
        path = PathLength(t, u, u, v);
        Lmin = L;
    }
    if (LpRumLumRp(-x, -y, phi, t, u, v) &&
        Lmin > (fabs(t) + 2. * fabs(u) + fabs(v))) {  // timeflip + reflect
        path = PathLength(-t, -u, -u, -v);
    }
}
// formula 8.9
inline bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
    polar(xi, eta, rho, theta);
    if (rho >= 2.) {
        double r = sqrt(rho * rho - 4.);
        u = 2. - r;
        t = mod2pi(theta + atan2(r, -2.));
        v = mod2pi(phi - .5 * pi - t);
        assert(fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x) < RS_EPS);
        assert(fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t + pi / 2 + v - phi)) < RS_EPS);
        return t >= -ZERO && u <= ZERO && v <= ZERO;
    }
    return false;
}
// formula 8.10
inline bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
    polar(-eta, xi, rho, theta);
    if (rho >= 2.) {
        t = theta;
        u = 2. - rho;
        v = mod2pi(t + .5 * pi - phi);
        assert(fabs(2 * sin(t) - cos(t - v) - u * sin(t) - x) < RS_EPS);
        assert(fabs(-2 * cos(t) - sin(t - v) + u * cos(t) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t + pi / 2 - v - phi)) < RS_EPS);
        return t >= -ZERO && u <= ZERO && v <= ZERO;
    }
    return false;
}
void CCSC(double x, double y, double phi, PathLength& path) {
    double t, u, v, Lmin = path.length() - .5 * pi, L;
    if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
        path = PathLength(t, -.5 * pi, u, v);
        Lmin = L;
    }
    if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
        path = PathLength(-t, .5 * pi, -u, -v);
        Lmin = L;
    }
    if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
        path = PathLength(t, -.5 * pi, u, v);
        Lmin = L;
    }
    if (LpRmSmLm(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    {
        path = PathLength(-t, .5 * pi, -u, -v);
        Lmin = L;
    }

    if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
        path = PathLength(t, -.5 * pi, u, v);
        Lmin = L;
    }
    if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
        path = PathLength(-t, .5 * pi, -u, -v);
        Lmin = L;
    }
    if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
        path = PathLength(t, -.5 * pi, u, v);
        Lmin = L;
    }
    if (LpRmSmRm(-x, -y, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    {
        path = PathLength(-t, .5 * pi, -u, -v);
        Lmin = L;
    }

    // backwards
    double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
    if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
        path = PathLength(v, u, -.5 * pi, t);
        Lmin = L;
    }
    if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
        path = PathLength(-v, -u, .5 * pi, -t);
        Lmin = L;
    }
    if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
        path = PathLength(v, u, -.5 * pi, t);
        Lmin = L;
    }
    if (LpRmSmLm(-xb, -yb, phi, t, u, v) &&
        Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    {
        path = PathLength(-v, -u, .5 * pi, -t);
        Lmin = L;
    }

    if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
        path = PathLength(v, u, -.5 * pi, t);
        Lmin = L;
    }
    if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
        path = PathLength(-v, -u, .5 * pi, -t);
        Lmin = L;
    }
    if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
        path = PathLength(v, u, -.5 * pi, t);
        Lmin = L;
    }
    if (LpRmSmRm(-xb, -yb, phi, t, u, v) &&
        Lmin > (fabs(t) + fabs(u) + fabs(v))) {  // timeflip + reflect
        path = PathLength(-v, -u, .5 * pi, -t);
    }
}
// formula 8.11 *** TYPO IN PAPER ***
inline bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v) {
    double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
    polar(xi, eta, rho, theta);
    if (rho >= 2.) {
        u = 4. - sqrt(rho * rho - 4.);
        if (u <= ZERO) {
            t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
            v = mod2pi(t - phi);
            assert(fabs(4 * sin(t) - 2 * cos(t) - u * sin(t) - sin(phi) - x) < RS_EPS);
            assert(fabs(-4 * cos(t) - 2 * sin(t) + u * cos(t) + cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
            return t >= -ZERO && v >= -ZERO;
        }
    }
    return false;
}
void CCSCC(double x, double y, double phi, PathLength& path) {
    double t, u, v, Lmin = path.length() - pi, L;
    if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) {
        path = PathLength(t, -.5 * pi, u, -.5 * pi, v);
        Lmin = L;
    }
    if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
    {
        path = PathLength(-t, .5 * pi, -u, .5 * pi, -v);
        Lmin = L;
    }
    if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
    {
        path = PathLength(t, -.5 * pi, u, -.5 * pi, v);
        Lmin = L;
    }
    if (LpRmSLmRp(-x, -y, phi, t, u, v) &&
        Lmin > (fabs(t) + fabs(u) + fabs(v))) {  // timeflip + reflect
        path = PathLength(-t, .5 * pi, -u, .5 * pi, -v);
    }
}

PathLength reedsShepp(double x, double y, double phi) {
    PathLength path;
    CSC(x, y, phi, path);
    CCC(x, y, phi, path);
    CCCC(x, y, phi, path);
    CCSC(x, y, phi, path);
    CCSCC(x, y, phi, path);
    return path;
}
}  // namespace

double reedsSheppDistance(double x, double y, double yaw) {
    return reedsShepp(x, y, yaw).length();
}
}  // namespace ugv_reset_safety
