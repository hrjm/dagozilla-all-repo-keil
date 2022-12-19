#include "PIDController.h"

//double PIDController::c1_ = 0.;
//double PIDController::c2_ = 0.;
//double PIDController::d0_ = 0.;
//double PIDController::d1_ = 0.;
//double PIDController::d2_ = 0.;
//double PIDController::prev_err_[2] = {0., 0.};
//double PIDController::prev_out_[2] = {0., 0.};
PIDController::PIDController(int mode, float kp, float ti, float td, float ff, float fc, float cp, bool is_active)
    : is_active_(is_active)
{
    double Kp = kp;

    switch (mode)
    {
    case 0:
    {
        d0_ = Kp;
        break;
    }
    case 1:
    {
        double KiTs = Kp / ti * cp;
        c1_ = 1.;
        d0_ = Kp + KiTs;
        d1_ = -Kp;
        break;
    }
    case 2:
    {
        double KdN = Kp * td * fc;
        double _1_NTs1 = 1 / (1 + fc * cp);
        c1_ = _1_NTs1;
        d0_ = Kp + KdN * _1_NTs1;
        d1_ = -(Kp + KdN) * _1_NTs1;
        break;
    }
    case 3:
    {
        double KiTs = Kp / ti * cp;
        double KdN = Kp * td * fc;
        double _1_NTs1 = 1 / (1 + fc * cp);
        c1_ = 1. + _1_NTs1;
        c2_ = -_1_NTs1;
        d0_ = Kp + KiTs + KdN * _1_NTs1;
        d1_ = -Kp - (Kp + KiTs + 2. * KdN) * _1_NTs1;
        d2_ = (Kp + KdN) * _1_NTs1;
        break;
    }
    }
}

double PIDController::clamp(double val, double min, double max)
{
    if (val >= max)
    {
        return max;
    }
    else if (val <= min)
    {
        return min;
    }
    else
    {
        return val;
    }
}
double PIDController::compute_action(double target, double feedback, float ff)
{
    double err = target - feedback;
    double out = 0.;
    if (is_active_)
    {
        out = c1_ * prev_out_[0] + c2_ * prev_out_[1] + d0_ * err + d1_ * prev_err_[0] + d2_ * prev_err_[1];
        out = clamp(out, -1., 1.);
    }

    prev_err_[1] = prev_err_[0];
    prev_err_[0] = err;
    prev_out_[1] = prev_out_[0];
    prev_out_[0] = out;

    return clamp(out + ff * target, -1., 1.);
}
