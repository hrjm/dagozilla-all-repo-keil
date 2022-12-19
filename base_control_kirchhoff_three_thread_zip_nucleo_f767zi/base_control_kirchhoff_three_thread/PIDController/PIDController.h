#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController
{
public:
   PIDController(int mode, float kp, float ti, float td, float ff, float fc, float cp, bool is_active);
    double clamp(double val, double min, double max);
    double compute_action(double target, double feedback, float ff);
    void SetActive(bool is_active) { is_active_ = is_active; }

private:
    /// ci = -ai/a0, di = bi/a0 in discrete-time PID
    double c1_ , c2_, d0_ , d1_ , d2_;
    double prev_err_[2] , prev_out_[2];
    bool is_active_;
};

#endif