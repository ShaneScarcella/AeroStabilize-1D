#pragma once

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double min_output, double max_output);

    double calculate(double setpoint, double current_value, double dt);
    
    void reset();

private:
    double _kp;
    double _ki;
    double _kd;

    double _min_output;
    double _max_output;

    double _integral;
    double _last_error;
    bool _first_update;
};