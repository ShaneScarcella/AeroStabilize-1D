#include "PIDController.hpp"

PIDController::PIDController(double kp, double ki, double kd, double min_output, double max_output)
    : _kp(kp), _ki(ki), _kd(kd), _min_output(min_output), _max_output(max_output),
      _integral(0.0), _last_error(0.0), _first_update(true) {
}

double PIDController::calculate(double setpoint, double current_value, double dt) {
    double error = setpoint - current_value;

    if (_first_update) {
        _last_error = error;
        _first_update = false;
    }

    double proportional = _kp * error;

    _integral += error * dt;

    double derivative = (error - _last_error) / dt;
    _last_error = error;

    double output = proportional + (_ki * _integral) + (_kd * derivative);

    if (output > _max_output) {
        output = _max_output;
        _integral -= error * dt; 
    } else if (output < _min_output) {
        output = _min_output;
        _integral -= error * dt; 
    }

    return output;
}

void PIDController::reset() {
    _integral = 0.0;
    _last_error = 0.0;
    _first_update = true;
}