using UnityEngine;

// https://en.wikipedia.org/wiki/PID_controller#Pseudocode
// TODO tuning? Ziegler-Nichols? IIR filter?
public class PID {
    private float _Kp;
    private float _Ki;
    private float _Kd;
    private float _dt;

    private float A0;
    private float A1;
    private float A2;

    // Proportional
    public float Kp {
        get => _Kp;
        set {
            _Kp = value;
            GenA();
        }
    }
    // Integral
    public float Ki {
        get => _Ki;
        set {
            _Ki = value;
            GenA();
        }
    }
    // Derivative
    public float Kd {
        get => _Kd;
        set {
            _Kd = value;
            GenA();
        }
    }
    // delta time
    public float dt {
        get => _dt;
        set {
            _dt = value;
            GenA();
        }
    }

    private float error0;
    private float error1;
    private float error2;
    private float output;

    // Initialize PID with PID factors and delta time
    public PID(float Kp, float Ki, float Kd, float dt) {
        _Kp = Kp;
        _Ki = Ki;
        _Kd = Kd;
        _dt = dt;
        Reset();
        GenA();
    }

    // Iterate PID with setpoint and measured value
    public float Iterate(float setpoint, float measuredValue) {
        error2 = error1;
        error1 = error0;
        error0 = setpoint - measuredValue;
        output += A0 * error0 + A1 * error1 + A2 * error2;
        return output;
    }

    // Iterate PID with error as setpoint - measured value
    public float Iterate(float error) {
        error2 = error1;
        error1 = error0;
        error0 = error;
        output += A0 * error0 + A1 * error1 + A2 * error2;
        return output;
    }

    // Reset internal state of PID
    public void Reset() {
        error0 = 0f;
        error1 = 0f;
        error2 = 0f;
        output = 0f;
    }

    private void GenA() {
        A0 = _Kp + _Ki * _dt + _Kd / _dt;
        A1 = -_Kp - 2 * _Kd / _dt;
        A2 = _Kd / _dt;
    }
}
