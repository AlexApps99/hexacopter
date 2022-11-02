using UnityEngine;
using AHRS;

[System.Serializable]
public class PIDParameters {
    [Min(0)]
    public float Kp = 1f;
    [Min(0)]
    public float Ki = 0f;
    [Min(0)]
    public float Kd = 0f;

    public PID Create(float dt) { return new PID(Kp, Ki, Kd, dt); }
}

public class DronePID : DronePhys {
    [Range(0,1)]
    public float gyroFac = 0.98f;
    [Header("PID tuning")]
    public PIDParameters pitch;
    private PID ppitch;
    public PIDParameters roll;
    private PID proll;
    public PIDParameters yaw;
    private PID pyaw;
    public PIDParameters thrust;
    private PID pthrust;
    public bool approxSensor = true;

    private MadgwickAHRS ahrs;
    public float targetHeight = 5f;
    [Range(-Mathf.PI, Mathf.PI)]
    public float targetYaw = 0f;

    Quaternion UpdateIMU(Vector3 accel, Vector3 gyro, Vector3 magneto) {
        magneto = -magneto;
        ahrs.Update(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, magneto.x, magneto.y, magneto.z);
        if (approxSensor) {
            float[] q = ahrs.Quaternion;
            return new Quaternion(q[0],q[1],q[2],q[3]);
        } else {
            // TODO i think i just need to reassign these axes
            return Quaternion.AngleAxis(90, Vector3.right) * transform.rotation;
        }
    }

    // Asin, but without the NaN
    float Asin(float x) {
        if (x >= 1) return Mathf.PI / 2f;
        else if (x <= -1) return -Mathf.PI / 2f;
        else return Mathf.Asin(x);
    }

    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
    Vector3 GetYPR(Quaternion q) {
        // This is bad:
        // the yaw seems to repeat twice because pitch/roll invert
        float roll = Mathf.Atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y));
        float pitch = Asin(2*(q.w*q.y - q.z*q.x));
        float yaw  = -Mathf.Atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
        // TODO bad hack
        //if (approxSensor) {
        return new Vector3(yaw, pitch, roll);
        //} else {
        //    return new Vector3(pitch, yaw, roll);
        //}
    }

    new private void Awake() {
        base.Awake();
        // Go faster so I can test faster
        Time.timeScale = 5f;
        ahrs = new MadgwickAHRS(Time.fixedDeltaTime);
        ppitch = pitch.Create(Time.fixedDeltaTime);
        proll = roll.Create(Time.fixedDeltaTime);
        pyaw = yaw.Create(Time.fixedDeltaTime);
        pthrust = thrust.Create(Time.fixedDeltaTime);
    }

    //protected override void UpdateControl(Vector3 accel, Vector3 gyro, Vector3 magneto) {
    //    float m = 40f;
    //    float v1 = (Mathf.Sin(Time.time) + 1f) * m;
    //    float v2 = 0f;//(Mathf.Cos(Time.time) + 1f) * m;
    //    for (uint i = 0; i < N; i++) {
    //        freqs[i] = (((i&1)!=0) ? v1 : v2);
    //    }
    //}

    string csv = "y\n";
    private float UnfudgeAngle(float a, float b) {
        const float TAU = 2f * Mathf.PI;
        float v = a - b;
        v += Mathf.PI;
        v -= Mathf.Floor(v/TAU) * TAU;
        v -= Mathf.PI;
        return v;
    }
    private void OnApplicationQuit() {
        GUIUtility.systemCopyBuffer = csv;
    }
    protected override void UpdateControl(Vector3 accel, Vector3 gyro, Vector3 magneto) {
        Quaternion bearing = UpdateIMU(accel, gyro, magneto);
        Vector3 ypr = GetYPR(bearing);
        Debug.Log(ypr);
        float y = pyaw.Iterate(UnfudgeAngle(targetYaw, ypr.x));
        float p = ppitch.Iterate(UnfudgeAngle(0f, ypr.y));
        float r = proll.Iterate(UnfudgeAngle(0f, ypr.z));
        // This is cheating, the drone won't know this
        float t = pthrust.Iterate(targetHeight, transform.position.y);
        csv += y + "\n";
        for (uint i = 0; i < N; i++) {
            RotorState rot = rotorData[i];
            // Mix motors
            freqs[i] = t;
            freqs[i] += r * rot.normPos.x;
            freqs[i] += p * rot.normPos.y;
            freqs[i] += y * (rot.ccw ? 1 : -1);
        }
    }
}
