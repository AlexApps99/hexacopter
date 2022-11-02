using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public struct RotorState {
    public bool ccw;
    public Vector2 normPos;
    public Vector2 pos;
    public float angle;
    public Transform rotor;
}


public class DronePhys : MonoBehaviour {
    protected const uint N = 6;

    // Angular frequencies
    [SerializeField]
    [Min(0)]
    protected float[] freqs = new float[N];
    // Maximum frequency
    [SerializeField]
    [Min(0)]
    protected float maxFreq = 10000f;
    private Rigidbody rb = null;
    [Header("Visualization")]
    [SerializeField]
    private Transform[] rotors;
    [SerializeField]
    [Min(0)]
    private float rotorRadius = .75f;
    // Multiply omega^2 by this to get the right unit
    [Header("Physics constants")]
    [SerializeField]
    [Min(0)]
    private float torqueFac = 1e-5f;
    [SerializeField]
    [Min(0)]
    private float forceFac = 0.002323f;

    private Vector3[] prevVels = new Vector3[10];

    protected RotorState[] rotorData;

    // Passed accelerometer, gyroscope, and magnetometer input
    // Should set freqs, they will be clamped between 0 and maxFreq
    protected virtual void UpdateControl(Vector3 accel, Vector3 gyro, Vector3 magneto) {}

    private void GenRotors() {
        rotorData = new RotorState[N];
        for (uint i = 0; i < N; i++) {
            float angle = ((float)i * Mathf.PI * 2f) / (float)N;
            float x = Mathf.Cos(angle);
            float z = Mathf.Sin(angle);

            rotorData[i] = new RotorState {
                normPos = new Vector2(x, z),
                pos = new Vector2(x, z) * rotorRadius,
                ccw = (i & 1) == 0,
                rotor = (rotors.Length >= N) ? rotors[i] : null,
                angle = angle,
            };
        }
    }

    // Start is called before the first frame update
    protected void Awake() {
        if (rb == null)
            rb = GetComponent<Rigidbody>();
        // Don't sleep
        rb.sleepThreshold = 0f;
        GenRotors();
    }

    private float Freq2Force(float fr) {
        return Mathf.Sign(fr) * fr * fr * forceFac;
    }

    private float GetTorque() {
        float v = 0;
        for (uint i = 0; i < N; i++) {
            RotorState rot = rotorData[i];
            // Torque is proportional to angular frequency squared
            v -= Mathf.Sign(freqs[i]) * (rot.ccw ? torqueFac : -torqueFac) * freqs[i] * freqs[i];
        }
        return v;
    }

    private Vector3 GetAccel() {
        Vector3 curVel = Vector3.zero;
        int n1 = 0;
        for (int i = 0; i < prevVels.Length / 2; i++) {
            curVel += prevVels[i];
            n1++;
        }
        curVel /= n1;

        Vector3 prevVel = Vector3.zero;
        int n2 = 0;
        for (int i = prevVels.Length / 2; i < prevVels.Length; i++) {
            prevVel += prevVels[i];
            n2++;
        }
        prevVel /= n2;

        Vector3 acc = (curVel - prevVel) / (Time.fixedDeltaTime * (prevVels.Length / 2f));
        return Physics.gravity - acc;
    }

    private Vector3 GetMagneto() {
        return transform.forward;
    }

    private Vector3 GetGyro() {
        return rb.angularVelocity;
    }

    private void FixedUpdate() {
        for (int i = prevVels.Length-1; i-1 >= 0; i--) prevVels[i] = prevVels[i-1];
        prevVels[0] = rb.velocity;

        Vector3 accel = GetAccel();
        Vector3 gyro = GetGyro();
        Vector3 magneto = GetMagneto();

        UpdateControl(accel, gyro, magneto);
        for (uint i = 0; i < N; i++) freqs[i] = Mathf.Clamp(freqs[i], 0f, maxFreq);

        rb.AddTorque(transform.up * GetTorque());
        for (uint i = 0; i < N; i++) {
            RotorState rot = rotorData[i];
            Vector3 p = transform.TransformPoint(rot.pos.x, 0, rot.pos.y);
            Vector3 r = transform.TransformVector(0, Freq2Force(freqs[i]), 0);
            rb.AddForceAtPosition(r, p);
            if (rot.rotor != null) {
                rot.rotor.Rotate(0, (rot.ccw ? 1 : -1) * Mathf.Rad2Deg * freqs[i] * Time.fixedDeltaTime, 0);
            }
        }
    }

    // Update is called once per frame
    private void Update() {
        Debug.DrawRay(transform.position, GetAccel(), Color.green);
        Debug.DrawRay(transform.position, GetMagneto(), Color.red);

        for (uint i = 0; i < N; i++) {
            RotorState rot = rotorData[i];
            Vector3 p = transform.TransformPoint(rot.pos.x, 0, rot.pos.y);
            Vector3 r = transform.TransformVector(0, Freq2Force(freqs[i]), 0);
            Debug.DrawRay(p, r, Color.green);
        }
    }
}
