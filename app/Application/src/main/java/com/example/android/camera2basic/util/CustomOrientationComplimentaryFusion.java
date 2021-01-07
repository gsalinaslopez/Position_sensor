package com.example.android.camera2basic.util;

import android.hardware.SensorManager;
import android.util.Log;

import org.apache.commons.math3.complex.Quaternion;

/**
 * Created by gio on 3/20/18.
 */

public class CustomOrientationComplimentaryFusion extends CustomOrientationFusion {
    private static final String tag = CustomOrientationComplimentaryFusion.class.getSimpleName();

    private float[] output;

    /**
     * Initialize a singleton instance.
     */
    public CustomOrientationComplimentaryFusion() {
        this(DEFAULT_TIME_CONSTANT);

        output = new float[3];
    }

    public CustomOrientationComplimentaryFusion(float timeConstant) {
        super(timeConstant);

        output = new float[3];
    }

    /**
     * Calculate the fused orientation of the device.
     * @param gyroscope the gyroscope measurements.
     * @param dt the gyroscope delta
     * @param acceleration the acceleration measurements
     * @param magnetic the magnetic measurements
     * @return the fused orientation estimation.
     */
    protected float[] calculateFusedOrientation(float[] gyroscope, float dt, float[] acceleration, float[] magnetic) {

        float[] baseOrientation = getBaseOrientation(acceleration, magnetic);

        if (baseOrientation != null) {
            float alpha = timeConstant / (timeConstant + dt);
            float oneMinusAlpha = (1.0f - alpha);

            Quaternion rotationVectorAccelerationMagnetic = rotationVectorToQuaternion(baseOrientation);
            initializeRotationVectorGyroscopeIfRequired(rotationVectorAccelerationMagnetic);

            rotationVectorGyroscope = getGyroscopeRotationVector(rotationVectorGyroscope, gyroscope, dt);

            // Apply the complementary filter. // We multiply each rotation by their
            // coefficients (scalar matrices)...
            Quaternion scaledRotationVectorAccelerationMagnetic = rotationVectorAccelerationMagnetic.multiply
                    (oneMinusAlpha);

            // Scale our quaternion for the gyroscope
            Quaternion scaledRotationVectorGyroscope = rotationVectorGyroscope.multiply(alpha);

            // ...and then add the two quaternions together.
            // output[0] = alpha * output[0] + (1 - alpha) * input[0];
            rotationVectorGyroscope = scaledRotationVectorGyroscope.add
                    (scaledRotationVectorAccelerationMagnetic);

            // Now we get a structure we can pass to get a rotation matrix, and then
            // an orientation vector from Android.

            float[] fusedVector = new float[4];

            fusedVector[0] = (float) rotationVectorGyroscope.getVectorPart()[0];
            fusedVector[1] = (float) rotationVectorGyroscope.getVectorPart()[1];
            fusedVector[2] = (float) rotationVectorGyroscope.getVectorPart()[2];
            fusedVector[3] = (float) rotationVectorGyroscope.getScalarPart();

            // rotation matrix from gyro data
            float[] fusedMatrix = new float[16];

            // We need a rotation matrix so we can get the orientation vector...
            // Getting Euler
            // angles from a quaternion is not trivial, so this is the easiest way,
            // but perhaps
            // not the fastest way of doing this.
            SensorManager.getRotationMatrixFromVector(fusedMatrix, fusedVector);

            // Get the fused orienatation
            SensorManager.getOrientation(fusedMatrix, output);

            return fusedMatrix;
        }

        // The device had a problem determining the base orientation from the acceleration and magnetic sensors,
        // possible because of bad inputs or possibly because the device determined the orientation could not be
        // calculated, e.g the device is in free-fall
        Log.w(tag, "Base Device Orientation could not be computed!");

        return null;
    }

    /**
     * Calculate the fused orientation of the device.
     * @param gyroscope the gyroscope measurements.
     * @param dt the gyroscope delta
     * @param orientation an estimation of device orientation.
     * @return the fused orientation estimation.
     */
    protected float[] calculateFusedOrientation(float[] gyroscope, float dt, float[] orientation) {
        float[] baseOrientation = orientation;

        if (baseOrientation != null) {
            float alpha = timeConstant / (timeConstant + dt);
            float oneMinusAlpha = (1.0f - alpha);

            Quaternion baseOrientationQuaternion = rotationVectorToQuaternion(baseOrientation);

            initializeRotationVectorGyroscopeIfRequired(baseOrientationQuaternion);

            rotationVectorGyroscope = getGyroscopeRotationVector(rotationVectorGyroscope, gyroscope, dt);

            // Apply the complementary filter. // We multiply each rotation by their
            // coefficients (scalar matrices)...
            Quaternion scaledRotationVectorAccelerationMagnetic = baseOrientationQuaternion.multiply
                    (oneMinusAlpha);

            // Scale our quaternion for the gyroscope
            Quaternion scaledRotationVectorGyroscope = rotationVectorGyroscope.multiply(alpha);

            // ...and then add the two quaternions together.
            // output[0] = alpha * output[0] + (1 - alpha) * input[0];
            rotationVectorGyroscope = scaledRotationVectorGyroscope.add
                    (scaledRotationVectorAccelerationMagnetic);

            // Now we get a structure we can pass to get a rotation matrix, and then
            // an orientation vector from Android.

            float[] fusedVector = new float[4];

            fusedVector[0] = (float) rotationVectorGyroscope.getVectorPart()[0];
            fusedVector[1] = (float) rotationVectorGyroscope.getVectorPart()[1];
            fusedVector[2] = (float) rotationVectorGyroscope.getVectorPart()[2];
            fusedVector[3] = (float) rotationVectorGyroscope.getScalarPart();

            // rotation matrix from gyro data
            float[] fusedMatrix = new float[16];

            // We need a rotation matrix so we can get the orientation vector...
            // Getting Euler
            // angles from a quaternion is not trivial, so this is the easiest way,
            // but perhaps
            // not the fastest way of doing this.
            SensorManager.getRotationMatrixFromVector(fusedMatrix, fusedVector);

            // Get the fused orienatation
            SensorManager.getOrientation(fusedMatrix, output);

            return fusedMatrix;
        }

        // The device had a problem determining the base orientation from the acceleration and magnetic sensors,
        // possible because of bad inputs or possibly because the device determined the orientation could not be
        // calculated, e.g the device is in free-fall
        Log.w(tag, "Base Device Orientation could not be computed!");

        return null;
    }

    @Override
    public void startFusion() {}

    @Override
    public void stopFusion() {}
}