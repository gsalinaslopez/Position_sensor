package com.example.android.camera2basic.livedata;

/**
 * Created by gio on 3/5/18.
 */

import android.app.Activity;
import android.arch.lifecycle.LiveData;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.support.annotation.MainThread;
import android.util.Log;
import android.view.Surface;

import com.example.android.camera2basic.util.CustomOrientationComplimentaryFusion;
import com.example.android.camera2basic.util.CustomOrientationFusion;
import com.kircherelectronics.fsensor.filter.averaging.LowPassFilter;
import com.kircherelectronics.fsensor.filter.fusion.OrientationComplimentaryFusion;
import com.kircherelectronics.fsensor.filter.fusion.OrientationFusion;
import com.kircherelectronics.fsensor.filter.fusion.OrientationKalmanFusion;
import com.kircherelectronics.fsensor.linearacceleration.LinearAcceleration;
import com.kircherelectronics.fsensor.linearacceleration.LinearAccelerationFusion;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

/**
 * Created by gio on 2/26/18.
 */

public class PositionSensorLiveData extends LiveData<double[]> {

    public final static int SENSOR_UNAVAILABLE = -1;
    private final static int sensorSpeed = SensorManager.SENSOR_DELAY_NORMAL;
    // Static instance for simpleton implementation
    private static PositionSensorLiveData sInstance;

    Activity mActivity;
    Context mContext;

    SensorManager mSensorManager;
    PositionSensorListener mPositionSensorListener;

    // FSensor filters and sensor fusion
    private LowPassFilter mLpfAccelerationSmoothing;
    private LinearAcceleration mLinearAccelerationFilterComplimentary;
    private LinearAcceleration mLinearAccelerationFilterKalman;
    private OrientationFusion mOrientationFusionComplimentary;
    private OrientationFusion mOrientationFusionKalman;
    private CustomOrientationFusion mCustomOrientationFusionComplimentary;

    // raw inputs from Android sensors
    float mNormGravity;           // length of raw gravity vector received in onSensorChanged(...).  NB: should be about 10
    float[] mNormGravityVector = new float[3];    // Normalised gravity vector, (i.e. length of this vector is 1), which points straight up into space
    float mNormMagneticField;          // length of raw magnetic field vector received in onSensorChanged(...).
    float[] mNormMagneticFieldValues = new float[3];   // Normalised magnetic field vector, (i.e. length of this vector is 1)

    // accuracy specifications. SENSOR_UNAVAILABLE if unknown, otherwise SensorManager.SENSOR_STATUS_UNRELIABLE, SENSOR_STATUS_ACCURACY_LOW, SENSOR_STATUS_ACCURACY_MEDIUM or SENSOR_STATUS_ACCURACY_HIGH
    int mGravityAccuracy;          // accuracy of gravity sensor
    int mMagneticFieldAccuracy;    // accuracy of magnetic field sensor

    // values calculated once gravity and magnetic field vectors are available
    float[] mNormEastVector = new float[3];       // normalised cross product of raw gravity vector with magnetic field values, points east
    float[] mNormNorthVector = new float[3];      // Normalised vector pointing to magnetic north
    boolean mOrientationOK = false;        // set true if m_azimuth_radians and m_pitch_radians have successfully been calculated following a call to onSensorChanged(...)
    float mAzimuthRadians = 0;        // angle of the device from magnetic north
    float mAzimuthRadiansPrev = 0;
    float mPitchRadians;          // tilt angle of the device from the horizontal.  m_pitch_radians = 0 if the device if flat, m_pitch_radians = Math.PI/2 means the device is upright.
    float mPitchAxisRadians;     // angle which defines the axis for the rotation m_pitch_radians

    double[] mCurrentpos = new double[2];

    private float[] mAccelerometerReading = new float[3];
    private float[] mMagnetometerReading = new float[3];
    private float[] mRotation = new float[3];
    private float[] mRotationMatrix = new float[16];
    private float[] mFusedOrientation = new float[3];

    private int mSteps = 0;

    private static int RMS_WINDOW = 10;
    private static float X_RMS_THRESHOLD = 1;
    private static float Y_RMS_THRESHOLD = 1;
    private Queue<Float> xAcceleration = new LinkedList<Float>();
    private int X_CURRENT_SIGN = 0;
    private Queue<Float> yAcceleration = new LinkedList<Float>();
    private int Y_CURRENT_SIGN = 0;

    private Queue<Float> eastAcceleration = new LinkedList<Float>();
    private int EAST_CURRENT_SIGN = 0;
    private Queue<Float> northAcceleration = new LinkedList<Float>();
    private int NORTH_CURRENT_SIGN = 0;
    private double[] output = new double[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // [0] x raw, [1] x square, [2] y raw, [3] y square, [4] compass, [5] pitch, [6] pos0, [7] pos1
    // [8] globeast, [9] globeast square, [10] globnorth, [11] globnorth square

    @MainThread
    public static PositionSensorLiveData get(Context context) {
        if (sInstance == null) {
            sInstance = new PositionSensorLiveData(context);
        }
        return sInstance;
    }

    public PositionSensorLiveData(Context context) {
        mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        mPositionSensorListener = new PositionSensorListener();

        initFiltersAndSensorFusion();
        registerPositionSensorListener();
    }

    public void setParams(float eventThreshold, float xRmsThreshold, float yRmsThreshold) {
        X_RMS_THRESHOLD = xRmsThreshold;
        Y_RMS_THRESHOLD = yRmsThreshold;
    }

    @Override
    protected void onActive() {
        registerPositionSensorListener();
    }

    @Override
    protected void onInactive() {
        mSensorManager.unregisterListener(mPositionSensorListener);
    }

    private void initFiltersAndSensorFusion() {
        mLpfAccelerationSmoothing = new LowPassFilter();
        mLpfAccelerationSmoothing.setTimeConstant(0.5f);
        mLpfAccelerationSmoothing.reset();

        mOrientationFusionComplimentary = new OrientationComplimentaryFusion();
        mOrientationFusionComplimentary.setTimeConstant(0.5f);
        mOrientationFusionComplimentary.reset();

        mOrientationFusionKalman = new OrientationKalmanFusion();
        mOrientationFusionKalman.setTimeConstant(0.5f);
        mOrientationFusionKalman.reset();

        mLinearAccelerationFilterComplimentary = new LinearAccelerationFusion(mOrientationFusionComplimentary);
        mLinearAccelerationFilterComplimentary.setTimeConstant(0.5f);
        mLinearAccelerationFilterComplimentary.reset();

        mLinearAccelerationFilterKalman = new LinearAccelerationFusion(mOrientationFusionKalman);
        mLinearAccelerationFilterKalman.setTimeConstant(0.5f);
        mLinearAccelerationFilterKalman.reset();

        mCustomOrientationFusionComplimentary = new CustomOrientationComplimentaryFusion();
        mCustomOrientationFusionComplimentary.setTimeConstant(0.5f);
        mCustomOrientationFusionComplimentary.reset();
    }

    private void registerPositionSensorListener() {

        /* Gravity sensor */
        Sensor SensorGravity = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        if (SensorGravity != null) {
            mSensorManager.registerListener(mPositionSensorListener, SensorGravity, sensorSpeed);
            mGravityAccuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH;
        } else {
            mGravityAccuracy = SENSOR_UNAVAILABLE;
        }

        /* Accelerometer sensor */
        Sensor SensorMagnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (SensorMagnetometer != null) {
            mSensorManager.registerListener(mPositionSensorListener, SensorMagnetometer, sensorSpeed);
        }

        /* Linear Acceleration */
        Sensor SensorLinearAcceleration = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        if (SensorLinearAcceleration != null) {
            mSensorManager.registerListener(mPositionSensorListener, SensorLinearAcceleration, sensorSpeed);
        }

        /* Magnetic field sensor */
        Sensor SensorMagField = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        if (SensorMagField != null) {
            mSensorManager.registerListener(mPositionSensorListener, SensorMagField, sensorSpeed);
            mMagneticFieldAccuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH;
        } else {
            mMagneticFieldAccuracy = SENSOR_UNAVAILABLE;
        }

        /* Gyroscope sensor */
        Sensor SensorGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        if (SensorGyroscope != null) {
            mSensorManager.registerListener(mPositionSensorListener, SensorGyroscope, sensorSpeed);
        }

        /* Step counter sensor */
        Sensor SensorStepCounter = mSensorManager.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR);
        if (SensorMagField != null) {
            mSensorManager.registerListener(mPositionSensorListener, SensorStepCounter, sensorSpeed, 0);
        }
    }

    private class PositionSensorListener implements SensorEventListener {

        @Override
        public void onSensorChanged(SensorEvent event) {
            switch (event.sensor.getType()) {
                case Sensor.TYPE_GRAVITY:
                    System.arraycopy(event.values, 0, mNormGravityVector,
                            0, mNormGravityVector.length);
                    processGravity();
                    break;
                case Sensor.TYPE_ACCELEROMETER:
                    float[] acceleration = new float[3];
                    System.arraycopy(event.values, 0, acceleration, 0, event.values.length);
                    processAcceleration(acceleration);
                    break;
                case Sensor.TYPE_MAGNETIC_FIELD:
                    System.arraycopy(event.values, 0, mMagnetometerReading,
                            0, mMagnetometerReading.length);
                    System.arraycopy(event.values, 0, mNormMagneticFieldValues,
                            0, mNormMagneticFieldValues.length);
                    processMagnetometer();
                    break;
                case Sensor.TYPE_GYROSCOPE:
                    System.arraycopy(event.values, 0, mRotation,
                            0, event.values.length);
                    processGyroscope();
                    break;
                case Sensor.TYPE_STEP_DETECTOR:
                    processStepDetector(event.values.length);
                    break;
                default:
                    break;
            }

            computeCompass();
            computeGlobalRotationAcceleration();
            computeXYSquareWave();
            setValue(output);
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
            switch(sensor.getType()) {
                case Sensor.TYPE_GRAVITY: mGravityAccuracy = accuracy; break;
                case Sensor.TYPE_MAGNETIC_FIELD: mMagneticFieldAccuracy = accuracy; break;
            }
        }
    }

    private void processGravity() {
        mNormGravity = (float)Math.sqrt(mNormGravityVector[0] * mNormGravityVector[0] +
                mNormGravityVector[1] * mNormGravityVector[1] +
                mNormGravityVector[2] * mNormGravityVector[2]);

        for(int i=0; i < mNormGravityVector.length; i++) {
            mNormGravityVector[i] /= mNormGravity;
        }
    }

    private void processAcceleration(float[] acceleration) {
        mOrientationFusionComplimentary.setAcceleration(acceleration);
        if (mLinearAccelerationFilterComplimentary.filter(acceleration) != null) {
            mAccelerometerReading = mLpfAccelerationSmoothing.filter(mLinearAccelerationFilterComplimentary.filter(acceleration));
        }

        mCustomOrientationFusionComplimentary.setAcceleration(acceleration);

        output[0] = mAccelerometerReading[0];
        output[2] = mAccelerometerReading[1];
    }

    private void processMagnetometer() {
        mCustomOrientationFusionComplimentary.setMagneticField(mMagnetometerReading);

        mOrientationFusionComplimentary.setMagneticField(mNormMagneticFieldValues);

        mNormMagneticField = (float)Math.sqrt(mNormMagneticFieldValues[0] * mNormMagneticFieldValues[0] +
                mNormMagneticFieldValues[1] * mNormMagneticFieldValues[1] +
                mNormMagneticFieldValues[2] * mNormMagneticFieldValues[2]);

        for(int i=0; i < mNormMagneticFieldValues.length; i++) {
            mNormMagneticFieldValues[i] /= mNormMagneticField;
        }
    }

    private void processGyroscope() {
        mRotationMatrix = mCustomOrientationFusionComplimentary.filter(mRotation);
        mFusedOrientation = mOrientationFusionComplimentary.filter(mRotation);

        output[5] = mFusedOrientation[1];
    }

    private void processStepDetector(int steps) {
        mSteps += steps;

        mCurrentpos[0] += Math.sin(mAzimuthRadians);
        mCurrentpos[1] += Math.cos(mAzimuthRadians);

        //output[5] = mSteps;
        output[6] = mCurrentpos[0];
        output[7] = mCurrentpos[1];
    }

    private void computeXYSquareWave() {
        int[] returnValue = new int[2];

        if (xAcceleration.size() != RMS_WINDOW) {
            xAcceleration.add(mAccelerometerReading[0]);
        } else {
            xAcceleration.remove();
            xAcceleration.add(mAccelerometerReading[0]);
            returnValue = getSquareWaveValue(xAcceleration, X_CURRENT_SIGN, X_RMS_THRESHOLD);
            output[1] = returnValue[0];
            X_CURRENT_SIGN = returnValue[1];
        }
        if (yAcceleration.size() != RMS_WINDOW) {
            yAcceleration.add(mAccelerometerReading[1]);
        } else {
            yAcceleration.remove();
            yAcceleration.add(mAccelerometerReading[1]);
            returnValue = getSquareWaveValue(yAcceleration, Y_CURRENT_SIGN, Y_RMS_THRESHOLD);
            output[3] = returnValue[0];
            Y_CURRENT_SIGN = returnValue[1];
        }
    }

    private int[] getSquareWaveValue(Queue<Float> queue, int CURRENT_SIGN, float threshold) {
        double MS = 0;
        double M = 0;
        for (Float i : queue) {
            MS = MS + Math.pow(i, 2);
            M = M + i;
        }
        MS = MS / RMS_WINDOW;
        M = M / RMS_WINDOW;
        double RMS = Math.sqrt(MS);

        if (RMS > threshold) {
            if (CURRENT_SIGN == 0) {
                if (M < 0) {
                    return new int[] {-3, 1};
                } else if (M > 0) {
                    return new int[] {3, 2};
                }
            } else {
                if (M > 0 && CURRENT_SIGN == 1) {
                    return new int[] {-3, 1};
                } else if (M < 0 && CURRENT_SIGN == 2) {
                    return new int[] {3, 2};
                } else if (M < 0 && CURRENT_SIGN == 1) {
                    return new int[] {-3, 1};
                } else if (M > 0 && CURRENT_SIGN == 2) {
                    return new int[] {3, 2};
                }
            }
        } else {
            return new int[] {0, 0};
        }
        return new int[] {0, 0};
    }

    private void computeCompass() {
        if (mNormGravityVector != null && mNormMagneticFieldValues != null) {
            // first calculate the horizontal vector that points due east
            float East_x = mNormMagneticFieldValues[1] * mNormGravityVector[2] -
                    mNormMagneticFieldValues[2] * mNormGravityVector[1];
            float East_y = mNormMagneticFieldValues[2] * mNormGravityVector[0] -
                    mNormMagneticFieldValues[0] * mNormGravityVector[2];
            float East_z = mNormMagneticFieldValues[0] * mNormGravityVector[1] -
                    mNormMagneticFieldValues[1] * mNormGravityVector[0];
            float norm_East = (float)Math.sqrt(East_x * East_x + East_y * East_y + East_z * East_z);

            if (mNormGravity * mNormMagneticField * norm_East < 0.1f) {  // Typical values are  > 100.
                mOrientationOK = false; // device is close to free fall (or in space?), or close to magnetic north pole.
            } else {
                mNormEastVector[0] = East_x / norm_East;
                mNormEastVector[1] = East_y / norm_East;
                mNormEastVector[2] = East_z / norm_East;

                // next calculate the horizontal vector that points due north
                float M_dot_G = (mNormGravityVector[0] * mNormMagneticFieldValues[0] +
                        mNormGravityVector[1] * mNormMagneticFieldValues[1] +
                        mNormGravityVector[2] * mNormMagneticFieldValues[2]);
                float North_x = mNormMagneticFieldValues[0] - mNormGravityVector[0] * M_dot_G;
                float North_y = mNormMagneticFieldValues[1] - mNormGravityVector[1] * M_dot_G;
                float North_z = mNormMagneticFieldValues[2] - mNormGravityVector[2] * M_dot_G;
                float norm_North = (float)Math.sqrt(North_x * North_x + North_y * North_y + North_z * North_z);
                mNormNorthVector[0] = North_x / norm_North;
                mNormNorthVector[1] = North_y / norm_North;
                mNormNorthVector[2] = North_z / norm_North;

                // take account of screen rotation away from its natural rotation
                /*int rotation = m_activity.getWindowManager().getDefaultDisplay().getRotation();
                float screen_adjustment = 0;
                switch(rotation) {
                    case Surface.ROTATION_0:   screen_adjustment =          0;         break;
                    case Surface.ROTATION_90:  screen_adjustment =   (float)Math.PI/2; break;
                    case Surface.ROTATION_180: screen_adjustment =   (float)Math.PI;   break;
                    case Surface.ROTATION_270: screen_adjustment = 3*(float)Math.PI/2; break;
                }*/
                // NB: the rotation matrix has now effectively been calculated. It consists of the three vectors m_NormEastVector[], m_NormNorthVector[] and m_NormGravityVector[]

                float screen_adjustment = 0;

                // calculate all the required angles from the rotation matrix
                // NB: see https://math.stackexchange.com/questions/381649/whats-the-best-3d-angular-co-ordinate-system-for-working-with-smartfone-apps
                float sin = mNormEastVector[1] -  mNormNorthVector[0], cos = mNormEastVector[0] +  mNormNorthVector[1];
                mAzimuthRadians = (float) (sin != 0 && cos != 0 ? Math.atan2(sin, cos) : 0);
                mPitchRadians = (float) Math.acos(mNormGravityVector[2]);
                sin = -mNormEastVector[1] -  mNormNorthVector[0];
                cos = mNormEastVector[0] -  mNormNorthVector[1];
                float aximuth_plus_two_pitch_axis_radians = (float)(sin != 0 && cos != 0 ? Math.atan2(sin, cos) : 0);
                mPitchAxisRadians = (float)(aximuth_plus_two_pitch_axis_radians - mAzimuthRadians) / 2;
                mAzimuthRadians += screen_adjustment;
                mPitchAxisRadians += screen_adjustment;
                mOrientationOK = true;

                if (Math.abs((mAzimuthRadians * (180 / Math.PI)) - (mAzimuthRadiansPrev * (180 / Math.PI))) > 1.0) {
                    // TODO: update compass reading
                }
                //Log.d("ORIENTATION", Double.toString(m_azimuth_radians * (180 / Math.PI)));
                output[4] = mAzimuthRadians * (180 / Math.PI);
            }
        }
    }

    private void computeGlobalRotationAcceleration() {
        float[] rotationMatrix = new float[16];
        System.arraycopy(mRotationMatrix, 0, rotationMatrix, 0, mRotationMatrix.length);
        Log.d("ORIENTATION", "rotationMatrix:" + Arrays.toString(rotationMatrix));

        float[] inverseRotationMatrix = new float[16];
        android.opengl.Matrix.invertM(inverseRotationMatrix, 0, rotationMatrix, 0);
        Log.d("ORIENTATION", "inverseRotationMatrix:" + Arrays.toString(inverseRotationMatrix));

        float[] acceleration = new float[4];
        acceleration[0] = mAccelerometerReading[0]; acceleration[1] = mAccelerometerReading[1]; acceleration[2] = mAccelerometerReading[2];
        acceleration[3] = 0;
        Log.d("ORIENTATION", "acceleration:" + Arrays.toString(acceleration));

        float[] globalAcceleration = new float[16];
        android.opengl.Matrix.multiplyMV(globalAcceleration, 0, inverseRotationMatrix, 0, acceleration, 0);
        Log.d("ORIENTATION", "globalAcceleration:" + Arrays.toString(globalAcceleration));

        output[8] = globalAcceleration[0];
        output[10] = globalAcceleration[1];

        int[] returnValue = new int[2];

        if (eastAcceleration.size() != RMS_WINDOW) {
            eastAcceleration.add((float)output[8]);
        } else {
            eastAcceleration.remove();
            eastAcceleration.add((float)output[8]);
            returnValue = getSquareWaveValue(eastAcceleration, EAST_CURRENT_SIGN, X_RMS_THRESHOLD);
            output[9] = returnValue[0];
            EAST_CURRENT_SIGN = returnValue[1];
        }
        if (northAcceleration.size() != RMS_WINDOW) {
            northAcceleration.add((float)output[9]);
        } else {
            northAcceleration.remove();
            northAcceleration.add((float)output[10]);
            returnValue = getSquareWaveValue(northAcceleration, NORTH_CURRENT_SIGN, Y_RMS_THRESHOLD);
            output[11] = returnValue[0];
            NORTH_CURRENT_SIGN = returnValue[1];
        }

    }
    /*
    @Override
    public void onSensorChanged(SensorEvent evnt) {
        int SensorType = evnt.sensor.getType();
        switch(SensorType) {
            case Sensor.TYPE_GRAVITY:
                if (m_NormGravityVector == null) m_NormGravityVector = new float[3];
                System.arraycopy(evnt.values, 0, m_NormGravityVector, 0, m_NormGravityVector.length);
                m_Norm_Gravity = (float)Math.sqrt(m_NormGravityVector[0]*m_NormGravityVector[0] + m_NormGravityVector[1]*m_NormGravityVector[1] + m_NormGravityVector[2]*m_NormGravityVector[2]);
                for(int i=0; i < m_NormGravityVector.length; i++) m_NormGravityVector[i] /= m_Norm_Gravity;
                break;
            case Sensor.TYPE_ACCELEROMETER:
                float[] acceleration = new float[3];
                System.arraycopy(evnt.values, 0, acceleration, 0, evnt.values.length);
                mOrientationFusionComplimentary.setAcceleration(acceleration);
                if (mLinearAccelerationFilterComplimentary.filter(acceleration) != null) {
                    mAccelerometerReading = mLpfAccelerationSmoothing.filter(mLinearAccelerationFilterComplimentary.filter(acceleration));
                }

                mCustomOrientationFusionComplimentary.setAcceleration(acceleration);
                m_orientationSensorCallback.onAccelerationUpdate(mAccelerometerReading);
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                System.arraycopy(evnt.values, 0, mMagnetometerReading, 0, mMagnetometerReading.length);
                mCustomOrientationFusionComplimentary.setMagneticField(mMagnetometerReading);

                if (m_NormMagFieldValues == null) m_NormMagFieldValues = new float[3];
                System.arraycopy(evnt.values, 0, m_NormMagFieldValues, 0, m_NormMagFieldValues.length);
                mOrientationFusionComplimentary.setMagneticField(m_NormMagFieldValues);


                m_Norm_MagField = (float)Math.sqrt(m_NormMagFieldValues[0]*m_NormMagFieldValues[0] + m_NormMagFieldValues[1]*m_NormMagFieldValues[1] + m_NormMagFieldValues[2]*m_NormMagFieldValues[2]);
                for(int i=0; i < m_NormMagFieldValues.length; i++) m_NormMagFieldValues[i] /= m_Norm_MagField;
                break;
            case Sensor.TYPE_GYROSCOPE:
                System.arraycopy(evnt.values, 0, mRotation, 0, evnt.values.length);

                mRotationMatrix = mCustomOrientationFusionComplimentary.filter(this.mRotation);
                mFusedOrientation = mOrientationFusionComplimentary.filter(this.mRotation);
                break;
            case Sensor.TYPE_STEP_DETECTOR:
                mSteps+= evnt.values.length;

                m_current_pos[0] += Math.sin(m_azimuth_radians);
                m_current_pos[1] += Math.cos(m_azimuth_radians);

                Log.i("STEPCOUNTER", "New step detected by STEP_COUNTER sensor. Total step count: " +
                        mSteps + ", orientation:" + Double.toString(m_azimuth_radians * (180 / Math.PI)) +
                        ", current_pos" + Arrays.toString(m_current_pos));

                m_orientationSensorCallback.onPositionUpdate(m_current_pos);
                m_orientationSensorCallback.onStepUpdate(mSteps);
        }
        if (m_NormGravityVector != null && m_NormMagFieldValues != null) {
            // first calculate the horizontal vector that points due east
            float East_x = m_NormMagFieldValues[1]*m_NormGravityVector[2] - m_NormMagFieldValues[2]*m_NormGravityVector[1];
            float East_y = m_NormMagFieldValues[2]*m_NormGravityVector[0] - m_NormMagFieldValues[0]*m_NormGravityVector[2];
            float East_z = m_NormMagFieldValues[0]*m_NormGravityVector[1] - m_NormMagFieldValues[1]*m_NormGravityVector[0];
            float norm_East = (float)Math.sqrt(East_x * East_x + East_y * East_y + East_z * East_z);
            if (m_Norm_Gravity * m_Norm_MagField * norm_East < 0.1f) {  // Typical values are  > 100.
                m_OrientationOK = false; // device is close to free fall (or in space?), or close to magnetic north pole.
            } else {
                m_NormEastVector[0] = East_x / norm_East; m_NormEastVector[1] = East_y / norm_East; m_NormEastVector[2] = East_z / norm_East;

                // next calculate the horizontal vector that points due north
                float M_dot_G = (m_NormGravityVector[0] *m_NormMagFieldValues[0] + m_NormGravityVector[1]*m_NormMagFieldValues[1] + m_NormGravityVector[2]*m_NormMagFieldValues[2]);
                float North_x = m_NormMagFieldValues[0] - m_NormGravityVector[0] * M_dot_G;
                float North_y = m_NormMagFieldValues[1] - m_NormGravityVector[1] * M_dot_G;
                float North_z = m_NormMagFieldValues[2] - m_NormGravityVector[2] * M_dot_G;
                float norm_North = (float)Math.sqrt(North_x * North_x + North_y * North_y + North_z * North_z);
                m_NormNorthVector[0] = North_x / norm_North; m_NormNorthVector[1] = North_y / norm_North; m_NormNorthVector[2] = North_z / norm_North;

                // take account of screen rotation away from its natural rotation
                int rotation = m_activity.getWindowManager().getDefaultDisplay().getRotation();
                float screen_adjustment = 0;
                switch(rotation) {
                    case Surface.ROTATION_0:   screen_adjustment =          0;         break;
                    case Surface.ROTATION_90:  screen_adjustment =   (float)Math.PI/2; break;
                    case Surface.ROTATION_180: screen_adjustment =   (float)Math.PI;   break;
                    case Surface.ROTATION_270: screen_adjustment = 3*(float)Math.PI/2; break;
                }
                // NB: the rotation matrix has now effectively been calculated. It consists of the three vectors m_NormEastVector[], m_NormNorthVector[] and m_NormGravityVector[]

                // calculate all the required angles from the rotation matrix
                // NB: see https://math.stackexchange.com/questions/381649/whats-the-best-3d-angular-co-ordinate-system-for-working-with-smartfone-apps
                float sin = m_NormEastVector[1] -  m_NormNorthVector[0], cos = m_NormEastVector[0] +  m_NormNorthVector[1];
                m_azimuth_radians = (float) (sin != 0 && cos != 0 ? Math.atan2(sin, cos) : 0);
                m_pitch_radians = (float) Math.acos(m_NormGravityVector[2]);
                sin = -m_NormEastVector[1] -  m_NormNorthVector[0]; cos = m_NormEastVector[0] -  m_NormNorthVector[1];
                float aximuth_plus_two_pitch_axis_radians = (float)(sin != 0 && cos != 0 ? Math.atan2(sin, cos) : 0);
                m_pitch_axis_radians = (float)(aximuth_plus_two_pitch_axis_radians - m_azimuth_radians) / 2;
                m_azimuth_radians += screen_adjustment;
                m_pitch_axis_radians += screen_adjustment;
                m_OrientationOK = true;

                if (Math.abs((m_azimuth_radians * (180 / Math.PI)) - (m_azimuth_radians_prev * (180 / Math.PI))) > 1.0) {
                    m_orientationSensorCallback.onOrientationUpdate(m_azimuth_radians);
                }
                //Log.d("ORIENTATION", Double.toString(m_azimuth_radians * (180 / Math.PI)));
            }
        }
        float[] rotationMatrix = new float[16];
        System.arraycopy(mRotationMatrix, 0, rotationMatrix, 0, mRotationMatrix.length);
        //Log.d("ORIENTATION", "rotationMatrix:" + Arrays.toString(rotationMatrix));

        float[] inverseRotationMatrix = new float[16];
        android.opengl.Matrix.invertM(inverseRotationMatrix, 0, rotationMatrix, 0);
        //Log.d("ORIENTATION", "inverseRotationMatrix:" + Arrays.toString(inverseRotationMatrix));

        float[] acceleration = new float[4];
        acceleration[0] = mAccelerometerReading[0]; acceleration[1] = mAccelerometerReading[1]; acceleration[2] = mAccelerometerReading[2];
        acceleration[3] = 0;
        //Log.d("ORIENTATION", "acceleration:" + Arrays.toString(acceleration));

        float[] globalAcceleration = new float[16];
        android.opengl.Matrix.multiplyMV(globalAcceleration, 0, inverseRotationMatrix, 0, acceleration, 0);
        //Log.d("ORIENTATION", "globalAcceleration:" + Arrays.toString(globalAcceleration));
        m_orientationSensorCallback.onGlobalAccelerationUpdate(globalAcceleration);
    }*/

    abstract static class OrientationSensorCallback {
        abstract public void onStepUpdate(int noSteps);
        abstract public void onPositionUpdate(double[] position);
        abstract public void onAccelerationUpdate(float[] acceleration);
        abstract public void onGlobalAccelerationUpdate(float[] acceleration);
        abstract public void onOrientationUpdate(double orientation);
    }
}