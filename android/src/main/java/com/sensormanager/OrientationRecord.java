package com.sensormanager;

import android.os.Bundle;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;
import android.support.annotation.Nullable;

import java.io.*;
import java.util.Date;
import java.util.Timer;

import com.facebook.react.bridge.Arguments;
import com.facebook.react.bridge.ReactContext;
import com.facebook.react.bridge.WritableMap;
import com.facebook.react.modules.core.DeviceEventManagerModule;
import com.facebook.react.bridge.ReactApplicationContext;

public class OrientationRecord implements SensorEventListener {

    private SensorManager mSensorManager;
    private Sensor mAccelerometer;
    private Sensor mMagnetometer;
    private Sensor mRotationVectorSensor;
    private long lastUpdate = 0;
    private int i = 0, n = 0;
    private int delay;
    private int isRegistered = 0;

    private float grav[] = new float[3];
	  private float mag[] = new float[3];
    static final float ALPHA = 0.2f;

    private ReactContext mReactContext;
    private Arguments mArguments;


    public OrientationRecord(ReactApplicationContext reactContext) {
        mSensorManager = (SensorManager)reactContext.getSystemService(reactContext.SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mMagnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        mReactContext = reactContext;
    }

    public int start(int delay) {
        this.delay = delay;

        if (mAccelerometer != null && isRegistered == 0) {
            mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_UI);
            mSensorManager.registerListener(this, mMagnetometer, SensorManager.SENSOR_DELAY_UI);
            mSensorManager.registerListener(this, mRotationVectorSensor, SensorManager.SENSOR_DELAY_GAME);
            isRegistered = 1;
            return (1);
        }
        return (0);
    }

    public void stop() {
        if (isRegistered == 1) {
            mSensorManager.unregisterListener(this);
        isRegistered = 0;
      }
    }

    private void sendEvent(String eventName, @Nullable WritableMap params)
    {
        try {
            mReactContext
                .getJSModule(DeviceEventManagerModule.RCTDeviceEventEmitter.class)
                .emit(eventName, params);
        } catch (RuntimeException e) {
            Log.e("ERROR", "java.lang.RuntimeException: Trying to invoke JS before CatalystInstance has been set!");
        }
    }

    float[] mGravity;
    float[] mGeomagnetic;
    float mOrientationData[] = new float[3];

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
      Sensor mySensor = sensorEvent.sensor;
      WritableMap map = mArguments.createMap();

      if (mySensor.getType() == Sensor.TYPE_ACCELEROMETER) {
        mGravity = lowPass(sensorEvent.values.clone(), mGravity);
        grav[0] = sensorEvent.values[0];
        grav[1] = sensorEvent.values[1];
        grav[2] = sensorEvent.values[2];
      }

      if (mySensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
        mGeomagnetic = lowPass(sensorEvent.values.clone(), mGeomagnetic);
  			mag[0] = sensorEvent.values[0];
  			mag[1] = sensorEvent.values[1];
  			mag[2] = sensorEvent.values[2];
      }

      if (mySensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
              long timeNow            = System.nanoTime();
              calcOrientation(mOrientationData, sensorEvent.values.clone());
      }

      if (mGravity != null && mGeomagnetic != null) {
        float R[] = new float[9];
        float I[] = new float[9];
        boolean success = mSensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
        if (success) {
          long curTime = System.currentTimeMillis();
          float orientation[] = new float[3];
          mSensorManager.getOrientation(R, orientation);

          // float heading = (float)((Math.toDegrees(orientation[0])) % 360.0f);
          // float pitch = (float)((Math.toDegrees(orientation[1])) % 360.0f);
          // float roll = (float)((Math.toDegrees(orientation[2])) % 360.0f);

          float heading = (float)(mOrientationData[0]);
          float pitch = (float)(mOrientationData[1]);
          float roll = (float)(mOrientationData[2]);

          // if (heading < 0) {
          //   heading = 360 - (0 - heading);
          // }
          //
          // if (pitch < 0) {
          //   pitch = 360 - (0 - pitch);
          // }
          //
          // if (roll < 0) {
          //   roll = 360 - (0 - roll);
          // }

          map.putDouble("azimuth", heading);
          map.putDouble("pitch", pitch);
          map.putDouble("roll", roll);
          sendEvent("Orientation", map);
          lastUpdate = curTime;
        }
      }
    }

    protected float[] lowPass( float[] input, float[] output ) {
      if ( output == null ) return input;

      for ( int i=0; i<input.length; i++ ) {
        output[i] = output[i] + ALPHA * (input[i] - output[i]);
      }
      return output;
    }

    private void calcOrientation(float[] orientation, float[] incomingValues) {
        // Get the quaternion
        float[] quatF = new float[4];
        mSensorManager.getQuaternionFromVector(quatF, incomingValues);

        // Get the rotation matrix
        //
        // This is a variant on the code presented in
        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
        // which has been altered for scaling and (I think) a different axis arrangement. It
        // tells you the rotation required to get from the between the phone's axis
        // system and the earth's.
        //
        // Phone axis system:
        // https://developer.android.com/guide/topics/sensors/sensors_overview.html#sensors-coords
        //
        // Earth axis system:
        // https://developer.android.com/reference/android/hardware/SensorManager.html#getRotationMatrix(float[], float[], float[], float[])
        //
        // Background information:
        // https://en.wikipedia.org/wiki/Rotation_matrix
        //
        float[][] rotMatF = new float[3][3];
        rotMatF[0][0] = quatF[1]*quatF[1] + quatF[0]*quatF[0] - 0.5f;
        rotMatF[0][1] = quatF[1]*quatF[2] - quatF[3]*quatF[0];
        rotMatF[0][2] = quatF[1]*quatF[3] + quatF[2]*quatF[0];
        rotMatF[1][0] = quatF[1]*quatF[2] + quatF[3]*quatF[0];
        rotMatF[1][1] = quatF[2]*quatF[2] + quatF[0]*quatF[0] - 0.5f;
        rotMatF[1][2] = quatF[2]*quatF[3] - quatF[1]*quatF[0];
        rotMatF[2][0] = quatF[1]*quatF[3] - quatF[2]*quatF[0];
        rotMatF[2][1] = quatF[2]*quatF[3] + quatF[1]*quatF[0];
        rotMatF[2][2] = quatF[3]*quatF[3] + quatF[0]*quatF[0] - 0.5f;

        // Get the orientation of the phone from the rotation matrix
        //
        // There is some discussion of this at
        // http://stackoverflow.com/questions/30279065/how-to-get-the-euler-angles-from-the-rotation-vector-sensor-type-rotation-vecto
        // in particular equation 451.
        //
        final float rad2deg = (float)(180.0 / Math.PI);
        orientation[0] = (float)Math.atan2(-rotMatF[1][0], rotMatF[0][0]) * rad2deg;
        orientation[1] = (float)Math.atan2(-rotMatF[2][1], rotMatF[2][2]) * rad2deg;
        orientation[2] = (float)Math.asin ( rotMatF[2][0])                * rad2deg;
        if (orientation[0] < 0) orientation[0] += 360;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
}
