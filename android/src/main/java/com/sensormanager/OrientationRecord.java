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
    private Sensor mGravityEvent;
    private Sensor mRotationEvent;
    private long lastUpdate = 0;
    private int i = 0, n = 0;
    private int delay;
    private int isRegistered = 0;

    private float[] mRotationMatrix = new float[9];
    private float[] mRotationResult = new float[3];

    private ReactContext mReactContext;
    private Arguments mArguments;


    public OrientationRecord(ReactApplicationContext reactContext) {
        mSensorManager = (SensorManager)reactContext.getSystemService(reactContext.SENSOR_SERVICE);
        mGravityEvent = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        mRotationEvent = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        mReactContext = reactContext;
    }

    public int start(int delay) {
        this.delay = delay;

        if (mRotationEvent != null && isRegistered == 0) {
            mSensorManager.registerListener(this, mGravityEvent, SensorManager.SENSOR_DELAY_UI);
            mSensorManager.registerListener(this, mRotationEvent, SensorManager.SENSOR_DELAY_GAME);
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
    float[] mRotationData;

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
      Sensor mySensor = sensorEvent.sensor;
      WritableMap map = mArguments.createMap();

      if (mySensor.getType() == Sensor.TYPE_GRAVITY) {
        mGravity = sensorEvent.values.clone();
      }

      if (mySensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
        mRotationData = sensorEvent.values.clone();
      }

      if (mRotationData != null) {
          long curTime = System.currentTimeMillis();

          mSensorManager.getRotationMatrixFromVector(mRotationMatrix, mRotationData);
          mSensorManager.getOrientation(mRotationMatrix, mRotationResult);
          map.putDouble("alpha", -mRotationResult[0]);
          map.putDouble("beta", -mRotationResult[1]);
          map.putDouble("gamma", mRotationResult[2]);

          if (mGravity != null) {
              map.putDouble("gravityX", mGravity[0]);
          }

          sendEvent("Orientation", map);
          lastUpdate = curTime;
        }
      }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
}
