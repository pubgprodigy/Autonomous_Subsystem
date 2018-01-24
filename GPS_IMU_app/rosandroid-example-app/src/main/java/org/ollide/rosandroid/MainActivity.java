/*
 * Copyright (C) 2014 Oliver Degener.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ollide.rosandroid;

import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Address;
import android.location.Geocoder;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;
import java.util.List;
import java.util.Locale;

import static android.content.ContentValues.TAG;
import static java.util.Objects.isNull;

public class MainActivity extends RosActivity implements LocationListener, SensorEventListener {

    public MainActivity() {
        super("RosAndroidExample", "RosAndroidExample");
    }
    private SimplePublisherNode node_sp;
    private static final int PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION = 1;
    private boolean mLocationPermissionGranted;
    private final float[] mAccelerometerReading = new float[3];
    private final float[] mMagnetometerReading = new float[3];
    private Sensor mAccelerometer;
    private Sensor mMagnetometer;
    private SensorManager mSensorManager;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        if (!(ContextCompat.checkSelfPermission(this.getApplicationContext(),
                android.Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED)) {
            Log.i(TAG,"Permissions Not Granted");
            final TextView helloTextView = findViewById(R.id.textView3);
            helloTextView.setText("Permissions Not Granted");

            getLocationPermission();

        }
        LocationManager locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, this);

        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        mSensorManager.registerListener(this, mAccelerometer,
                SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mMagnetometer,
                SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);

        mAccelerometer =   mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mMagnetometer =   mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

    }

    @Override
    protected void onResume() {
        super.onResume();

        // Get updates from the accelerometer and magnetometer at a constant rate.
        // To make batch operations more efficient and reduce power consumption,
        // provide support for delaying updates to the application.
        //
        // In this example, the sensor reporting delay is small enough such that
        // the application receives an update before the system checks the sensor
        // readings again.
        mSensorManager.registerListener(this, mAccelerometer,
                SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mMagnetometer,
                SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);

    }

    @Override
    protected void onPause() {
        super.onPause();

        // Don't receive any more updates from either sensor.
        mSensorManager.unregisterListener(this);
    }


    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        node_sp = new SimplePublisherNode();


        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getROS_IP());

        //nodeConfiguration.setTcpRosBindAddress()

        nodeConfiguration.setMasterUri(getMasterUri());

        nodeMainExecutor.execute(node_sp, nodeConfiguration);
    }

    @Override
    public void onLocationChanged(Location location) {
        // TODO Auto-generated method stub

        double latitude =  location.getLatitude();
        double longitude = location.getLongitude();

        Log.i("Geo_Location", "Latitude: " + latitude + ", Longitude: " + longitude);
        final TextView helloTextView = findViewById(R.id.textView2);
        helloTextView.setText("Latitude: " + latitude + ", Longitude: " + longitude);
        String msg="Latitude: " + latitude + ", Longitude: " + longitude;
        node_sp.GPS_arr[0]=latitude;
        node_sp.GPS_arr[1]=longitude;
    }

    @Override
    public void onProviderDisabled(String provider) {
        // TODO Auto-generated method stub

    }

    @Override
    public void onProviderEnabled(String provider) {
        // TODO Auto-generated method stub

    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {
        // TODO Auto-generated method stub

    }
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Do something here if sensor accuracy changes.
        // You must implement this callback in your code.
    }

    private void getLocationPermission() {
        /*
         * Request location permission, so that we can get the location of the
         * device. The result of the permission request is handled by a callback,
         * onRequestPermissionsResult.
         */
        if (ContextCompat.checkSelfPermission(this.getApplicationContext(),
                android.Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED) {
            mLocationPermissionGranted = true;
        } else {
            ActivityCompat.requestPermissions(this,
                    new String[]{android.Manifest.permission.ACCESS_FINE_LOCATION},
                    PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION);
        }
    }

    // Get readings from accelerometer and magnetometer. To simplify calculations,
    // consider storing these readings as unit vectors.
    @Override
    public void onSensorChanged(SensorEvent event) {

        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, mAccelerometerReading,
                    0, mAccelerometerReading.length);
        }
        else if (event.sensor == mMagnetometer) {
            System.arraycopy(event.values, 0, mMagnetometerReading,
                    0, mMagnetometerReading.length);
        }

        // --------------------------------------------------------------------
        // Orientation Sensor
        // Rotation matrix based on current readings from accelerometer and magnetometer.
        float[] rotationMatrix = new float[9];
        mSensorManager.getRotationMatrix(rotationMatrix, null,
                mAccelerometerReading, mMagnetometerReading);

        // Express the updated rotation matrix as three orientation angles.
        float[] orientationAngles = {0,0,0};
        mSensorManager.getOrientation(rotationMatrix, orientationAngles);
        //CharSequence text = steps+":"+level+":"+(orientationAngles[0]*180/3.14)+":"+(orientationAngles[1]*180/3.14)+":"+(orientationAngles[2]*180/3.14);
        //final TextView helloTextView = (TextView) findViewById(R.id.textView_id);
        //helloTextView.setText(text);

        for (int i=0;i<3;i++){
            if(!isNull(node_sp))
            {
                node_sp.Orientation_arr[i]=orientationAngles[i];
            }
        }
        final TextView helloTextView = findViewById(R.id.textView3);
        helloTextView.setText(orientationAngles[0]*180/3.14 + ":" + orientationAngles[1]*180/3.14+":"+orientationAngles[2]*180/3.14);

        /*
        if(orientation_angles.size()<q_size){
            orientation_angles.add(curr_ang);
            sum_ang=sum_ang+curr_ang;
        }
        else{
            double remove_elem=orientation_angles.poll();
            sum_ang=sum_ang-remove_elem+curr_ang;
            orientation_angles.add(curr_ang);
        }*/
        //toast = Toast.makeText(context,text, Toast.LENGTH_LONG);
        // toast.show();
        // --------------------------------------------------------------------

    }
}
