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
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import java.text.SimpleDateFormat;
import java.util.Date;

import static android.content.ContentValues.TAG;

public class SimplePublisherNode extends AbstractNodeMain implements NodeMain {

    private static final String TAG = SimplePublisherNode.class.getSimpleName();
    private Publisher<std_msgs.Float64MultiArray> publisher_GPS;
    private Publisher<std_msgs.Float32MultiArray> publisher_IMU;

    public double[] GPS_arr = {19,71};
    public float[] Orientation_arr={0,0,0};
    private ConnectedNode cNode;
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("SimplePublisher/TimeLoopNode");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        publisher_GPS = connectedNode.newPublisher(GraphName.of("LatLon"), std_msgs.Float64MultiArray._TYPE);
        publisher_IMU = connectedNode.newPublisher(GraphName.of("IMU"), std_msgs.Float32MultiArray._TYPE);

        cNode=connectedNode;

        final CancellableLoop loop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                // retrieve current system time
                //String time = new SimpleDateFormat("HH:mm:ss").format(new Date());

                //Log.i(TAG, "publishing the current time: " + time);

                // create and publish a simple string message
                std_msgs.Float64MultiArray GPS_msg = publisher_GPS.newMessage();
                std_msgs.Float32MultiArray IMU_msg = publisher_IMU.newMessage();
                GPS_msg.setData(GPS_arr);
                IMU_msg.setData(Orientation_arr);
                //Log.i(TAG,Double.toString(longitude)+Double.toString(latitude));
                publisher_GPS.publish(GPS_msg);
                publisher_IMU.publish(IMU_msg);
                // go to sleep for one second
                Thread.sleep(5);
            }
        };
        connectedNode.executeCancellableLoop(loop);
    }
    /*
    public void publishMessage(String mssg){
        final String pub_msg=mssg;
        final CancellableLoop loop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                // retrieve current system time
                //String time = new SimpleDateFormat("HH:mm:ss").format(new Date());

                //Log.i(TAG, "publishing the current time: " + time);

                // create and publish a simple string message
                std_msgs.String str = publisher.newMessage();
                str.setData(pub_msg);
                Log.i(TAG,longitude+latitude);
                publisher.publish(str);

            }
        };
        cNode.executeCancellableLoop(loop);
        try {
            Thread.sleep(100);
        }
        catch (InterruptedException ie){

        }
        loop.cancel();
    }
    */

}
