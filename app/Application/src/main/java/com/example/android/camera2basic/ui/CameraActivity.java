/*
 * Copyright 2017 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.android.camera2basic.ui;

import androidx.lifecycle.Observer;
import android.content.Intent;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.TextView;

import com.example.android.camera2basic.livedata.PositionSensorLiveData;
import com.example.android.camera2basic.R;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.lang.ref.WeakReference;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Locale;
import java.util.Scanner;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;


public class CameraActivity extends AppCompatActivity {

    private OutThread mOutThread;

    private SensorManager mSensorManager;

    private PositionSensorLiveData mPositionSensorLiveData;

    private ArrayList<double[]> userPathList = new ArrayList<>();


    private double mEast = 0;
    private String mX = "0";
    private double mNorth = 0;
    private String mY = "0";
    private double mOrientationDegrees = 0;
    private String mPitch = "0";
    private int mSteps = 0;

    private int mPollingPeriod = 100;
    private float mEventThreshold = 0.3f;
    private float mXRMSThreshold = 1.0f;
    private float mYRMSThreshold = 1.0f;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);
        if (null == savedInstanceState) {
            getSupportFragmentManager().beginTransaction()
                    .replace(R.id.fragment_camera2_container, Camera2BasicFragment.newInstance())
                    .commit();
        }

        initTCPServerConnection();
    }

    public void onChartClick(View view) {
        Intent intent = new Intent(this, ChartActivity.class);
        startActivity(intent);
    }

    public void startPositionSensorLiveDataObserve() {
        PositionSensorLiveData mPositionSensorLiveData = PositionSensorLiveData.get(getApplicationContext());
        mPositionSensorLiveData.setParams(mEventThreshold, mXRMSThreshold, mYRMSThreshold);
        mPositionSensorLiveData.observe(this, new Observer<double[]>() {
            @Override
            public void onChanged(@Nullable double[] doubles) {
                mEast = doubles[1];
                mNorth = doubles[3];
                mX = String.format(Locale.getDefault(), "%.2f", doubles[0]);
                mY = String.format(Locale.getDefault(), "%.2f", doubles[2]);

                ((TextView)(findViewById(R.id.accelerationTextView)))
                        .setText("x:" + mX + ", y:" + mY);

                mOrientationDegrees = doubles[4];

                ((TextView)(findViewById(R.id.compassTextView)))
                        .setText(Integer.toString((int)mOrientationDegrees));

                mPitch = String.format(Locale.getDefault(), "%.2f", doubles[5]);;
                ((TextView)(findViewById(R.id.stepCountTextView)))
                        .setText(mPitch + " pitch");

                double[] position = new double[2];
                position[0] = doubles[6];
                position[1] = doubles[7];

                ((UserPathView)(findViewById(R.id.user_path_view)))
                        .updateUserPathList(position, (int)mOrientationDegrees, 0);
            }
        });
    }

    private void initTCPServerConnection() {
        final Runnable orientationRunnable = new Runnable() {
            @Override
            public void run() {
                if (mOutThread.mHandler != null) {
                    Message msg = new Message();

                    String front_back = "0";
                    if (mNorth > 0) {
                        front_back = "1";
                    } else if (mNorth < 0) {
                        front_back = "-1";
                    }

                    String left_right = "0";
                    if (mEast > 0) {
                        left_right = "1";
                    } else if (mEast < 0) {
                        left_right = "-1";
                    }

                    msg.obj = "[" + "xraw:" + mX + ", yraw:" + mY +
                            ", xsqr:" + left_right + ", ysqr:" + front_back +
                            ", compass:" + Integer.toString((int)mOrientationDegrees) + ", pitch:" + mPitch + "]";
                    mOutThread.mHandler.sendMessage(msg);
                }
            }
        };

        Runnable tcpServerRunnable = new Runnable() {
            @Override
            public void run() {
                try {
                    ServerSocket serverSocket = new ServerSocket(6666);

                    while (true) {
                        Log.d("TCPSERVER", "waiting for connection");
                        Socket clientSocket = serverSocket.accept();
                        Log.d("TCPSERVER", "connected!");
                        final PrintWriter out =
                                new PrintWriter(clientSocket.getOutputStream(), true);
                        BufferedReader in = new BufferedReader(
                                new InputStreamReader(clientSocket.getInputStream()));

                        String inputLine, outputLine;

                        while ((inputLine = in.readLine()) != null) {
                            Log.d("TCPSERVER", inputLine);
                            String [] input = inputLine.split(" ");
                            if (input[0].equals("start")) {
                                mOutThread = new OutThread(out);
                                mOutThread.start();

                                int i = 0;
                                for (String arg : input) {
                                    Log.d("TCPSERVER", "arg " + arg);
                                    if (arg.equals("-p") && i+1 <= input.length - 1) {
                                        if (isInteger(input[i + 1], 10) ) {
                                            mPollingPeriod = Integer.parseInt(input[i + 1]);
                                        }
                                    }
                                    if (arg.equals("-e") && i+1 <= input.length - 1) {
                                        mEventThreshold = Float.parseFloat(input[i + 1]);
                                    }
                                    if (arg.equals("-xt") && i+1 <= input.length - 1) {
                                        mXRMSThreshold = Float.parseFloat(input[i + 1]);
                                    }
                                    if (arg.equals("-yt") && i+1 <= input.length - 1) {
                                        mYRMSThreshold = Float.parseFloat(input[i + 1]);
                                    }
                                    i++;
                                }

                                startPositionSensorLiveDataObserve();
                                ScheduledExecutorService scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();
                                ScheduledFuture<?> scheduledFuture = scheduledExecutorService
                                        .scheduleAtFixedRate(orientationRunnable, 1000, mPollingPeriod, TimeUnit.MILLISECONDS);

                            }
                        }
                        Log.d("TCPSERVER", "closed connection");
                        mOutThread.interrupt();
                        mOutThread.mHandler.getLooper().quit();
                        clientSocket.shutdownInput();
                        clientSocket.shutdownOutput();
                        clientSocket.close();
                    }
                } catch (Exception e) {
                    ;
                }
            }
        };

        ExecutorService executorService = Executors.newSingleThreadExecutor();
        executorService.execute(tcpServerRunnable);
    }

    public static boolean isInteger(String s, int radix) {
        Scanner sc = new Scanner(s.trim());
        if(!sc.hasNextInt(radix)) return false;
        sc.nextInt(radix);
        return !sc.hasNext();
    }

    private static class OutThread extends Thread {

        static class handler extends Handler {
            private final WeakReference<OutThread> mThread;
            private PrintWriter mPrintWriter;
            handler(OutThread thread, PrintWriter printWriter) {
                mThread = new WeakReference<OutThread>(thread);
                mPrintWriter = printWriter;
            }

            @Override
            public void handleMessage(Message msg) {
                Log.d("TCPSERVER", "receiving message " + msg.what);

                if (Thread.interrupted()) {
                    Log.d("TCPSERVER", "thread stoped");
                    Thread.currentThread().interrupt();
                    this.removeCallbacksAndMessages(null);
                    return;
                }
                String out = (String) msg.obj;
                mPrintWriter.println(out);
            }
        }
        public handler mHandler;

        private PrintWriter mPrintWriter;
        OutThread(PrintWriter in) {
            mPrintWriter = in;
            Log.d("TCPSERVER", "thread created");
        }

        @Override
        public void run() {
            Looper.prepare();
            mHandler = new handler(this, mPrintWriter);
            Looper.loop();
        }
    }

    @Override
    public void onPause() {
        super.onPause();
    }
}
