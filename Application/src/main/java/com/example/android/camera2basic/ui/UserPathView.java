package com.example.android.camera2basic.ui;

/**
 * Created by gio on 3/5/18.
 */

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.graphics.drawable.BitmapDrawable;
import android.support.annotation.Nullable;
import android.util.AttributeSet;
import android.util.Log;
import android.util.TypedValue;
import android.view.View;
import android.view.ViewGroup;
import android.widget.FrameLayout;

import com.example.android.camera2basic.R;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by gio on 3/1/18.
 */

public class UserPathView extends View {

    private Paint backgroundPaint;

    private Paint pathPaint;
    private Paint userCirclePaint;
    private List<double[]> userPathList = new ArrayList<double[]>();

    private RectF rimRect;

    private float x;
    private float y;
    public UserPathView(Context context) {
        super(context);
        init();
    }

    public UserPathView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public UserPathView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    public UserPathView(Context context, @Nullable AttributeSet attrs, int defStyleAttr, int defStyleRes) {
        super(context, attrs, defStyleAttr, defStyleRes);
        init();
    }

    private void init() {
        userPathList.add(new double[] {0, 0});
        rimRect = new RectF(0.1f, 0.1f, 0.9f, 0.9f);

        backgroundPaint = new Paint();
        backgroundPaint.setFilterBitmap(true);

        pathPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        pathPaint.setStrokeWidth(10.01f);
        pathPaint.setColor(Color.parseColor("#4CAF50"));
        pathPaint.setStyle(Paint.Style.STROKE);

        userCirclePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        userCirclePaint.setStyle(Paint.Style.FILL);
        userCirclePaint.setColor(Color.RED);
    }

    public void updateUserPathList(double[] newPos) {
        this.userPathList.add(new double[]{newPos[0], newPos[1]});
        this.invalidate();
    }

    public void clearUserPathList() {
        this.userPathList.clear();
        this.invalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {

        setBackgroundResource(R.drawable.floormap);

        int centerX = getWidth() / 2;
        int centerY = getHeight() / 2;
        for (int i = 0; i < this.userPathList.size(); i++ ) {

            if (i + 1 < this.userPathList.size()) {

                double[] end = this.userPathList.get(i + 1);
                double[] start = this.userPathList.get(i);

                //canvas.drawLine(centerX + (float)start[0] * 25, centerY - ((float)start[1] * 25),
                        //centerX + (float)end[0] * 25, centerY - ((float)end[1] * 25), pathPaint);
                if (i + 1 == this.userPathList.size() - 1) {
                    canvas.drawCircle(centerX + (float)end[0] * 25, centerY - ((float)end[1] * 25), 15, userCirclePaint);
                }
            }
        }

        /*double[] userCircleCenter = this.userPathList.get(this.userPathList.size() - 1);
        canvas.drawCircle(centerX + (float)userCircleCenter[0],
                centerY - ((float)userCircleCenter[1]), 20, pathPaint);*/
    }
}
