package com.example.android.camera2basic.ui;

import android.content.Context;
import android.graphics.Color;
import android.os.Handler;
import android.util.Log;

import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.components.Description;
import com.github.mikephil.charting.components.Legend;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.components.YAxis;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.LineData;
import com.github.mikephil.charting.data.LineDataSet;
import com.github.mikephil.charting.interfaces.datasets.ILineDataSet;

/**
 * Created by gio on 4/2/18.
 */

public class DynamicChart {
    private static final String tag = DynamicChart.class.getSimpleName();

    private String lineString = "";

    private static final int WINDOW_SIZE = 100;
    private static final float LINE_WIDTH = 3;

    private Context context;

    private LineChart chart;

    private Handler handler;

    private float[] acceleration;

    /**
     * Initialize a new Acceleration View object.
     */
    public DynamicChart(Context context, LineChart chart, String str) {
        this.context = context;
        this.chart = chart;

        initPlot();

        acceleration = new float[2];

        lineString = str;
    }

    public void onStartPlot() {
    }

    /**
     * Stop logging data.
     */
    public void onStopPlot() {
    }

    public void onPause() {
        onStopPlot();
        removeSets();
    }

    public void onResume() {
        addSets();
    }

    public void setAcceleration(float[] acceleration) {
        this.acceleration = acceleration;
        for (int i = 0; i < acceleration.length; i++) {
            setData(acceleration[i], i);
        }
    }

    /**
     * Set the data.
     *
     * @param data the data.
     */
    private void setData(double data, int key) {

        ILineDataSet set = chart.getData().getDataSetByIndex(key);

        if (set != null) {
            set.addEntry(new Entry(set.getEntryCount(), (float) data));

            chart.getData().notifyDataChanged();

            // let the chart know it's data has changed
            chart.notifyDataSetChanged();

            // limit the number of visible entries
            chart.setVisibleXRangeMaximum(WINDOW_SIZE);

            // move to the latest entry
            chart.moveViewToX(chart.getData().getEntryCount());
        }
    }

    /**
     * Add a lineDataSets to the plot.
     *
     * @param setName The name of the lineDataSets.
     * @param color   The lineDataSets color.
     */
    private void addSet(String setName, int color, int key) {

        LineDataSet set = new LineDataSet(null, setName);
        set.setAxisDependency(YAxis.AxisDependency.LEFT);
        set.setColor(color);
        set.setDrawCircles(false);
        set.setLineWidth(LINE_WIDTH);
        set.setDrawFilled(false);
        set.setHighLightColor(color);
        set.setDrawValues(false);
        chart.getData().addDataSet(set);

        setData(0, key);
    }

    /**
     * Remove a lineDataSets from the plot.
     *
     * @param key The unique lineDataSets key.
     */
    public void removeSet(int key) {
        if(chart.getData() != null) {
            chart.getData().removeDataSet(key);
        }
    }

    /**
     * Create the plot.
     */
    private void initPlot() {
        // enable touch gestures
        chart.setTouchEnabled(true);

        // enable scaling and dragging
        chart.setDragEnabled(true);
        chart.setScaleEnabled(true);
        chart.setDrawGridBackground(false);

        // if disabled, scaling can be done on x- and y-axis separately
        chart.setPinchZoom(true);

        // get the legend (only possible after setting data)
        Legend l = chart.getLegend();

        // modify the legend ...
        l.setForm(Legend.LegendForm.LINE);
        l.setTextColor(Color.GRAY);
        l.setWordWrapEnabled(true);

        Description description = chart.getDescription();
        description.setEnabled(false);

        XAxis xl = chart.getXAxis();
        xl.setTextColor(Color.GRAY);
        xl.setDrawGridLines(false);
        xl.setAvoidFirstLastClipping(true);
        xl.setEnabled(true);

        YAxis leftAxis = chart.getAxisLeft();
        leftAxis.setTextColor(Color.GRAY);
        leftAxis.setDrawGridLines(true);

        YAxis rightAxis = chart.getAxisRight();
        rightAxis.setEnabled(false);
    }

    private void addSets() {

        chart.clear();

        LineData data = new LineData();
        data.setValueTextColor(Color.WHITE);

        // add empty data
        chart.setData(data);

        this.addSet(lineString + " raw", Color.parseColor("#03A9F4"), 0);
        this.addSet(lineString, Color.parseColor("#F44336"), 1);
    }

    private void removeSets() {
        for (int i = 0; i < 3; i++) {
            this.removeSet(i);
        }
    }
}
