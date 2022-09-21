package com.example.android.camera2basic.ui;

import androidx.lifecycle.Observer;
import android.content.Context;
import android.net.Uri;
import android.os.Bundle;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import com.example.android.camera2basic.R;
import com.example.android.camera2basic.livedata.PositionSensorLiveData;
import com.github.mikephil.charting.charts.LineChart;

public class ChartFragment extends Fragment {
    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";

    // TODO: Rename and change types of parameters
    private String mParam1;
    private String mParam2;

    private DynamicChart xDynamicChart;
    private DynamicChart yDynamicChart;

    private OnFragmentInteractionListener mListener;

    float[] xAcceleration = new float[2];
    float[] yAcceleration = new float[2];

    public ChartFragment() {
        // Required empty public constructor
    }

    // TODO: Rename and change types and number of parameters
    public static ChartFragment newInstance() {
        return new ChartFragment();
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getArguments() != null) {
            mParam1 = getArguments().getString(ARG_PARAM1);
            mParam2 = getArguments().getString(ARG_PARAM2);
        }

    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        return inflater.inflate(R.layout.fragment_chart, container, false);
    }

    @Override
    public void onViewCreated(final View view, Bundle savedInstanceState) {
        // Create the graph plot
        LineChart xPlot = (LineChart) view.findViewById(R.id.east_west_line_chart);
        xDynamicChart = new DynamicChart(getContext(), xPlot, "X");

        LineChart yPlot = (LineChart) view.findViewById(R.id.north_south_line_chart);
        yDynamicChart = new DynamicChart(getContext(), yPlot, "y");

        xDynamicChart.onResume();
        xDynamicChart.onStartPlot();

        yDynamicChart.onResume();
        yDynamicChart.onStartPlot();

        PositionSensorLiveData.get(getContext().getApplicationContext()).observe(this, new Observer<double[]>() {
            @Override
            public void onChanged(@Nullable double[] doubles) {
                xAcceleration[0] = (float)doubles[0];
                xAcceleration[1] = (float)doubles[1];
                xDynamicChart.setAcceleration(xAcceleration);

                yAcceleration[0] = (float)doubles[2];
                yAcceleration[1] = (float)doubles[3];
                yDynamicChart.setAcceleration(yAcceleration);
            }
        });
    }

    @Override
    public void onResume() {
        super.onResume();

        xDynamicChart.onResume();
        xDynamicChart.onStartPlot();

        yDynamicChart.onResume();
        yDynamicChart.onStartPlot();
    }

    @Override
    public void onPause() {
        xDynamicChart.onStopPlot();
        xDynamicChart.onPause();

        yDynamicChart.onStopPlot();
        yDynamicChart.onPause();
        super.onPause();
    }

    // TODO: Rename method, update argument and hook method into UI event
    public void onButtonPressed(Uri uri) {
        if (mListener != null) {
            mListener.onFragmentInteraction(uri);
        }
    }

    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
        if (context instanceof OnFragmentInteractionListener) {
            mListener = (OnFragmentInteractionListener) context;
        } else {
            throw new RuntimeException(context.toString()
                    + " must implement OnFragmentInteractionListener");
        }
    }

    @Override
    public void onDetach() {
        super.onDetach();
        mListener = null;
    }

    public interface OnFragmentInteractionListener {
        // TODO: Update argument type and name
        void onFragmentInteraction(Uri uri);
    }
}
