package com.example.android.camera2basic.viewmodel;

import android.app.Application;
import android.arch.lifecycle.AndroidViewModel;
import android.support.annotation.NonNull;

import com.example.android.camera2basic.livedata.PositionSensorLiveData;

/**
 * Created by gio on 4/2/18.
 */

public class PositionSensorViewModel extends AndroidViewModel {

    public PositionSensorViewModel(@NonNull Application application) {
        super(application);
    }
}
