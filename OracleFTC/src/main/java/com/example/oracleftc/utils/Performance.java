package com.example.oracleftc.utils;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

public class Performance implements OpModeManagerNotifier.Notifications {

    private static Performance instance = new Performance();
    private long lastTime;

    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        eventLoop.getOpModeManager().registerListener(instance);
    }

    public static long loopTimeNano()
    {
        long currentTime = System.nanoTime();
        long deltaTime =currentTime- instance.lastTime;
        instance.lastTime = currentTime;
        return deltaTime;
    }


    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        lastTime=System.nanoTime();
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {

    }
}
