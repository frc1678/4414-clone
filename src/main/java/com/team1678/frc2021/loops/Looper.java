package com.team1678.frc2021.loops;

import com.team1678.frc2021.Constants;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper {
    public final double kPeriod = Constants.kLooperDt;

    private boolean running_;

    private final Notifier notifier_;
    private final List<Loop> loops_;
    private final Object taskRunningLock_ = new Object();
    private double timestamp_ = 0;
    private double dt_ = 0;
    public double dt(){ return dt_; }

    public Looper() {
        Runnable runnable_ = null;
        notifier_ = new Notifier(runnable_);
        running_ = false;
        loops_ = new ArrayList<>();
    }

    @Override
    public synchronized void register(Loop loop) {
        synchronized (taskRunningLock_) {
            loops_.add(loop);
        }
    }

    public synchronized void start() {
        if (!running_) {
            System.out.println("Starting loops");
            synchronized (taskRunningLock_) {
                timestamp_ = Timer.getFPGATimestamp();
                for (Loop loop : loops_) {
                    loop.onStart(timestamp_);
                }
                running_ = true;
            }
            notifier_.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop() {
        if (running_) {
            System.out.println("Stopping loops");
            notifier_.stop();
            synchronized (taskRunningLock_) {
                running_ = false;
                timestamp_ = Timer.getFPGATimestamp();
                for (Loop loop : loops_) {
                    System.out.println("Stopping " + loop);
                    loop.onStop(timestamp_);
                }
            }
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("looper_dt", dt_);
    }
}