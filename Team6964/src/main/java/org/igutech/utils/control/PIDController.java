package org.igutech.utils.control;

import org.apache.commons.math3.util.FastMath;

/**
 * Created by Kevin on 7/21/2018.
 */

public class PIDController implements BasicController {

    private long currentTimeMillis;
    private double kP, kI, kD;
    private double iTerm = 0;
    private double prevError = 0;
    private double sp;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    @Override
    public void init() {
        currentTimeMillis = System.currentTimeMillis();
    }

    @Override
    public double update(double pv) {
        if (FastMath.abs(error(pv)) <= 1) return 0;
        double timeOffset = System.currentTimeMillis() - currentTimeMillis;
        timeOffset = Math.max(timeOffset, 1);
        timeOffset /= 1000;
        //Log.d("PID_CONTROLLER", "\t" + timeOffset + "");
        double p = kP * error(pv);
        iTerm += error(pv) * timeOffset;
        double i = kI * iTerm;
        double d = kD * ((error(pv) - prevError) / timeOffset);

        currentTimeMillis = System.currentTimeMillis();
        prevError = error(pv);
        return p + i + d;
    }

    @Override
    public void updateSetpoint(double sp) {
        this.sp = sp;
    }

    private double error(double pv) {
        return sp - pv;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }
}
