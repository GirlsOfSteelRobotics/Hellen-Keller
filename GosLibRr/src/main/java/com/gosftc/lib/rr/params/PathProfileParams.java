package com.gosftc.lib.rr.params;

public class PathProfileParams {
    // path profile parameters (in inches)
    public double maxWheelVel;
    public double minProfileAccel;
    public double maxProfileAccel;

    public PathProfileParams(double maxWheelVel, double minProfileAccel, double maxProfileAccel) {
        this.maxWheelVel = maxWheelVel;
        this.minProfileAccel = minProfileAccel;
        this.maxProfileAccel = maxProfileAccel;

    }
}
