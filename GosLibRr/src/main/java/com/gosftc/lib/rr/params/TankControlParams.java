package com.gosftc.lib.rr.params;

import com.acmerobotics.roadrunner.MotorFeedforward;

public class TankControlParams {
    public double kS;
    public double kV;
    public double kA;

    // turn controller gains
    public double turnGain;
    public double turnVelGain;

    public double ramseteZeta; // in the range (0, 1)
    public double ramseteBBar; // positive

    public TankControlParams(
            double kS, double kV, double kA,
            double turnGain, double turnVelGain,
            double ramseteZeta, double ramseteBBar) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;

        this.turnGain = turnGain;
        this.turnVelGain = turnVelGain;
    }

    public MotorFeedforward createFeedForward(double inPerTick) {
        return new MotorFeedforward(kS,
                kV / inPerTick, kA / inPerTick);
    }
}
