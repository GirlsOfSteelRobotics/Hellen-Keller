package com.gosftc.lib.rr.params;

import com.acmerobotics.roadrunner.MotorFeedforward;

public class MecanumControlParams {
    // feedforward parameters (in tick units)
    public double kS;
    public double kV;
    public double kA;

    // path controller gains
    public double axialGain;
    public double lateralGain;
    public double headingGain; // shared with turn

    public double axialVelGain;
    public double lateralVelGain;
    public double headingVelGain; // shared with turn

    public MecanumControlParams(
            double kS, double kV, double kA,
            double axialGain, double lateralGain, double headingGain,
            double axialVelGain, double lateralVelGain, double headingVelGain) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;

        this.axialGain = axialGain;
        this.lateralGain = lateralGain;
        this.headingGain = headingGain;

        this.axialVelGain = axialVelGain;
        this.lateralVelGain = lateralVelGain;
        this.headingVelGain = headingVelGain;
    }

    public MotorFeedforward createFeedForward(double inPerTick) {
        return new MotorFeedforward(kS,
                kV / inPerTick, kA / inPerTick);
    }

}
