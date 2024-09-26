package com.gosftc.lib.rr.localizer;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.gosftc.lib.rr.messages.TankLocalizerInputsMessage;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class TankDriveLocalizer implements Localizer {
    private final List<Encoder> leftEncs, rightEncs;
    private final TankKinematics kinematics;

    private double lastLeftPos, lastRightPos;
    private boolean initialized;
    private final double inPerTick;

    public TankDriveLocalizer(List<DcMotorEx> leftMotors, List<DcMotorEx> rightMotors, TankKinematics kinematics, double inPerTick) {
        {
            List<Encoder> leftEncs = new ArrayList<>();
            for (DcMotorEx m : leftMotors) {
                Encoder e = new OverflowEncoder(new RawEncoder(m));
                leftEncs.add(e);
            }
            this.leftEncs = Collections.unmodifiableList(leftEncs);
        }

        {
            List<Encoder> rightEncs = new ArrayList<>();
            for (DcMotorEx m : rightMotors) {
                Encoder e = new OverflowEncoder(new RawEncoder(m));
                rightEncs.add(e);
            }
            this.rightEncs = Collections.unmodifiableList(rightEncs);
        }

        this.kinematics = kinematics;
        this.inPerTick = inPerTick;
    }

    @Override
    public Twist2dDual<Time> update() {
        List<PositionVelocityPair> leftReadings = new ArrayList<>(), rightReadings = new ArrayList<>();
        double meanLeftPos = 0.0, meanLeftVel = 0.0;
        for (Encoder e : leftEncs) {
            PositionVelocityPair p = e.getPositionAndVelocity();
            meanLeftPos += p.position;
            meanLeftVel += p.velocity;
            leftReadings.add(p);
        }
        meanLeftPos /= leftEncs.size();
        meanLeftVel /= leftEncs.size();

        double meanRightPos = 0.0, meanRightVel = 0.0;
        for (Encoder e : rightEncs) {
            PositionVelocityPair p = e.getPositionAndVelocity();
            meanRightPos += p.position;
            meanRightVel += p.velocity;
            rightReadings.add(p);
        }
        meanRightPos /= rightEncs.size();
        meanRightVel /= rightEncs.size();

        FlightRecorder.write("TANK_LOCALIZER_INPUTS",
                 new TankLocalizerInputsMessage(leftReadings, rightReadings));

        if (!initialized) {
            initialized = true;

            lastLeftPos = meanLeftPos;
            lastRightPos = meanRightPos;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        TankKinematics.WheelIncrements<Time> twist = new TankKinematics.WheelIncrements<>(
                new DualNum<Time>(new double[] {
                        meanLeftPos - lastLeftPos,
                        meanLeftVel
                }).times(inPerTick),
                new DualNum<Time>(new double[] {
                        meanRightPos - lastRightPos,
                        meanRightVel,
                }).times(inPerTick)
        );

        lastLeftPos = meanLeftPos;
        lastRightPos = meanRightPos;

        return kinematics.forward(twist);
    }

    @Override
    public void validateParams() throws RuntimeException {
        // Nothing to do
    }

    @Override
    public List<Encoder> getLeftEncoders() {
        return leftEncs;
    }

    @Override
    public List<Encoder> getRightEncoders() {
        return rightEncs;
    }

    @Override
    public List<Encoder> getParallelEncoders() {
        return new ArrayList<>();
    }

    @Override
    public List<Encoder> getPerpendicularEncoders() {
        return new ArrayList<>();
    }
}
