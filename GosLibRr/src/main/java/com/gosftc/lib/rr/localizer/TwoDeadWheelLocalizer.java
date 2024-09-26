package com.gosftc.lib.rr.localizer;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.gosftc.lib.rr.messages.TwoDeadWheelInputsMessage;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class TwoDeadWheelLocalizer implements Localizer {
    public static class Params {
        // y position of the parallel encoder (in tick units)
        public final double parYTicks;

        // x position of the perpendicular encoder (in tick units)
        public final double perpXTicks;

        public final double inPerTick;

        public Params(double parYTicks, double perpXTicks, double inPerTick) {
            this.parYTicks = parYTicks;
            this.perpXTicks = perpXTicks;
            this.inPerTick = inPerTick;
        }
    }

    private final Params params;

    private final Encoder par, perp;
    private final IMU imu;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public TwoDeadWheelLocalizer(OverflowEncoder par, OverflowEncoder perp, IMU imu, Params params) {
        this.params = params;

        this.imu = imu;
        this.par = par;
        this.perp = perp;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", params);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        // Use degrees here to work around https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070
        AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        AngularVelocity angularVelocity = new AngularVelocity(
                UnnormalizedAngleUnit.RADIANS,
                (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
                angularVelocityDegrees.acquisitionTime
        );

        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - params.parYTicks * headingDelta,
                                parPosVel.velocity - params.parYTicks * headingVel,
                        }).times(params.inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - params.perpXTicks * headingDelta,
                                perpPosVel.velocity - params.perpXTicks * headingVel,
                        }).times(params.inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        return twist;
    }

    @Override
    public void validateParams() throws RuntimeException {
        if (params.perpXTicks == 0 && params.parYTicks == 0) {
            throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }
    }
    @Override
    public List<Encoder> getLeftEncoders() {
        return new ArrayList<>();
    }

    @Override
    public List<Encoder> getRightEncoders() {
        return new ArrayList<>();
    }

    @Override
    public List<Encoder> getParallelEncoders() {
        return Collections.singletonList(par);
    }

    @Override
    public List<Encoder> getPerpendicularEncoders() {
        return Collections.singletonList(perp);
    }
}
