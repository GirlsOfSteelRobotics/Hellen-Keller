package com.gosftc.lib.rr.localizer;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.gosftc.lib.rr.messages.MecanumLocalizerInputsMessage;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MecanumDriveLocalizer implements Localizer {
    private final Encoder leftFront, leftBack, rightBack, rightFront;
    private final IMU imu;
    private final MecanumKinematics kinematics;

    private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
    private Rotation2d lastHeading;
    private boolean initialized;
    private final double inPerTick;

    public MecanumDriveLocalizer(DcMotorEx leftFrontMotor, DcMotorEx leftBackMotor, DcMotorEx rightBackMotor, DcMotorEx rightFrontMotor, LazyImu lazyImu, MecanumKinematics kinematics, double inPerTick) {
        leftFront = new OverflowEncoder(new RawEncoder(leftFrontMotor));
        leftBack = new OverflowEncoder(new RawEncoder(leftBackMotor));
        rightBack = new OverflowEncoder(new RawEncoder(rightBackMotor));
        rightFront = new OverflowEncoder(new RawEncoder(rightFrontMotor));


        imu = lazyImu.get();

        this.kinematics = kinematics;
        this.inPerTick = inPerTick;
    }

    @Override
    public Twist2dDual<Time> update() {
        PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
        PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
        PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
        PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        if (!initialized) {
            initialized = true;

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        double headingDelta = heading.minus(lastHeading);
        Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                new DualNum<Time>(new double[]{
                        (leftFrontPosVel.position - lastLeftFrontPos),
                        leftFrontPosVel.velocity,
                }).times(inPerTick),
                new DualNum<Time>(new double[]{
                        (leftBackPosVel.position - lastLeftBackPos),
                        leftBackPosVel.velocity,
                }).times(inPerTick),
                new DualNum<Time>(new double[]{
                        (rightBackPosVel.position - lastRightBackPos),
                        rightBackPosVel.velocity,
                }).times(inPerTick),
                new DualNum<Time>(new double[]{
                        (rightFrontPosVel.position - lastRightFrontPos),
                        rightFrontPosVel.velocity,
                }).times(inPerTick)
        ));

        lastLeftFrontPos = leftFrontPosVel.position;
        lastLeftBackPos = leftBackPosVel.position;
        lastRightBackPos = rightBackPosVel.position;
        lastRightFrontPos = rightFrontPosVel.position;

        lastHeading = heading;

        return new Twist2dDual<>(
                twist.line,
                DualNum.cons(headingDelta, twist.angle.drop(1))
        );
    }

    @Override
    public void validateParams() throws RuntimeException {
        // Nothing to do
    }

    @Override
    public List<Encoder> getLeftEncoders() {
        return Arrays.asList(leftFront, leftBack);
    }

    @Override
    public List<Encoder> getRightEncoders() {
        return Arrays.asList(rightFront, rightBack);
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
