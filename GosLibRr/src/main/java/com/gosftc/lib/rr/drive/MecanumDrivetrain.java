package com.gosftc.lib.rr.drive;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.gosftc.lib.rr.localizer.Localizer;
import com.gosftc.lib.rr.messages.DriveCommandMessage;
import com.gosftc.lib.rr.messages.MecanumCommandMessage;
import com.gosftc.lib.rr.messages.PoseMessage;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class MecanumDrivetrain {
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;
    private final DcMotorEx rightFront;

    public final VoltageSensor voltageSensor;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final Localizer localizer;

    private Pose2d pose;

    public MecanumDrivetrain(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightBack, DcMotorEx rightFront, Pose2d pose, VoltageSensor voltageSensor, Localizer localizer) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.rightFront = rightFront;
        this.pose = pose;
        this.voltageSensor = voltageSensor;
        this.localizer = localizer;
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        setDrivePowers(
                wheelVels.leftFront.get(0) / maxPowerMag,
                wheelVels.leftBack.get(0) / maxPowerMag,
                wheelVels.rightBack.get(0) / maxPowerMag,
                wheelVels.rightFront.get(0) / maxPowerMag
        );
    }

    public void setDrivePowers(double leftFrontPower, double leftBackPower, double rightBackPower, double rightFrontPower) {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }

    public void stop() {
        setDrivePowers(0, 0, 0, 0);
    }

    public void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }

    public Pose2d getPose() {
        return pose;
    }

    public void write(MecanumCommandMessage msg) {
        mecanumCommandWriter.write(msg);
    }

    public void writeTargetPose(PoseMessage msg) {
        targetPoseWriter.write(msg);
    }

    public void write(DriveCommandMessage msg) {
        driveCommandWriter.write(msg);
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public Localizer getLocalizer() {
        return localizer;
    }

    public List<DcMotorEx> getLeftMotors() {
        return Arrays.asList(leftFront, leftBack);
    }

    public List<DcMotorEx> getRightMotors() {
        return Arrays.asList(rightFront, rightBack);
    }
}
