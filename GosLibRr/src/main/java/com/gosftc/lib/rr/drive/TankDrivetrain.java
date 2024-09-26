package com.gosftc.lib.rr.drive;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.gosftc.lib.rr.messages.PoseMessage;
import com.gosftc.lib.rr.messages.TankCommandMessage;
import com.gosftc.lib.rr.localizer.Localizer;
import com.gosftc.lib.rr.messages.DriveCommandMessage;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.LinkedList;
import java.util.List;

public class TankDrivetrain {
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    private final List<DcMotorEx> leftMotors;
    private final List<DcMotorEx> rightMotors;

    public final VoltageSensor voltageSensor;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final Localizer localizer;

    private Pose2d pose;

    public TankDrivetrain(List<DcMotorEx> leftMotors, List<DcMotorEx> rightMotors, VoltageSensor voltageSensor, Localizer localizer, Pose2d pose) {
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
        this.localizer = localizer;
        this.voltageSensor = voltageSensor;
        this.pose = pose;
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        TankKinematics.WheelVelocities<Time> wheelVels = new TankKinematics(2).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        setDrivePowers(wheelVels.left.get(0) / maxPowerMag, wheelVels.right.get(0) / maxPowerMag);
    }

    public void setDrivePowers(double leftPower, double rightPower) {
        for (DcMotorEx m : leftMotors) {
            m.setPower(leftPower);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(rightPower);
        }
    }

    public void stop() {
        setDrivePowers(0, 0);
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

    public void write(TankCommandMessage msg) {
        tankCommandWriter.write(msg);
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
        return leftMotors;
    }

    public List<DcMotorEx> getRightMotors() {
        return rightMotors;
    }
}
