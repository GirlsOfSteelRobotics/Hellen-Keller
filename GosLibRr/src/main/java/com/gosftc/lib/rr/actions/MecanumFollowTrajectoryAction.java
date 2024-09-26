package com.gosftc.lib.rr.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.gosftc.lib.rr.drive.MecanumDrivetrain;
import com.gosftc.lib.rr.Drawing;
import com.gosftc.lib.rr.params.MecanumControlParams;

import com.gosftc.lib.rr.messages.DriveCommandMessage;
import com.gosftc.lib.rr.messages.MecanumCommandMessage;
import com.gosftc.lib.rr.messages.PoseMessage;

import java.lang.Math;
import java.util.List;

public final class MecanumFollowTrajectoryAction implements Action {
    private final MecanumControlParams params;
    private final MecanumKinematics kinematics;
    private final MecanumDrivetrain drive;
    private final double inPerTick;

    private final TimeTrajectory timeTrajectory;
    private double beginTs = -1;

    private final double[] xPoints, yPoints;

    public MecanumFollowTrajectoryAction(MecanumDrivetrain drive, MecanumKinematics kinematics, MecanumControlParams params, double inPerTick, TimeTrajectory t) {
        this.drive = drive;
        this.kinematics = kinematics;
        this.params = params;
        this.inPerTick = inPerTick;

        timeTrajectory = t;

        List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                0, t.path.length(),
                Math.max(2, (int) Math.ceil(t.path.length() / 2)));
        xPoints = new double[disps.size()];
        yPoints = new double[disps.size()];
        for (int i = 0; i < disps.size(); i++) {
            Pose2d p = t.path.get(disps.get(i), 1).value();
            xPoints[i] = p.position.x;
            yPoints[i] = p.position.y;
        }
    }

    @Override
    public boolean run(@NonNull TelemetryPacket p) {
        double t;
        if (beginTs < 0) {
            beginTs = Actions.now();
            t = 0;
        } else {
            t = Actions.now() - beginTs;
        }

        if (t >= timeTrajectory.duration) {
            drive.stop();
            return false;
        }

        Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
        drive.writeTargetPose(new PoseMessage(txWorldTarget.value()));

        PoseVelocity2d robotVelRobot = drive.updatePoseEstimate();
        Pose2d pose = drive.getPose();

        PoseVelocity2dDual<Time> command = new HolonomicController(
                params.axialGain, params.lateralGain, params.headingGain,
                params.axialVelGain, params.lateralVelGain, params.headingVelGain
        )
                .compute(txWorldTarget, pose, robotVelRobot);
        drive.write(new DriveCommandMessage(command));

        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        double voltage = drive.getVoltage();

        final MotorFeedforward feedforward = params.createFeedForward(inPerTick);
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
        drive.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        drive.setDrivePowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

        p.put("x", pose.position.x);
        p.put("y", pose.position.y);
        p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

        Pose2d error = txWorldTarget.value().minusExp(pose);
        p.put("xError", error.position.x);
        p.put("yError", error.position.y);
        p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

        // only draw when active; only one drive action should be active at a time
        Canvas c = p.fieldOverlay();
        drive.drawPoseHistory(c);

        c.setStroke("#4CAF50");
        Drawing.drawRobot(c, txWorldTarget.value());

        c.setStroke("#3F51B5");
        Drawing.drawRobot(c, pose);

        c.setStroke("#4CAF50FF");
        c.setStrokeWidth(1);
        c.strokePolyline(xPoints, yPoints);

        return true;
    }

    @Override
    public void preview(Canvas c) {
        c.setStroke("#4CAF507A");
        c.setStrokeWidth(1);
        c.strokePolyline(xPoints, yPoints);
    }
}
