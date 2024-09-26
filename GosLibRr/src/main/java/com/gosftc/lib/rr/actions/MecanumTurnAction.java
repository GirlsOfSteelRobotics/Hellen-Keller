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
import com.acmerobotics.roadrunner.TimeTurn;
import com.gosftc.lib.rr.drive.MecanumDrivetrain;
import com.gosftc.lib.rr.Drawing;
import com.gosftc.lib.rr.params.MecanumControlParams;

import com.gosftc.lib.rr.messages.DriveCommandMessage;
import com.gosftc.lib.rr.messages.MecanumCommandMessage;
import com.gosftc.lib.rr.messages.PoseMessage;

public final class MecanumTurnAction implements Action {
    private final MecanumControlParams params;
    private final MecanumKinematics kinematics;
    private final MecanumDrivetrain drive;
    private final double inPerTick;

    private final TimeTurn turn;

    private double beginTs = -1;

    public MecanumTurnAction(MecanumDrivetrain drive, MecanumKinematics kinematics, MecanumControlParams params, double inPerTick, TimeTurn turn) {
        this.drive = drive;
        this.kinematics = kinematics;
        this.params = params;
        this.inPerTick = inPerTick;

        this.turn = turn;
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

        if (t >= turn.duration) {
            drive.stop();
            return false;
        }

        Pose2dDual<Time> txWorldTarget = turn.get(t);
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

        drive.setDrivePowers(
                feedforward.compute(wheelVels.leftFront) / voltage,
                feedforward.compute(wheelVels.leftBack) / voltage,
                feedforward.compute(wheelVels.rightBack) / voltage,
                feedforward.compute(wheelVels.rightFront) / voltage);

        Canvas c = p.fieldOverlay();
        drive.drawPoseHistory(c);

        c.setStroke("#4CAF50");
        Drawing.drawRobot(c, txWorldTarget.value());

        c.setStroke("#3F51B5");
        Drawing.drawRobot(c, pose);

        c.setStroke("#7C4DFFFF");
        c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

        return true;
    }

    @Override
    public void preview(Canvas c) {
        c.setStroke("#7C4DFF7A");
        c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
    }
}
