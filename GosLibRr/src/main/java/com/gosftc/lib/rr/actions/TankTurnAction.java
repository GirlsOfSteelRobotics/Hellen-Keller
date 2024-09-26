package com.gosftc.lib.rr.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.gosftc.lib.rr.drive.TankDrivetrain;
import com.gosftc.lib.rr.messages.DriveCommandMessage;
import com.gosftc.lib.rr.messages.PoseMessage;
import com.gosftc.lib.rr.messages.TankCommandMessage;
import com.gosftc.lib.rr.Drawing;

import com.gosftc.lib.rr.params.TankControlParams;

public final class TankTurnAction implements Action {
    private final TankControlParams params;
    private final TankDrivetrain drive;
    private final TankKinematics kinematics;
    private final double inPerTick;

    private final TimeTurn turn;

    private double beginTs = -1;

    public TankTurnAction(TankDrivetrain drive, TankKinematics kinematics, TankControlParams params, double inPerTick, TimeTurn turn) {
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

        PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<>(
                Vector2dDual.constant(new Vector2d(0, 0), 3),
                txWorldTarget.heading.velocity().plus(
                        params.turnGain * pose.heading.minus(txWorldTarget.heading.value()) +
                        params.turnVelGain * (robotVelRobot.angVel - txWorldTarget.heading.velocity().value())
                )
        );
        drive.write(new DriveCommandMessage(command));

        TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        double voltage = drive.getVoltage();
        final MotorFeedforward feedforward = params.createFeedForward(inPerTick);
        double leftPower = feedforward.compute(wheelVels.left) / voltage;
        double rightPower = feedforward.compute(wheelVels.right) / voltage;
        drive.write(new TankCommandMessage(voltage, leftPower, rightPower));

        drive.setDrivePowers(leftPower, rightPower);

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
