package org.firstinspires.ftc.teamcode.opmodes.rr_tuning.tank;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TankDrive;

public final class TankManualFeedbackTuner extends LinearOpMode {
    public static double goalDistance = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive.getLocalizer().validateParams();

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(goalDistance)
                        .lineToX(0)
                        .build());
        }
    }
}
