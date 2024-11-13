package org.firstinspires.ftc.teamcode.opmodes.rr_tuning.mecanum;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous
public class pushBot extends LinearOpMode {
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-35, -61, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();

        Actions.runBlocking(drive.actionBuilder(beginPose)
                // START COPY AND PASTE
                .setReversed(true)
                .splineTo(new Vector2d(-35.5,-9), Math.toRadians(90))
                .strafeTo(new Vector2d(-45,-9))
                .strafeToConstantHeading(new Vector2d(-45,-55))
                .strafeTo(new Vector2d(-60,-60))
                .strafeTo(new Vector2d(-45, - 38))
                .strafeTo(new Vector2d(-45, -9))
                .strafeTo(new Vector2d(-56,-9))
                .strafeTo(new Vector2d(-56,-60))
                .strafeTo(new Vector2d(-56,-40))
                .strafeTo(new Vector2d(-56,-8))
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(-53,-8))
                .strafeTo(new Vector2d(-53,-29))
                .strafeTo(new Vector2d(-63,-20))
                .strafeTo(new Vector2d(-63,-60))


                // END COPY AND PASTE
                .build());

        }
    }



