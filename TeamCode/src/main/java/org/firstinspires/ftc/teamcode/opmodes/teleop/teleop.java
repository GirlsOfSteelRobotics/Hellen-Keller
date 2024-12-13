package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ClawServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;

@TeleOp
public class teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        LinearSlide slide = new LinearSlide(hardwareMap);
        ClawServoSubsystem claw = new ClawServoSubsystem(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            drive.fieldRelativeDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            drive.updatePoseEstimate();

            Pose2d pose = drive.getPose();


            if(gamepad1.start && gamepad1.back) {
                drive.zeroWheels();
            }
            //linear slide
            if(gamepad1.right_trigger > 0.5) {
                slide.goUp();
            } else if (gamepad1.left_trigger>0.5) {
                slide.goDown();
            } else {
                slide.stop();
            }
            //need encoders!!!!!

            if(gamepad1.b) {
                claw.openClaw();
            } else if(gamepad1.x) {
                claw.closeClaw();
            } else {
                claw.stopClaw();
            }
            //pivot
            if(gamepad1.left_bumper){
                pivot.goUp();
                telemetry.addData("Going Up", true);
            } else if (gamepad1.right_bumper){
                pivot.goDown();
                telemetry.addData("Going Down", true);
            } else {
                pivot.stop();
            }

            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("extension", slide.getHeight());
            telemetry.addData("degrees", pivot.getHeight());
            telemetry.addData("arm power", pivot.getPower());
            telemetry.addData("arm angle", pivot.getAngle());
            telemetry.update();

        }


    }
}
