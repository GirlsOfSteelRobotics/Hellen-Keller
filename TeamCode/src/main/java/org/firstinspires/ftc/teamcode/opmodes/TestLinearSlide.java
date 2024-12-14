package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;

@Config
@TeleOp
public class TestLinearSlide extends LinearOpMode {
    public static double SLIDE_GOAL = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        LinearSlide linearSlide = new LinearSlide(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                linearSlide.goToLength(SLIDE_GOAL);
            }
            else {
                linearSlide.stop();
            }


            if (gamepad1.back) {
                linearSlide.zeroEncoder();
            }

            telemetry.addData("slide length", linearSlide.getHeight());
            telemetry.update();
        }
    }
}
