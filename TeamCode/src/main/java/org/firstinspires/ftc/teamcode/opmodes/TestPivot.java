package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;

@Config
@TeleOp
public class TestPivot extends LinearOpMode {
    public static double PIVOT_GOAL = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pivot pivot = new Pivot(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                pivot.goToAngle(PIVOT_GOAL);
            }
            else {
                pivot.stop();
            }

            if (gamepad1.back) {
                pivot.zeroEncoder();
            }

            telemetry.addData("arm ticks", pivot.getAngle());
            telemetry.update();
        }
    }
}
