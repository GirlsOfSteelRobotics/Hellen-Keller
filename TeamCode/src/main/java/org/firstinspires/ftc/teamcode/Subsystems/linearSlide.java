package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class linearSlide {
    private static final double TICKS_PER_INCH = 1;

    public static final double GROUND_HEIGHT = 0;



    private final DcMotor linearSlide;
    //might not be final just make sure later

    public linearSlide (HardwareMap hardwareMap) {
        linearSlide = hardwareMap.get(DcMotor.class, "motorLinearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        //change "linearSlide" to the name that we configured on the phone, once charged
    }

    public void goUp() {
        linearSlide.setPower(-1);
    }

    public void goDown() {
        linearSlide.setPower(1);
    }

    public void stop() {
        linearSlide.setPower(0);
    }

    public double getHeight () {
        double height = linearSlide.getCurrentPosition() / TICKS_PER_INCH;
        return height;
    }
}


