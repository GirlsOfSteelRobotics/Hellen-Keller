package org.firstinspires.ftc.teamcode.Subsystems;

//package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class LinearSlide {
    private static final double TICKS_PER_INCH_LINEAR_SLIDE = 1;

    private static final double GROUND_HEIGHT = 0;



    private final DcMotor linearSlide;
    //might not be final just make sure later

    public LinearSlide(HardwareMap hardwareMap) {
        linearSlide = hardwareMap.get(DcMotor.class, "motorLinearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
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
        return linearSlide.getCurrentPosition() / TICKS_PER_INCH_LINEAR_SLIDE;
    }
}


