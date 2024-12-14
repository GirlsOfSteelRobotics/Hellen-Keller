package org.firstinspires.ftc.teamcode.Subsystems;

//package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LinearSlide {
    private static final double TICKS_PER_INCH_LINEAR_SLIDE = 1;

    private static final double GROUND_HEIGHT = 0;

    public static double OUT_SPEED = 1;
    public static double IN_SPEED = -0.5;
    public static double KP = 0.005;

    private static double tickOffset;


    private final DcMotor linearSlide;
    //might not be final just make sure later

    public LinearSlide(HardwareMap hardwareMap) {
        linearSlide = hardwareMap.get(DcMotor.class, "motorLinearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void goUp() {
        linearSlide.setPower(OUT_SPEED);
    }

    public void goDown() {
        linearSlide.setPower(IN_SPEED);
    }

    public void stop() {
        linearSlide.setPower(0);
    }

    public double getHeight () {
        return (linearSlide.getCurrentPosition() - tickOffset) / TICKS_PER_INCH_LINEAR_SLIDE;
    }

    public boolean goToLength(double length){
        double error = getHeight() - length;
        double power = -KP * error;

        linearSlide.setPower(power);
        if(Math.abs(error) > 50) {
            return false;
        } else {
            return true;
        }
    }

    public void zeroEncoder() {
        tickOffset = linearSlide.getCurrentPosition();
    }
}


