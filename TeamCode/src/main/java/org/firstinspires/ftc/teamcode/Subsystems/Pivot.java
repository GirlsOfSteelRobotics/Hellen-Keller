package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Pivot {
    private static final double TICKS_PER_INCH_PIVOT = 1;
    public static  double KP = 0.0005;
    public static boolean ENFORCE_LIMITS = true;
    public static double LOWER_LIMIT = 0;
    public static double UPPER_LIMIT = -600;
    public static double MAX_PID_POWER = 0.8;
    private static double tickOffset;
    private final DcMotor pivotMotor;
    OverflowEncoder encoder;
    private static double HELLO_EHSTA = 0;

    public Pivot(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx hackEncoderMotor = hardwareMap.get(DcMotorEx.class, "pivotMotor");//get the name of the motor
        encoder = new OverflowEncoder(new RawEncoder(hackEncoderMotor));
    }

    public void goUp() {
        if(ENFORCE_LIMITS && getAngle()<UPPER_LIMIT ) {
            pivotMotor.setPower(0);
        } else {
            pivotMotor.setPower(1);
            //might have to change later based on how heavy it is
        }
//        pivotMotor.setPower(UP_POWER);
    }

    public void goDown() {
       if (ENFORCE_LIMITS && getAngle()>LOWER_LIMIT) {
           pivotMotor.setPower(0);
       } else {
           pivotMotor.setPower(-0.2);
           //might have to change direction idk yet
       }
//        pivotMotor.setPower(DOWN_POWER);
    }

    public void stop() {
        pivotMotor.setPower(0);
    }

    public double getHeight () {
        return pivotMotor.getCurrentPosition() / TICKS_PER_INCH_PIVOT;
    }

    public double getAngle() {
        PositionVelocityPair ticks = encoder.getPositionAndVelocity();
        return (ticks.position - tickOffset) * TICKS_PER_INCH_PIVOT;
    }

    public boolean goToAngle(double angle) {
        double error = getAngle()-angle;
        double power = -KP * error;
        if (power > MAX_PID_POWER) {
            power = MAX_PID_POWER;
        } else if (power < -MAX_PID_POWER) {
            power = -MAX_PID_POWER;
        }
        pivotMotor.setPower(power);
        if (Math.abs(error) > 50) {
            return false;
        }
        else {
            return true;
        }

    }
    public double getPower() {
        return pivotMotor.getPower();
    }

    public void zeroEncoder() {
        tickOffset = encoder.getPositionAndVelocity().position;
    }

}
