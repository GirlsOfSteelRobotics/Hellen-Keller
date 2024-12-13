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
    //prolly have to change

    private static final double INITIAL_HEIGHT = 0;
    public static  double KP = 0.005;
    //have to find the KP slop
    public static double LOW_BASKET_SCORING_ANGLE = 150;
    //have to tune this to find this
    public static boolean ENFORCE_LIMITS = true;
    //need this for the limit on the top baskets and the low baskets
    //have to do PID for these values also to find the ticks
    public static double INTAKE_ANGLE = 270;
    public static double LOWER_LIMIT = 0;
    public static double UPPER_LIMIT = 94;
    //have to do pid to make sure that this is all right
    public static double UP_POWER = .75;
    public static double DOWN_POWER = -0.2;

    private static double tickOffset;
    private final DcMotor pivotMotor;
    OverflowEncoder encoder;

    public Pivot(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx hackEncoderMotor = hardwareMap.get(DcMotorEx.class, "rightBack");//get the name of the motor
        encoder = new OverflowEncoder(new RawEncoder(hackEncoderMotor));
    }

    public void goUp() {
        if(ENFORCE_LIMITS && getAngle()>UPPER_LIMIT ) {
            pivotMotor.setPower(0);
        } else {
            pivotMotor.setPower(0.5);
            //might have to change later based on how heavy it is
        }
//        pivotMotor.setPower(UP_POWER);
    }

    public void goDown() {
       if (ENFORCE_LIMITS && getAngle()<LOWER_LIMIT) {
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

//    public boolean goToAngle(double angle){
//        double error = getAngle()-angle;
//        double power = KP * error;
//        //idk ask pj for help on this one
//    }
    public double getPower() {
        return pivotMotor.getPower();
    }

    public void zeroEncoder() {
        tickOffset = encoder.getPositionAndVelocity().position;
    }

}
