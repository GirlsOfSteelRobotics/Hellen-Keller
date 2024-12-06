package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Pivot {
    private static final double TICKS_PER_INCH_PIVOT = 1;

    private static final double INITIAL_HEIGHT = 0;

    private final DcMotor pivotMotor;

    public Pivot(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void goUp() {
        pivotMotor.setPower(-1);
    }

    public void goDown() {
        pivotMotor.setPower(1);
    }

    public void stop() {
        pivotMotor.setPower(0);
    }

    public double getHeight () {
        return pivotMotor.getCurrentPosition() / TICKS_PER_INCH_PIVOT;
    }

}
