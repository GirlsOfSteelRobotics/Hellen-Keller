package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawServoSubsystem {
    //add extra wire.
    private final Servo clawServo;

    public ClawServoSubsystem(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    public void openClaw(){
        clawServo.setPosition(1);
    }

    public void closeClaw(){
        clawServo.setPosition(-1);
    }

    public void stopClaw(){
        clawServo.setPosition(0);
    }
}
