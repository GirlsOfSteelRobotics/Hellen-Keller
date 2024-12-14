package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawServoSubsystem {
    //add extra wire.
    private final CRServo clawServo;

    public ClawServoSubsystem(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(CRServo.class, "clawServo");
    }

    public void openClaw(){
        clawServo.setPower(1);
    }

    public void closeClaw(){
        clawServo.setPower(-1);
    }

    public void stopClaw(){
        clawServo.setPower(0);
    }
}
