package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble {
    public Servo wobble;
    public Servo hook;
    int wobbleUpPos = 1;
    int wobbleDownPos = 0;
    int hookDownPos = 1;
    int hookUpPos = 0;

    public Wobble(LinearOpMode opMode){
        wobble = opMode.hardwareMap.servo.get("wobble");
        hook = opMode.hardwareMap.servo.get("whook");
        wobble.setPosition(wobbleUpPos);
    }

    public void wobbleUp(){
        wobble.setPosition(wobbleUpPos);
    }
    public void wobbleDown(){
        wobble.setPosition(wobbleDownPos);
    }
    public void hookDown(){
        wobble.setPosition(hookDownPos);
    }
    public void hookUp(){
        wobble.setPosition(wobbleUpPos);
    }
}
