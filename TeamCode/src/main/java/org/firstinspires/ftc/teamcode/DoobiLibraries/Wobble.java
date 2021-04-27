package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble {
    public Servo wobble1;
    public Servo wobble2;
    public Servo hook;
    int wobbleUpPos = 1;
    int wobbleDownPos = 0;
    int hookDownPos = 0;
    int hookUpPos = 1;
    LinearOpMode opMode;

    public Wobble(LinearOpMode opMode) {
        wobble1 = opMode.hardwareMap.servo.get("wobble1");
        wobble2 = opMode.hardwareMap.servo.get("wobble2");

        hook = opMode.hardwareMap.servo.get("whook");
        this.opMode = opMode;
        wobble1.setDirection(Servo.Direction.FORWARD);
        wobble2.setDirection(Servo.Direction.REVERSE);

        wobbleDown();
        hookOpen();
        opMode.sleep(1000);
        hookClose();
        opMode.sleep(1000);
        wobbleUp();


    }


    public void wobbleUp() {

        wobble1.setPosition(0);
        wobble2.setPosition(0);;
    }

    public void wobbleDown() {

        wobble1.setPosition(1);
        wobble2.setPosition(1);
    }

    public void hookClose() {
        hook.setPosition(0);
    }

    public void hookOpen() {
        hook.setPosition(1);
    }

    public void getWobble() {
        while (opMode.opModeIsActive()) {
            /*wobbleDown();
            opMode.sleep(800);
            hookOpen();
            opMode.sleep(300);*/
            hookClose();
            opMode.sleep(700);
            wobbleUp();
            break;
        }
    }
    public void releaseWobble() {
        while (opMode.opModeIsActive()) {
            wobbleDown();
            opMode.sleep(700);
            hookOpen();
            wobbleUp();
            opMode.sleep(500);
            break;
        }
    }
}



