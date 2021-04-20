package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble {
    public Servo wobble;
    public Servo hook;
    int wobbleUpPos = 1;
    int wobbleDownPos = 0;
    int hookDownPos = 0;
    int hookUpPos = 1;
    LinearOpMode opMode;

    public Wobble(LinearOpMode opMode) {
        wobble = opMode.hardwareMap.servo.get("wobble");
        hook = opMode.hardwareMap.servo.get("whook");
        this.opMode = opMode;
        wobble.setDirection(Servo.Direction.FORWARD);

        wobbleDown();
        hookOpen();
        opMode.sleep(1000);
        hookClose();
        opMode.sleep(1000);
        wobbleUp();


    }


    public void wobbleUp() {
        hook.setPosition(1);
    }

    public void wobbleDown() {
        hook.setPosition(0);
    }

    public void hookClose() {
        wobble.setPosition(0);
    }

    public void hookOpen() {
        wobble.setPosition(1);
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



