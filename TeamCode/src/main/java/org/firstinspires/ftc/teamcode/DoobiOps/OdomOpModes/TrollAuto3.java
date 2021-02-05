package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.BackupVision;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Bezier;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.JankOdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Shooter;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Wobble;
import org.firstinspires.ftc.teamcode.DoobiLibraries.jankOdom;
import org.firstinspires.ftc.teamcode.Loop;

import java.util.ArrayList;

@Autonomous(group = "Auto", name = "Threaded Auto")
public class TrollAuto3 extends LinearOpMode {

    OdomDriveTrain odt;
    Wobble wobble;
    ShooterHardware sh;
    Loop loop;

    @Override
    public void runOpMode() throws InterruptedException {

        odt = new OdomDriveTrain(this);
        wobble = new Wobble(this);
        sh = new ShooterHardware(this);
        loop = new Loop();


        Thread thread1 = new Thread(new Runnable() {
            @Override
            public void run() {
                //1. strafe
                sleep(1000);
                odt.gyroStrafe(.7, 24, false, 4);
            }
        });

        Thread thread2 = new Thread(new Runnable() {
            @Override
            public void run() {
                //2. mag up

                sh.setLift();
            }
        });

        Thread thread3 = new Thread(new Runnable() {
            @Override
            public void run() {
                //3. pivot + ring shoot
                sleep(1000);
                sh.setPivotAngle();
                sleep(300);
                sh.hitRing();
                sleep(300);
                sh.hitRing();
                sleep(300);
                sh.hitRing();
            }
        });
        Thread thread4 = new Thread(new Runnable() {
            @Override
            public void run() {
                //4. shoot the whole time
                sh.ignite();
            }
        });


        loop.add(thread1);
        loop.add(thread2);
        loop.add(thread3);
        loop.add(thread4);

        waitForStart();

        switch (1) {
            case 0:
                break;

            case 1:
                break;

            case 4:
                break;
        }

    }
}
