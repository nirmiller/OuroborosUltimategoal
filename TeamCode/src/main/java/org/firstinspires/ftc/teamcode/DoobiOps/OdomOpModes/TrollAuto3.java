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
                odt.encoderMove(.7, 50,  4);
                odt.turnPID(45, false, .7/45, .02, .02/45, 1.5);
                odt.encoderMove(.4, 10,  4);
                odt.turnPID(45, true, .7/45, .02, .02/45, 2);
                odt.encoderMove(-.6, 24,  4);
                odt.gyroStrafe(.4, 24, false, 5);
                loop.run();

                break;

            case 1:
                odt.encoderMove(.7, 72,  4);
                odt.turnPID(45, true, .7/45, .02, .02/45, 1.5);
                odt.encoderMove(.4, 10,  4);
                odt.turnPID(45, false, .7/45, .02, .02/45, 2);
                odt.encoderMove(-.6, 24,  4);
                loop.run();
                break;

            case 4:
                odt.encoderMove(.8, 106, 5); //moves forward 106 inches
                odt.turnPID(45, false, .7/45, .02, .02/45, 1.5); //turns left
                odt.encoderMove(.5, 10, 5); //moves forward 10 inches
                odt.turnPID(45, true, .7/45, .02, .02/45, 1.5); //turns right back to straight
                odt.encoderMove(-.6, 60, 5); //moves backwards 60 inches
                odt.gyroStrafe(.5, 24, false, 5); //strafes right 24 inches
                loop.run();

                break;
        }

    }
}
