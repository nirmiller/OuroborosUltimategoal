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
                sh.setLift();
            }
        });
        Thread thread2 = new Thread(new Runnable() {
            @Override
            public void run() {
                //2. mag up
                sleep(2000);
                while (!sh.liftReady) {
                }
                sh.ignite();
                sh.setPivotAngle();
                odt.turnPID(80, false, .5 / 80, .02, .02 / 80, 2);
                sleep(300);
                sh.hitRing();
                sleep(300);
                sh.hitRing();
                odt.turnPID(10, true, .3 / 10, .02, .02 / 10, 2);
                sleep(300);
                sh.hitRing();
                sleep(300);
                odt.turnPID(10, true, .3 / 10, .02, .02 / 10, 2);
                sleep(300);
                sh.hitRing();
                sleep(300);
                odt.encoderMove(.5, 10, 3);


            }
        });
        Thread thread3 = new Thread(new Runnable() {
            @Override
            public void run() {
                //4. shoot the whole time
                //sh.ignite();
            }
        });
        Thread thread4 = new Thread(new Runnable() {
            @Override
            public void run() {
                //3. pivot + ring shoot


            }
        });


        loop.add(thread1);
        loop.add(thread2);
        //loop.add(thread3);
        //loop.add(thread4);

        waitForStart();
        int pos = 2;
        while (opModeIsActive()) {

            if (pos == 0) {
                odt.encoderMove(.5, 65, 5);
                //odt.encoderMove(.5, 55, 4);
                odt.turnPID(45, false, .7 / 45, .03, .02 / 45, 1);
                odt.encoderMove(.4, 15, 4);
                odt.turnPID(45, true, .7 / 45, .03, .02 / 45, 1);
                odt.encoderMove(-.4, 15, 4);
                odt.gyroTurnStraight(500);
                odt.turnPID(90, false, .5 / 90, .02, .02 / 90, 1.5);
                odt.encoderMove(-.4, 25, 4);

                //loop.run();

                break;
            } else if (pos == 1) {
                odt.encoderMove(.5, 85, 7);
                //odt.encoderMove(.5, 55, 4);
                odt.turnPID(45, true, .7 / 45, .03, .02 / 45, 1);
                odt.encoderMove(.4, 15, 4);
                odt.turnPID(45, false, .7 / 45, .03, .02 / 45, 1);
                odt.encoderMove(-.4, 34, 4);
                odt.gyroTurnStraight(500);
                odt.turnPID(90, false, .5 / 90, .02, .02 / 90, 1.5);
                odt.encoderMove(-.4, 4, 4);
                //odt.encoderMove(-.4, 10, 4);

                //loop.run();
                break;
            } else if (pos == 2) {
                //TODO: Run this to check if the loop works. After you get it working, add it to the other cases
                loop.run();

                loop.end();
                break;
            } else if (pos == 4) {
                odt.encoderMove(.5, 30, 10);
                //TODO: if the robot is moving too crooked, add this turn PID to it
                odt.turnPID(10, false, .4 / 10, .02, .02 / 10, 1);
                odt.encoderMove(.6, 80, 10);

                //odt.encoderMove(.5, 55, 4);
                //odt.gyroTurnStraight(1000);
                odt.turnPID(30, false, .7 / 30, .03, .02 / 30, 1);
                odt.encoderMove(.4, 20, 4);
                odt.turnPID(30, true, .7 / 30, .03, .02 / 30, 1);
                odt.encoderMove(-.4, 67, 4);
                odt.gyroTurnStraight(500);
                odt.turnPID(90, false, .5 / 90, .02, .02 / 90, 1.5);
                odt.encoderMove(-.4, 25, 4);

                //loop.run();

                break;
            }
            //loop.end();

            //loop.end();
            sh.lift.setPower(0);
            sh.withdraw();
            sh.pivot.setPower(0);
        }
    }
}
