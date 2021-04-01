package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.BackupVision;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Wobble;
import org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes.ShooterHardware;
import org.firstinspires.ftc.teamcode.Loop;

@Autonomous(group = "Auto", name = "BlueAuto")
public class BlueAuto extends LinearOpMode {

    OdomDriveTrain odt;
    Wobble wobble;
    ShooterHardware sh;
    Loop loop;
    BackupVision backupVision;

    @Override
    public void runOpMode() throws InterruptedException {

        odt = new OdomDriveTrain(this);
        //wobble = new Wobble(this);
        sh = new ShooterHardware(this);
        loop = new Loop();
        backupVision = new BackupVision(this);
        wobble = new Wobble(this);


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
                //odt.gyroTurn180(500);
                ElapsedTime time = new ElapsedTime();
                time.reset();
                while (!sh.liftReady && time.milliseconds() < 1000) {
                }

                sh.hitRing();
                //sleep(300);
                sh.hitRing();
                sleep(300);

                sh.hitRing();
                sleep(300);

                sh.hitRing();
                sleep(300);
                //sh.pivotDown();
                sh.withdraw();
                sh.lift.setPower(0);
                //odt.encoderMove(-.5, 12, 3);
                sh.lift.setPower(0);
                loop.end();


            }
        });
        Thread thread3 = new Thread(new Runnable() {
            @Override
            public void run() {
                //4. shoot the whole time
                sh.ignite();
            }
        });
        Thread thread4 = new Thread(new Runnable() {
            @Override
            public void run() {
                //3. pivot + ring shoot
                ElapsedTime time = new ElapsedTime();
                time.reset();
                while (!sh.liftReady && time.milliseconds() < 1500) {
                }
                //
                sh.pivotPID(650, true, .07/650, .01, .01/650, 2);
                sh.pivotPID(640, false, .07/640, .01, .01/640, 2);



            }
        });


        loop.add(thread1);
        loop.add(thread2);
        loop.add(thread3);
        //loop.add(thread4);

        waitForStart();
        int pos = backupVision.senseBlue(this);
        while (opModeIsActive()) {

            if (pos == 0) {

                loop.run();
                loop.end();
                odt.gyroTurn180(1000);
                odt.encoderMove(.7, 13, 3);


                //odt.encoderMove(.7, 30, 3);
                odt.gyroStrafe(.5, 7, false, 3000);
                odt.gyroTurn180(1000);
                odt.encoderMove(.4, 5, 3);
                wobble.releaseWobble();

                odt.turnPID(7, false, .28 / 7, 0, 0, .4);
                wobble.getWobble();

                odt.turnPID(7, true, .28 / 7, 0, 0, .4);

                //sleep(5000);
                odt.encoderMove(-.5, 55, 4);
                //odt.turnPID(90, true, .5 / 90, .02, .02/90, 1.2);
                odt.gyroTurnNinetyFast(500);
                odt.gyroTurnNinety(1000);
                odt.encoderMove(.4, 10, 3);
                wobble.releaseWobble();
                odt.encoderMove(-.4, 5, 3);

                break;
            } else if (pos == 1) {
                odt.encoderMove(.5, 38, 7);
                odt.encoderMove(.7, 50, 7);

                //odt.encoderMove(.5, 55, 4);
                //odt.gyroTurnStraight(500);
                odt.turnPID(30, true, .7 / 45, .03, .02 / 45, .75);
                odt.encoderMove(.4, 9, 4);
                odt.turnPID(30, false, .7 / 45, .03, .02 / 45, .75);
                odt.encoderMove(-.5, 30, 4);
                odt.gyroTurnStraight(500);
                odt.turnPID(90, false, .5 / 90, .02, .02 / 90, 1.5);
                //odt.encoderMove(-.4, 4, 1);
                odt.gyroTurnNinety(500);
                odt.turnPID(88, false, .7 / 90, .01, .02/90, 2);
                odt.gyroTurn180(500);

                loop.run();
                loop.end();

                odt.gyroTurn180(1000);
                //odt.encoderMove(.7, 10, 3);
                //odt.gyroTurnNinetyFast(500);
                odt.gyroTurnStraight(100);
                //odt.turnPID(88, true, .4 / 90, .01, .01/90, 1.2);
                odt.gyroTurnNinetyFast(600);
                sleep(100);
                odt.gyroTurnNinety(500);
                odt.encoderMove(-.7, 40, 4);
                odt.gyroTurnNinety(500);
                //odt.encoderMove(.4, 10, 4);
                odt.turnPID(58, false, .5 / 90, .02, .02/90, 1.2);


                //odt.encoderMove(.7, 30, 3);
                odt.encoderMove(.7, 25.5, 3);
                odt.encoderMove(.4, 5, 3);

                wobble.releaseWobble();
                odt.turnPID(10, false, .28 / 7, 0, 0, .4);
                wobble.getWobble();
                odt.turnPID(10, true, .28 / 7, 0, 0, .4);
                odt.encoderMove(-.6, 15, 3);
                odt.turnPID(105, true, .7 / 90, .02, .02/90, 1.2);
                //odt.turnPID(43, true, .5 / 50, .02, .02/50, 1.2);
                odt.encoderMove(.7, 51, 4);
                wobble.releaseWobble();
                odt.encoderMove(-.4, 7, 3);



                break;
            } else if (pos == 2) {
                //odt.gyroStrafe(.5, 20, false, 3000);
                loop.run();
                loop.end();
                break;
            } else if (pos == 4) {

                odt.encoderMove(.5, 30, 10);
                //TODO: if the robot is moving too crooked, add this turn PID to it
                odt.turnPID(4, false, .7 / 10, .01, 0, .1);
                odt.encoderMove(.7, 80, 10);

                //odt.encoderMove(.5, 55, 4);
                //odt.gyroTurnStraight(1000);
                odt.turnPID(45, false, .7 / 45, .03, .02 / 45, 1);
                odt.encoderMove(.5, 20, 4);
                odt.turnPID(45, true, .7 / 45, .03, .02 / 45, 1);
                odt.gyroTurnStraight(500);
                odt.encoderMove(-.7, 57, 4);
                odt.gyroTurnStraight(500);
                odt.turnPID(90, false, .5 / 90, .02, .02 / 90, 1.5);
                odt.encoderMove(-.4, 12, 4);
                odt.gyroTurnNinety(500);
                odt.turnPID(88, false, .5 / 90, .01, .02 / 90, 2);
                odt.gyroTurn180(1000);

                loop.run();
                loop.end();
                odt.encoderMove(-.5, 12, 3);


              /*  odt.gyroTurn180(1000);
                //odt.encoderMove(.7, 10, 3);
                //odt.gyroTurnNinetyFast(500);
                odt.gyroTurnStraight(100);
                //odt.turnPID(88, true, .4 / 90, .01, .01/90, 1.2);
                odt.gyroTurnNinetyFast(600);
                sleep(50);
                odt.gyroTurnNinety(500);
                odt.encoderMove(-.8, 40, 4);
                odt.gyroTurnNinety(500);
                //odt.encoderMove(.4, 10, 4);
                odt.turnPID(60, false, .5 / 90, .02, .02/90, 1.2);


                //odt.encoderMove(.7, 30, 3);
                odt.encoderMove(.7, 28, 3);
                odt.encoderMove(.4, 4, 3);

                wobble.releaseWobble();
                odt.turnPID(7, false, .28 / 7, 0, 0, .4);
                wobble.getWobble();
                odt.turnPID(7, true, .28 / 7, 0, 0, .4);
                odt.encoderMove(-.4, 15, 3);
                odt.turnPID(80, true, .7 / 90, .02, .02/90, 1.2);
                odt.turnPID(30, true, .5 / 30, .02, .02/30, 1.2);
                odt.encoderMove(.8, 70, 5);
                wobble.releaseWobble();
                odt.encoderMove(-.4, 20, 3);
                //loop.run();*/

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
