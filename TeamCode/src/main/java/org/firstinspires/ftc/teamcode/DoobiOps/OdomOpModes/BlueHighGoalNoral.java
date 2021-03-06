package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.BackupVision;
import org.firstinspires.ftc.teamcode.DoobiLibraries.HolonomicClasses.HolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Wobble;
import org.firstinspires.ftc.teamcode.Loop;

public class BlueHighGoalNoral extends LinearOpMode {

    Wobble wobble;
    ShooterHardware sh;
    Loop loop;
    BackupVision backupVision;
    HolonomicDrivetrain hdt;
    Sensors sensors;

    @Override
    public void runOpMode() throws InterruptedException {
        //wobble = new Wobble(this);
        sh = new ShooterHardware(this);
        loop = new Loop();
        hdt = new HolonomicDrivetrain(this);
        backupVision = new BackupVision(this);
        wobble = new Wobble(this);
        sensors = new Sensors(this);

        int pos = 0;
        Thread thread1 = new Thread(new Runnable() {
            @Override
            public void run() {
                //hdt.gyroHoloStrafe(1, 2, true, 500, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                hdt.gyroTurn180Fast(1500);
                hdt.gyroTurn180(1100);
                //sleep(1300);
            }
        });
        Thread thread2 = new Thread(new Runnable() {
            @Override
            public void run() {
                //2. mag up
                //hdt.gyroTurn180(500);
                ElapsedTime time = new ElapsedTime();
                time.reset();
                sleep(3000);
                sh.hitRing();
                sleep(600);
                sh.hitRing();
                sleep(600);
                sh.hitRing();
                sleep(600);
                //sh.pivotStop.setPosition(.55);
                sh.hitRing();
                sleep(500);
                //sh.pivotDown();
                sh.hitRing();
                sleep(500);
                sh.withdraw();
                sh.lift.setPower(0);
                sh.lift.setPower(0);
                wobble.wobbleUp();
                sh.pivotStop.setPosition(1);
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
        pos = 4;
        while (opModeIsActive()) {

            if (pos == 0) {
                hdt.gyroHoloForward(1, 66, 3000, 0);
                wobble.releaseWobble();
                hdt.gyroHoloForward(-.7, 13, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());

                //NEW STUFF
                //hdt.gyroTurnStraight(1000);

                loop.run();
                loop.end();
                hdt.gyroHoloForward(-.7, 10, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());

                break;

            } else if (pos == 1) {

                hdt.gyroHoloForward(1, 80, 3000, 0);
                sleep(300);
                hdt.gyroHoloStrafe(1, 16, false, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                sleep(300);
                wobble.releaseWobble();
                hdt.gyroHoloStrafe(1, 14, true, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
               // sleep(300);
                hdt.gyroHoloForward(-1, 35, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());

                loop.run();
                loop.end();

                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(700);
                //PROB DOESNT WORK

                hdt.gyroHoloForward(-1, 37, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                //sleep(300);

                hdt.gyroTurn270Fast(1000);
                hdt.gyroTurn270(700);

                wobble.hookOpen();
                //hdt.turnPID(90, true, .7/90, .003, .052/90, .75);
                wobble.wobbleDown();
                sleep(300);
                wobble.hookOpen();
                hdt.gyroHoloForward(.4, 16, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.getWobble();
                sleep(600);
                hdt.gyroHoloForward(-1, 14, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                hdt.gyroTurnStraightfast(700);
                hdt.gyroTurnStraight(300);
                hdt.gyroHoloForward(1, 75, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                sleep(300);
                hdt.gyroHoloStrafe(1, 14, false, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.releaseWobble();
                hdt.gyroHoloForward(-.4, 5, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.wobbleUp();
                //sh.setLift();
                //hdt.gyroHoloStrafe(1, 15, true, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());

                break;
            } else if (pos == 2) {

                hdt.gyroTurnStraight(1000);
                break;
            } else if (pos == 4) {
                hdt.gyroHoloForward(.8, 127, 4000, 0);
                wobble.releaseWobble();
                hdt.gyroHoloForward(-.7, 65, 3000, 0);
                //hdt.gyroHoloStrafe(1, 3, false, 500, sensors.getGyroYawwwwwwwwwwwwwwwwwww());


                //NEW STUFF
                //hdt.gyroTurnStraight(1000);


                loop.run();
                loop.end();
                hdt.gyroHoloForward(-.7, 20, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());


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
