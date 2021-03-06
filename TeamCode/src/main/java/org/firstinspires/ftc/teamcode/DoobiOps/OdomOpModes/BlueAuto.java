package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.BackupVision;
import org.firstinspires.ftc.teamcode.DoobiLibraries.HolonomicClasses.HolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Wobble;
import org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes.ShooterHardware;
import org.firstinspires.ftc.teamcode.Loop;

//@Autonomous(group = "Auto", name = "BlueAuto")
public class BlueAuto extends LinearOpMode {

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
        //backupVision = new BackupVision(this);
        wobble = new Wobble(this);
        sensors = new Sensors(this);


        Thread thread1 = new Thread(new Runnable() {
            @Override
            public void run() {
                hdt.gyroTurn180Fast(1500);
                hdt.gyroTurn180(750);
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
                //sleep(300);
                sh.hitRing();
                sleep(1000);
                sh.hitRing();
                sleep(1000);
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
        int pos = 0; //backupVision.senseBlue(this);
        while (opModeIsActive()) {

            if (pos == 0) {
                hdt.gyroHoloForward(1, 65, 3000, 0);
                wobble.releaseWobble();
                //hdt.gyroTurn180Fast(1400);
                //hdt.gyroTurn180(1000);
                hdt.gyroHoloForward(-.7, 9, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                loop.run();
                loop.end();

                hdt.gyroTurn180Fast(1000);
                hdt.gyroTurn180(500);
                hdt.gyroHoloForward(.7, 33, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.hookOpen();
                //hdt.turnPID(90, true, .7/90, .003, .052/90, .75);
                hdt.gyroTurn270Fast(1000);
                hdt.gyroTurn270(500);
                wobble.wobbleDown();
                sleep(1000);
                wobble.hookOpen();
                hdt.gyroHoloForward(.7, 11, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.getWobble();
                sleep(1000);
                hdt.gyroHoloForward(-.7, 15, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                //hdt.gyroTurnNinetyFast(300);
                hdt.gyroTurnStraightfast(500);
                hdt.gyroTurnStraight(1000);
                hdt.gyroHoloForward(1, 55, 3000, 0);
                wobble.releaseWobble();

                //hdt.gyroHoloStrafe(1, 15, true, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                hdt.gyroHoloForward(-.7, 9, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                hdt.gyroHoloStrafe(1, 17, false, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                hdt.gyroHoloForward(.7, 12, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());



                break;

            } else if (pos == 1) {

                hdt.gyroHoloForward(1, 82, 3000, 0);
                hdt.gyroHoloStrafe(1, 22, false, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.releaseWobble();
                hdt.gyroHoloStrafe(1, 20, true, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                hdt.gyroHoloForward(-.7, 31, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());


                loop.run();
                loop.end();


                //SECOND WOBBLE MOTHERFUCKER CANT TURN TO 90 for SOME REASON!

                hdt.gyroTurn180Fast(1000);
                hdt.gyroTurn180(500);
                hdt.gyroHoloForward(.7, 33, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.hookOpen();
                //hdt.turnPID(90, true, .7/90, .003, .052/90, .75);
                hdt.gyroTurn270Fast(1000);
                hdt.gyroTurn270(500);
                wobble.wobbleDown();
                sleep(1000);
                wobble.hookOpen();
                hdt.gyroHoloForward(.4, 15, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.getWobble();
                sleep(1000);
                hdt.gyroHoloForward(-.7, 15, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                //hdt.gyroTurnNinetyFast(300);
                hdt.gyroTurnStraightfast(500);
                hdt.gyroTurnStraight(1000);
                hdt.gyroHoloForward(1, 69, 3000, 0);
                hdt.gyroHoloStrafe(1, 24, false, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.releaseWobble();
                //hdt.gyroHoloStrafe(1, 15, true, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                hdt.gyroHoloForward(-.7, 7, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());




                break;
            } else if (pos == 2) {

                hdt.gyroTurnStraight(1000);
                break;
            } else if (pos == 4) {
                hdt.gyroHoloForward(1, 127, 3000, 0);
                wobble.releaseWobble();
                hdt.gyroHoloForward(-.7, 62, 3000, 0);

                loop.run();
                loop.end();

                //SECOND WOBBLE MOTHERFUCKER CANT TURN TO 90 for SOME REASON!

                hdt.gyroTurn180Fast(1000);
                hdt.gyroTurn180(500);
                hdt.gyroHoloForward(.7, 33, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.hookOpen();
                //hdt.turnPID(90, true, .7/90, .003, .052/90, .75);
                hdt.gyroTurn270Fast(1000);
                hdt.gyroTurn270(500);
                wobble.wobbleDown();
                sleep(1000);
                wobble.hookOpen();
                hdt.gyroHoloForward(.4, 13, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.getWobble();
                sleep(1000);
                hdt.gyroHoloForward(-.7, 17, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                //hdt.gyroTurnNinetyFast(300);
                hdt.gyroTurnStraightfast(500);
                hdt.gyroTurnStraight(1000);
                hdt.gyroHoloForward(1, 112, 3000, 0);
                wobble.releaseWobble();

                //hdt.gyroHoloStrafe(1, 15, true, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
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
