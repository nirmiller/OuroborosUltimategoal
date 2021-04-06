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

@Autonomous(group = "Auto", name = "BlueAuto")
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
                hdt.gyroTurn180(1000);
            }
        });
        Thread thread2 = new Thread(new Runnable() {
            @Override
            public void run() {
                //2. mag up
                hdt.gyroTurn180(500);
                ElapsedTime time = new ElapsedTime();
                time.reset();
                sleep(3000);
                sh.hitRing();
                //sleep(300);
                sh.hitRing();
                sleep(500);

                sh.hitRing();
                sleep(500);

                sh.hitRing();
                sleep(500);
                //sh.pivotDown();
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
        int pos = 1; //backupVision.senseBlue(this);
        while (opModeIsActive()) {

            if (pos == 0) {
                hdt.gyroHoloForward(1, 65, 3000, 0);
                wobble.releaseWobble();
                //hdt.gyroTurn180Fast(1400);
                //hdt.gyroTurn180(1000);
                hdt.gyroHoloForward(-.7, 9, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                loop.run();
                loop.end();


                //SECOND WOBBLE MOTHERFUCKER CANT TURN TO 90 for SOME REASON!
                /*
                hdt.gyroTurn180Fast(1000);
                hdt.gyroTurn180(500);
                hdt.gyroHoloForward(.7, 30, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.hookOpen();
                */

                break;

            } else if (pos == 1) {

                hdt.gyroHoloForward(1, 82, 3000, 0);
                hdt.gyroHoloStrafe(1, 17, false, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.releaseWobble();
                hdt.gyroHoloStrafe(1, 15, true, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                hdt.gyroHoloForward(-.7, 31, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());


                loop.run();
                loop.end();


                //SECOND WOBBLE MOTHERFUCKER CANT TURN TO 90 for SOME REASON!

                hdt.gyroTurn180Fast(1000);
                hdt.gyroTurn180(500);
                hdt.gyroHoloForward(.7, 30, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.hookOpen();
                hdt.turnPID(90, true, .7/90, .003, .052/90, .75);



                break;
            } else if (pos == 2) {hdt.turnPID(180, false, .9/180, .007, .052/180, 1.5);
                break;
            } else if (pos == 4) {
                hdt.gyroHoloForward(1, 125, 3000, 0);
                wobble.releaseWobble();
                hdt.gyroHoloForward(-.7, 59, 3000, 0);

                loop.run();
                loop.end();

                //SECOND WOBBLE MOTHERFUCKER CANT TURN TO 90 for SOME REASON!

                hdt.gyroTurn180Fast(1000);
                hdt.gyroTurn180(500);
                hdt.gyroHoloForward(.7, 30, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                wobble.hookOpen();
                hdt.turnPID(90, true, .7/90, .003, .052/90, .75);

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
