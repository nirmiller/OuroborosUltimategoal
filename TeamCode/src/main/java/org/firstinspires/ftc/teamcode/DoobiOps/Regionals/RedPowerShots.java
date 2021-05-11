package org.firstinspires.ftc.teamcode.DoobiOps.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.BackupVision;
import org.firstinspires.ftc.teamcode.DoobiLibraries.HolonomicClasses.HolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Wobble;
import org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes.ShooterHardware;
import org.firstinspires.ftc.teamcode.Loop;


@Autonomous(group = "Autonomous", name = "Red Power Shots")
public class RedPowerShots extends LinearOpMode {
    Wobble wobble;
    ShooterHardware sh;
    Loop loop;
    BackupVision backupVision;
    HolonomicDrivetrain hdt;
    Sensors sensors;

    @Override
    public void runOpMode() throws InterruptedException {
        //wobble = new Wobble(this);
        sh = new ShooterHardware(this, true);
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
                sleep(300);
                sh.hitRing();
                sleep(300);
                hdt.turnPID(5, true, .28 / 7, 0, 0, .4);
                sleep(300);
                sh.hitRing();
                sleep(300);
                hdt.turnPID(4, true, .28 / 7, 0, 0, .3);
                sleep(300);
                sh.hitRing();
                sleep(300);
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
                sh.pivotPID(650, true, .07 / 650, .01, .01 / 650, 2);
                sh.pivotPID(640, false, .07 / 640, .01, .01 / 640, 2);


            }
        });


        loop.add(thread1);
        loop.add(thread2);
        loop.add(thread3);
        //loop.add(thread4);

        waitForStart();
        pos = backupVision.senseBlue(this);
        while (opModeIsActive()) {

            if (pos == 0) {
                //hdt.gyroHoloForward(1, 66, 3000, 0);
                hdt.gyroHoloPIDMovement(0, 0, 66, 6);

                hdt.gyroTurn270Fast(1000);
                hdt.gyroTurn270(700);

                //hdt.gyroHoloForward(1, 30, 3000, 90);
                hdt.gyroHoloPIDMovement(0, 270, 30, 3);

                wobble.releaseWobble();

                //hdt.gyroHoloForward(1, 30, 3000, 90);
                hdt.gyroHoloPIDMovement(180, 270, 30, 3);

                hdt.gyroTurn180Fast(1000);
                hdt.gyroTurn180(700);

                //hdt.gyroHoloForward(1, 15, 3000, 0);
                hdt.gyroHoloPIDMovement(0, 0, 15, 3);

                loop.run();
                loop.end();

                hdt.gyroHoloPIDMovement(180, 0, 15, 3);
                //hdt.gyroHoloForward(-1, 15, 3000, 0);


                break;

            } else if (pos == 1) {

                //hdt.gyroHoloForward(1, 66, 3000, 0);
                hdt.gyroHoloPIDMovement(0, 0, 80, 6);

                wobble.releaseWobble();

                //hdt.gyroHoloForward(1, 30, 3000, 0);
                hdt.gyroHoloPIDMovement(180, 0, 30, 3);

                hdt.gyroTurn180Fast(1000);
                hdt.gyroTurn180(700);

                //hdt.gyroHoloForward(1, 15, 3000, 0);
                hdt.gyroHoloPIDMovement(0, 0, 30, 3);

                loop.run();
                loop.end();

                hdt.gyroHoloPIDMovement(180, 0, 15, 3);
                //hdt.gyroHoloForward(-1, 15, 3000, 0);


                break;
            } else if (pos == 2) {

                hdt.gyroTurnStraight(1000);
                break;
            } else if (pos == 4) {


                //hdt.gyroHoloForward(1, 66, 3000, 0);
                hdt.gyroHoloPIDMovement(0, 0, 127, 8);

                hdt.gyroTurn270Fast(1000);
                hdt.gyroTurn270(700);

                //hdt.gyroHoloForward(1, 30, 3000, 0);
                hdt.gyroHoloPIDMovement(0, 270, 30, 3);

                wobble.releaseWobble();

                //hdt.gyroHoloForward(1, 30, 3000, 0);
                hdt.gyroHoloPIDMovement(180, 270, 30, 3);

                hdt.gyroTurn180Fast(1000);
                hdt.gyroTurn180(700);

                //hdt.gyroHoloForward(1, 15, 3000, 0);
                hdt.gyroHoloPIDMovement(0, 0, 86, 3);

                loop.run();
                loop.end();

                hdt.gyroHoloPIDMovement(180, 0, 15, 3);
                //hdt.gyroHoloForward(-1, 15, 3000, 0);

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