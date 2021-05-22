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


@Autonomous(group = "Autonomous", name = "Red High Goal then Wobble")
public class RedHighGoal_v2 extends LinearOpMode {

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


                //PID TURN TO RED HIGH GOAL


                hdt.gyroTurn180FastRed(1500);
                hdt.gyroTurn180Red(1400);
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
                sh.pivotPID(650, true, .07 / 650, .01, .01 / 650, 2);
                sh.pivotPID(640, false, .07 / 640, .01, .01 / 640, 2);


            }
        });


        loop.add(thread1);
        loop.add(thread2);
        loop.add(thread3);
        //loop.add(thread4);

        waitForStart();
        pos = backupVision.senseRed(this);
        while (opModeIsActive()) {

            if (pos == 0) {

                hdt.gyroHoloPIDMovement(0, 0, 72, 3);
                hdt.gyroHoloStrafe(.5, 7, true, 1000, 0);

                loop.run();
                loop.end();


                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(700);

                sleep(2000);
                hdt.gyroHoloStrafe(.5, 12, false, 3000, 0);
                wobble.releaseWobble();


                //hdt.gyroHoloStrafe(1, 30, false, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                hdt.gyroHoloStrafe(.5, 25, true, 3000, 0);
                //hdt.gyroHoloForward(.7, 20, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                hdt.gyroHoloPIDMovement(0, 0, 15, 3);
                break;

            } else if (pos == 1) {

                hdt.gyroHoloPIDMovement(0, 0, 63, 5);
                hdt.gyroHoloStrafe(.7, 8, true, 3000, 0);

                loop.run();
                loop.end();

                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(700);

                hdt.gyroHoloPIDMovement(0, 0, 46, 5);
                wobble.releaseWobble();
                sleep(500);
                hdt.gyroHoloPIDMovement(180, 0, 15, 5);


                break;
            } else if (pos == 2) {

                hdt.gyroTurnStraight(1000);
                break;
            } else if (pos == 4) {


                hdt.gyroHoloPIDMovement(0, 0, 70, 6);
                hdt.gyroHoloStrafe(.5, 8, true, 1000, 0);

                loop.run();
                loop.end();

                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(500);
                sleep(500);
                hdt.gyroHoloPIDMovement(0, 0, 60, 6);
                hdt.gyroHoloStrafe(.5, 15, false, 1000, 0);
                wobble.releaseWobble();
                hdt.gyroHoloPIDMovement(180, 0, 50, 6);

                break;
            }
            //loop.end();

            //loop.end();
            sh.lift.setPower(0);
            sh.withdraw();
            sh.pivot.setPower(0);
            wobble.wobble_TeleOp();
        }
    }
}
