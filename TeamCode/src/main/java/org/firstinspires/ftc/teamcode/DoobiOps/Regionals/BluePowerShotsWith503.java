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

@Autonomous(group = "Autonomous", name = "(503) Blue Inside - Wobble then High Goal")
public class BluePowerShotsWith503 extends LinearOpMode {
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
                hdt.gyroTurn180FastRed(1500);
                hdt.gyroTurn180Red(1100);
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
                sh.pivotPID(650, true, .07 / 650, .01, .01 / 650, 2);
                sh.pivotPID(640, false, .07 / 640, .01, .01 / 640, 2);


            }
        });


        loop.add(thread1);
        loop.add(thread2);
        loop.add(thread3);
        //loop.add(thread4);

        waitForStart();
        pos = backupVision.senseBlue2(this);
        while (opModeIsActive()) {

            if (pos == 0) {
                hdt.gyroHoloPIDMovement(0, 0, 95, 6, .7/96, .01, .001);

                hdt.gyroTurnNinetyFast(1000);
                hdt.gyroTurnNinety(700);

                //hdt.gyroHoloForward(1, 30, 3000, 90);
                hdt.gyroHoloPIDMovement(0, -90, 30, 3);

                wobble.releaseWobble();

                //hdt.gyroHoloForward(1, 30, 3000, 90);
                hdt.gyroHoloPIDMovement(180, -90, 28, 3);

                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(700);

                //hdt.gyroHoloForward(1, 15, 3000, 0);
                hdt.gyroHoloPIDMovement(180, 0, 38, 3);

                loop.run();
                loop.end();

                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(700);


                hdt.gyroHoloPIDMovement(0, 0, 25, 3);                //hdt.gyroHoloForward(-1, 15, 3000, 0);


                break;

            } else if (pos == 1) {


                hdt.gyroHoloPIDMovement(0, 0, 105, 6, .7/114, .01, .001);


                wobble.releaseWobble();


                hdt.gyroHoloPIDMovement(180, 0, 45, 3);

                hdt.gyroHoloStrafe(.5, 4, true, 1000, 0);

                loop.run();
                loop.end();

                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(700);


                hdt.gyroHoloPIDMovement(0, 0, 30, 3);                //



                break;
            } else if (pos == 2) {

               //add anything we want to test
                break;
            } else if (pos == 4) {


                //hdt.gyroHoloForward(1, 66, 3000, 0);
                hdt.gyroHoloPIDMovement(0, 0, 140, 8);

                hdt.gyroTurnNinetyFast(1000);
                hdt.gyroTurnNinety(700);

                //hdt.gyroHoloForward(1, 30, 3000, 0);
                hdt.gyroHoloPIDMovement(0, -90, 30, 3);
                sleep(2000);
                wobble.releaseWobble();

                //hdt.gyroHoloForward(1, 30, 3000, 0);
                hdt.gyroHoloPIDMovement(180, -90, 23, 3);

                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(700);

                hdt.gyroHoloPIDMovement(180, 0, 70, 3);

                loop.run();
                loop.end();

                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(700);


                hdt.gyroHoloPIDMovement(0, 0, 25, 3);

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
