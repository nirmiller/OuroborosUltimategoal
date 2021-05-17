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


@Autonomous(group = "Autonomous", name = "Blue High Goal then Wobble")
public class BlueHighGoal_V2 extends LinearOpMode {
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


                hdt.gyroHoloPIDMovement(0, 0, 59, 3);


                loop.run();
                loop.end();

                sleep(1000);
                //sh.setLift();

                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(700);

                hdt.gyroHoloPIDMovement(0, 0, 13, 3);

                wobble.releaseWobble();

                sleep(300);
                hdt.gyroHoloPIDMovement(180, 0, 13, 3);

                hdt.gyroHoloStrafe(1, 25, false, 1000, 0);
                //hdt.gyroHoloForward(.7, 20, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
                hdt.gyroHoloPIDMovement(0, 0, 30, 3);
                break;

            } else if (pos == 1) {
                //hdt.gyroHoloForward(1, 80, 3000, 0);
                hdt.gyroHoloPIDMovement(0, 0, 96, 5, .7/84, .01, .001);
                sleep(300);
                hdt.gyroHoloStrafe(.7, 16, false, 3000, 0);
                sleep(300);
                wobble.releaseWobble();
                hdt.gyroHoloStrafe(.7, 16, true, 3000,0);
                // hdt.gyroHoloPIDMovement(-90, 0, 14, 3, .7/24, .01, .001);
                sleep(300);
                //hdt.gyroHoloForward(-1, 35, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());


                hdt.gyroHoloPIDMovement(0, 0, 61, 5, .7/61, .01, .001);

                loop.run();
                loop.end();

                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(700);


                sleep(500);
                //hdt.gyroHoloForward(1, 80, 3000, 0);
                hdt.gyroHoloPIDMovement(0, 0, 35, 5);
                sleep(300);
                hdt.gyroHoloStrafe(.7, 16, false, 3000, 0);
                sleep(300);
                wobble.releaseWobble();
                hdt.gyroHoloStrafe(.7, 16, true, 3000,0);
                // hdt.gyroHoloPIDMovement(-90, 0, 14, 3, .7/24, .01, .001);
                hdt.gyroHoloPIDMovement(180, 0, 35, 5);
                sleep(300);
                //hdt.gyroHoloForward(-1, 35, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());


                hdt.gyroHoloPIDMovement(0, 0, 30, 3);


                break;
            } else if (pos == 2) {

                hdt.gyroTurnStraight(1000);
                break;
            } else if (pos == 4) {
                //hdt.gyroHoloForward(1, 127, 3000, 0);
                hdt.gyroHoloPIDMovement(0, 0, 127, 6, .7/127, .01, .001);
                wobble.releaseWobble();
                //hdt.gyroHoloForward(-1, 65, 3000, 0);
                hdt.gyroHoloPIDMovement(180, 0, 65, 5);
                //hdt.gyroHoloStrafe(1, 3, false, 500, sensors.getGyroYawwwwwwwwwwwwwwwwwww());

                hdt.gyroHoloPIDMovement(0, 0, 62, 5);

                loop.run();
                loop.end();

                hdt.gyroTurnStraightfast(1000);
                hdt.gyroTurnStraight(500);

                sleep(500);
                //sh.setLift();
                hdt.gyroHoloPIDMovement(0, 0, 65, 5);
                wobble.releaseWobble();
                hdt.gyroHoloPIDMovement(180, 0, 65, 5, .4/65, .0075, .0015);

                hdt.gyroHoloPIDMovement(0, 0, 30, 3);

                break;
            }

            sh.lift.setPower(0);
            sh.withdraw();
            sh.pivot.setPower(0);
            wobble.wobble_TeleOp();
        }
    }
}
