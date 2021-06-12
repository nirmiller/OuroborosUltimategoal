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


//@Autonomous(group = "Autonomous", name = "Blue shoot and Park")
public class BlueParkAndShoot extends LinearOpMode {
    Wobble wobble;
    ShooterHardware sh;
    Loop loop;
    BackupVision backupVision;
    HolonomicDrivetrain hdt;
    Sensors sensors;
    Thread thread1 = new Thread(new Runnable() {
        @Override
        public void run() {


            //PID TURN TO RED HIGH GOAL


            hdt.gyroTurn180Fast(1500);
            hdt.gyroTurn180(1100);
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


    @Override
    public void runOpMode() throws InterruptedException {


        sh = new ShooterHardware(this);
        loop = new Loop();
        hdt = new HolonomicDrivetrain(this);
        backupVision = new BackupVision(this);
        wobble = new Wobble(this);
        sensors = new Sensors(this);

        loop.add(thread1);
        loop.add(thread2);
        loop.add(thread3);

        waitForStart();

        hdt.gyroHoloPIDMovement(0, 0, 67, 3, .5 / 67, .01, .001);

        loop.run();
        loop.end();


        hdt.gyroTurnStraightfast(1000);
        hdt.gyroTurnStraight(700);

        hdt.gyroHoloStrafe(.5, 30, false, 3000, 0);
        hdt.gyroHoloForward(.5, 25, 1000, 0);
        wobble.wobble_TeleOp();
    }


}
