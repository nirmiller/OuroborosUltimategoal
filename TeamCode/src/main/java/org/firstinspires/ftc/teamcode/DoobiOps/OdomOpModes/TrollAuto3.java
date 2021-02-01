package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Bezier;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.JankOdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Shooter;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Wobble;
import org.firstinspires.ftc.teamcode.DoobiLibraries.jankOdom;

import java.util.ArrayList;

@Autonomous(group = "Auto", name = "Threaded Auto")
public class TrollAuto3 extends LinearOpMode {

    OdomDriveTrain odt;
    Wobble wobble;
    ShooterHardware sh;

    @Override
    public void runOpMode() throws InterruptedException {

        odt = new OdomDriveTrain(this);
        wobble = new Wobble(this);
        sh = new ShooterHardware(this);

        Thread thread1 = new Thread(new Runnable() {
            @Override
            public void run() {
                odt.timeMoveForward(4, .5);
            }
        });

        Thread thread2 = new Thread(new Runnable() {
            @Override
            public void run() {
                ElapsedTime time = new ElapsedTime();
                sh.ignite();
                while (time.milliseconds() < 2000){

                }
                sh.withdraw();
            }
        });

        waitForStart();

        thread1.start();
        thread2.start();

        thread1.interrupt();
        thread2.interrupt();





    }
}
