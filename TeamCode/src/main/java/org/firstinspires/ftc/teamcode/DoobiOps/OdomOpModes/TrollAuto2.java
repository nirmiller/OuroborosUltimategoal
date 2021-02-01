package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.BackupVision;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Bezier;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.JankOdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Wobble;
import org.firstinspires.ftc.teamcode.DoobiLibraries.jankOdom;

import java.util.ArrayList;

@Autonomous(group = "Auto", name = "The Sad Auto")
public class TrollAuto2 extends LinearOpMode {

    OdomDriveTrain odt;
    Wobble wobble;
    BackupVision vision;

    @Override
    public void runOpMode() throws InterruptedException {

        odt = new OdomDriveTrain(this);
        wobble = new Wobble(this);
        vision = new BackupVision(this);
        int battery = -300;



        waitForStart();
        int pos = vision.senseBlue(this);
        telemetry.addData("pos", pos);
        telemetry.update();

        if (pos == 0) {

            wobble.wobbleUp();
            wobble.wobbleDown();
            odt.timeMoveForward(1850 + battery, .7);
            //odt.turnPID(45, false, .3/45, .1, .02/45, 2);
            odt.turn(.6, false);
            sleep(400);
            odt.choop();
            odt.timeMoveForward(500, .5);
            odt.timeMoveForward(300, -.5);


        }
        else if (pos == 1){
            odt.timeMoveForward(2400 + battery, .7);
            odt.turn(.6, true);
            sleep(400);
            odt.choop();
            //odt.turnPID(45, true, .5/45, .1, .02/45, 2);
            odt.timeMoveForward(500, .4);
            odt.turn(.6, false);
            sleep(300);
            odt.choop();
            //odt.turnPID(45, false, .5/45, .1, .02/45, 2);
            odt.timeMoveForward(900 + battery, -.5);

        }
        else if(pos == 2){
            odt.timeMoveForward(3200 + battery, .7);
            odt.turn(.6, false);
            sleep(300);
            odt.choop();
            //odt.turnPID(45, false, .5/45, .1, .02/45, 2);
            odt.timeMoveForward(550, .5);
            //odt.turn(.6, true);
            //sleep(350);
            //odt.choop();
            //odt.turnPID(45, true, .5/45, .1, .02/45, 2);
            odt.timeMoveForward(2100 + battery, -.5);

        }

    }
}
