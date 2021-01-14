package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    @Override
    public void runOpMode() throws InterruptedException {

        odt = new OdomDriveTrain(this);
        wobble = new Wobble(this);

        int pos = 0;

        if (pos == 0) {

            wobble.wobbleUp();
            wobble.wobbleDown();
            odt.timeMoveForward(2000, .7);
        }
        else if (pos == 1){

            odt.timeMoveForward(1000, .7);

        }else if(pos == 2){

        }


        waitForStart();


    }
}
