package org.firstinspires.ftc.teamcode.DoobiOps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DoobiLibraries.DriveTrain;


@Autonomous(group = "troll", name = "TrollAuto")
public class TrollAuto extends LinearOpMode {

    DriveTrain driveTrain;


    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new DriveTrain(this);

        waitForStart();



        for(double i = 0; i <= 360; i += 45 )
        {
            driveTrain.driveTest(i, 0, .75, 2);
            sleep(400);
            driveTrain.driveTest(i, 0, -.75, 2);
        }





    }
}
