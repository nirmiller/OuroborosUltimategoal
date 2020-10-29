package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;

@Autonomous(group = "Auto", name = "Troll Auto")
public class TrollAuto extends LinearOpMode {
    OdomDriveTrain odt;

    @Override
    public void runOpMode() throws InterruptedException {

        odt = new OdomDriveTrain(this);

        waitForStart();

       // odt.goToPoint(24, 24, 1, 0, 4, 10);
       // odt.goToPoint(0, 48, 1, 0, 4, 10);
        odt.flex(0, 5);

        odt.end();

    }
}
