package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;

@Autonomous(group = "Auto", name = "Troll Auto")
public class TrollAuto extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        OdomDriveTrain odt = new OdomDriveTrain(this);

        waitForStart();

        odt.flex(0, 4);
        odt.global.interrupt();

    }
}
