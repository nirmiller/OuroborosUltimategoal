package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Bezier;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;

import java.util.ArrayList;

@Autonomous(group = "Auto", name = "Troll Auto")
public class TrollAuto extends LinearOpMode {
    OdomDriveTrain odt;

    @Override
    public void runOpMode() throws InterruptedException {

        odt = new OdomDriveTrain(this);


        ArrayList<Point> spline0 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, 48, 0, 96, 0));

        waitForStart();



        odt.splineMove(spline0, 1, 10);

        odt.end();

    }
}
