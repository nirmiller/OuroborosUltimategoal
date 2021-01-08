package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Bezier;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Wobble;

import java.util.ArrayList;

@Autonomous(group = "Auto", name = "Troll Auto")
public class TrollAuto extends LinearOpMode {
    OdomDriveTrain odt;
    Wobble wobble;

    @Override
    public void runOpMode() throws InterruptedException {

        odt = new OdomDriveTrain(this);
        wobble = new Wobble(this);



        ArrayList<Point> OneRing0 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, 0, 30, 0, 60));
        ArrayList<Point> OneRing1 = Bezier.interpolateSpline(Bezier.getVariables(0, 60, -5, 60, -10, 60));
        ArrayList<Point> OneRing2 = Bezier.interpolateSpline(Bezier.getVariables(0, -80, 0, -72, 0, -64));

        waitForStart();



        odt.splineMove(OneRing0, .75, 10, 7);
        odt.splineMove(OneRing1, .6, .5, 3);
        //odt.splineMove(OneRing2, 1, 10);

        odt.end();

    }
}
