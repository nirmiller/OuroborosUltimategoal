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



        ArrayList<Point> OneRing0 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, -10, -30, -10, -60));
        ArrayList<Point> OneRing1 = Bezier.interpolateSpline(Bezier.getVariables(-10, -60, 0, -70, 0, -80));
        ArrayList<Point> OneRing2 = Bezier.interpolateSpline(Bezier.getVariables(0, -80, 0, -72, 0, -64));

        waitForStart();



        odt.splineMove(OneRing0, 1, 10);
        odt.splineMove(OneRing1, 1, 10);
        odt.splineMove(OneRing2, 1, 10);

        odt.end();

    }
}
