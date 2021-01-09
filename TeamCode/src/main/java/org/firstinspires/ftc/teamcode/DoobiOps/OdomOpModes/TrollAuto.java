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



        ArrayList<Point> noRing0 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, -7, 36, -14, 72));
        ArrayList<Point> noRing1 = Bezier.interpolateSpline(Bezier.getVariables(-14, 70, -14, 67, -14, 64));

        ArrayList<Point> OneRing1 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, 10, 45, 20, 90));
        ArrayList<Point> OneRing2 = Bezier.interpolateSpline(Bezier.getVariables(0, -80, 0, -72, 0, -64));

        waitForStart();
        int pos = 0;

        if (pos == 0) {

            odt.splineMove(noRing0, .75, 10, 3);
            odt.splineMove(noRing1, .5, 5, 1);
        }
        else if (pos == 1){

            odt.splineMove(OneRing1, .6, .5, 7);
        }
       // odt.splineMove(OneRing1, .6, .5, 3);
        //odt.splineMove(OneRing2, 1, 10);

        odt.end();

    }
}
