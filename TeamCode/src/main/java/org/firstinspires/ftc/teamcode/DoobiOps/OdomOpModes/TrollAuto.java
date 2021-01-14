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



        ArrayList<Point> noRing0 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, -7, 36, -14, 68));
        ArrayList<Point> noRing1 = Bezier.interpolateSpline(Bezier.getVariables(-14, 68, -14, 66, -14, 64));
        ArrayList<Point> noRing01 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, -10, 64, -7, 63));

        ArrayList<Point> OneRing1 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, -5, 45, 20, 90));
        ArrayList<Point> OneRing2 = Bezier.interpolateSpline(Bezier.getVariables(20, 90, 20, 72, 20, 63));

        ArrayList<Point> FourRing1 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, -5, 63, 0, 110));
        ArrayList<Point> FourRing2 = Bezier.interpolateSpline(Bezier.getVariables(0, 110, 5, 80, 0, 72));
        waitForStart();


        int pos = 1;

        if (pos == 0) {

            wobble.wobbleUp();
            wobble.wobbleDown();
            odt.splineMove(noRing01, .75, 10, 3);
           //odt.splineMove(noRing1, .5, 5, 2);
        }
        else if (pos == 1){

            odt.splineMove(OneRing1, .8, 10, 10);
            odt.splineMove(OneRing2, .75, 5, 8);
        }else if(pos == 2){
            odt.splineMove(FourRing1, .8, 10, 7);
            odt.splineMove(FourRing2, .6, 5, 4);
        }
       // odt.splineMove(OneRing1, .6, .5, 3);
        //odt.splineMove(OneRing2, 1, 10);

        odt.end();

    }
}
