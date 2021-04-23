package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Bezier;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Holonomic;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Shooter;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Wobble;

import java.util.ArrayList;
//@Disabled
@Autonomous(group = "Auto", name = "Troll Auto")
public class TrollAuto extends LinearOpMode {
    OdomDriveTrain odt;
    Sensors sensors;

    @Override
    public void runOpMode() throws InterruptedException {

        odt = new OdomDriveTrain(this);
        //wobble = new Wobble(this);
        sensors = new Sensors(this);

        ArrayList<Point> spline1 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, 0, 24, 0, 48));
        waitForStart();


        while (opModeIsActive())
        {
            telemetry.addData("Angle :", sensors.getGyroYaw());
            telemetry.update();
        }
        //odt.encoderMove(.7, 24,  4);
        //odt.turnPID(179, false, .5 / 180, .02, .02 / 180, 2);


        odt.splineMove(spline1, 1, 1, 2);
        //odt.gyroStrafe(.7, 24, false, 3000);
        //odt.timestrafeMove(3000, .5, 1);


        odt.end();

    }
}
