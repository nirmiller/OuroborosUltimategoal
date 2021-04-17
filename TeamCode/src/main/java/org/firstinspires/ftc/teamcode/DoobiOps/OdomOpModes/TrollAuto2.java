package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.BackupVision;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Bezier;
import org.firstinspires.ftc.teamcode.DoobiLibraries.HolonomicClasses.HolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.JankOdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Wobble;
import org.firstinspires.ftc.teamcode.DoobiLibraries.jankOdom;

import java.util.ArrayList;

@Autonomous(group = "Auto", name = "The Sad Auto")
public class TrollAuto2 extends LinearOpMode {

    HolonomicDrivetrain hdt;
    OdomDriveTrain odt;
    Sensors sensors;
    Wobble wobble;
    @Override
    public void runOpMode() throws InterruptedException {

        hdt = new HolonomicDrivetrain(this);
        odt = new OdomDriveTrain(this);
        sensors = new Sensors(this);

        waitForStart();
        //TODO: Test gyroTurn180 for 5 seconds and turn it to a 45 degree angle while
        //looking at the telemetry
        hdt.gyroTurnStraight(5000);
        /*
        //TODO: Do the same thing for 270, but start it at 180 degrees
        hdt.gyroTurn270(5000);
*/
    }
}
