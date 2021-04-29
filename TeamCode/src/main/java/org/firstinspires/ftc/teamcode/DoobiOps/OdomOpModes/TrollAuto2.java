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

        waitForStart();
        hdt.gyroHoloForward(.5, 24, 100000, 0);
        sleep(1000);
        hdt.gyroHoloForward(-.5, 24, 100000, 0);


    }
}
