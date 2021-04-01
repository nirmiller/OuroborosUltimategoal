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
    Sensors sensors;
    @Override
    public void runOpMode() throws InterruptedException {

        hdt = new HolonomicDrivetrain(this);
        sensors = new Sensors(this);

        waitForStart();
/*
        for(int i = 0; i < 2; i++){
            hdt.gyroHoloForward(1, 24, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
            hdt.gyroHoloForward(-1, 24, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
        }
*/
        hdt.gyroHoloStrafe(1, 24, false, 2000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
        hdt.gyroHoloStrafe(1, 24, true, 2000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
        //hdt.gyroHoloForward(1, 24, 3000, sensors.getGyroYawwwwwwwwwwwwwwwwwww());
        //hdt.holoStrafe(.4, 20, false, 2000);
        //odt.goToPoint(24, 0, 0, 1, 5, 2);
        //hdt.end();
        hdt.end();


    }
}
