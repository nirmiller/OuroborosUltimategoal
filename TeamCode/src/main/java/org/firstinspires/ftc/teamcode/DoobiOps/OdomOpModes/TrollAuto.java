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
@Autonomous(group = "Auto", name = "Gyro Test")
public class TrollAuto extends LinearOpMode {
    OdomDriveTrain odt;
    Sensors sensors;

    @Override
    public void runOpMode() throws InterruptedException {

        sensors = new Sensors(this);

        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("Angle GyroYaw :", sensors.getGyroYaw());
            telemetry.addData("Angle  GyroYawwwwwwwwwwwwwww:", sensors.getGyroYawwwwwwwwwwwwwwwwwww());
            telemetry.update();
        }
        //odt.encoderMove(.7, 24,  4);


    }
}
