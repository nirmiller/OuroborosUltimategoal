package org.firstinspires.ftc.teamcode.DoobiOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DoobiLibraries.TeleLib;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleDrive")
public class TeleOp extends TeleLib {


    @Override
    public void loop() {

        holonomicdrive();
        wobbleGoal();
        shooter();
        magazine();
        intake();
        telemetry.update();
    }

    @Override
    public void stop() {

        global.interrupt();
        super.stop();

    }
}
