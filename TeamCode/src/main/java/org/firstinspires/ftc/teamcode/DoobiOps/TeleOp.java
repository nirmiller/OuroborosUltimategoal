package org.firstinspires.ftc.teamcode.DoobiOps;


import org.firstinspires.ftc.teamcode.DoobiLibraries.TeleLib;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp



public class TeleOp extends TeleLib {


    @Override
    public void loop() {

        holonomicdrive();
        wobbleGoal();
        output();
        telemetry.update();
    }

    @Override
    public void stop() {

        global.interrupt();
        super.stop();

    }
}
