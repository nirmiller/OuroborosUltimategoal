package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "TeleOp", name = "Shooter")
public class TeleShooter extends OpMode {

    DcMotor shooter;

    @Override
    public void init() {
        shooter = hardwareMap.dcMotor.get("bl");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {


        if(gamepad2.dpad_up)
        {
            shooter.setPower(1);
        }else if(gamepad2.dpad_down)
        {
            shooter.setPower(-1);
        }
        else
        {
            shooter.setPower(0);
        }
    }
}
