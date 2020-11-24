package org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shooter {

    LinearOpMode opMode;

    private DcMotor shooter;
    private DcMotor pivot;
    private DcMotor lift;

    public Shooter(LinearOpMode opMode)
    {
        this.opMode = opMode;

        pivot.setDirection(DcMotor.Direction.FORWARD);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



    }


}
