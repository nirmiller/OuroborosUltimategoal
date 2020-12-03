package org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ShooterHardware {


    static double PIVOT_THETA_TO_ENCODER = 0.0;
    static double LIFT_INDEX_TO_ENCODER = 0.0;

    LinearOpMode opMode;

    private DcMotor shooter;
    private DcMotor pivot;
    private DcMotor lift;
    private double currentAngle;

    private double currentLiftHeightIndex;

    public ShooterHardware(LinearOpMode opMode)
    {
        this.opMode = opMode;

        pivot.setDirection(DcMotor.Direction.FORWARD);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }


    public void setPivotAngle(double theta)
    {

        double angleChange = theta - currentAngle;


        //turn on pivot motors until reached equal encoder level-set an level of error


        currentAngle = theta;
    }

    public void setLift(double heightIndex)
    {

        double indexChange = heightIndex - currentLiftHeightIndex;

        //move motors until reached height index

        currentLiftHeightIndex = heightIndex;

    }

    public void shoot()
    {
        shooter.setPower(1);
    }

    public void withdraw()
    {
        shooter.setPower(0);
    }
    public void resetShooter()
    {
        double angleChange = 0 - currentAngle;
        double indexChange = 0 - currentLiftHeightIndex;

        //Move pivotdown
        //Move Lift Down



        currentAngle = 0;
        currentLiftHeightIndex = 0;
    }
}
