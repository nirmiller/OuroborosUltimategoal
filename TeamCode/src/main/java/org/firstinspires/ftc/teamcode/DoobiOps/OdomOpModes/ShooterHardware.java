package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Shooter;

import static android.os.SystemClock.sleep;

public class ShooterHardware {

    public static double GOAL_X_BLUE = 0.0;
    public static double GOAL_Y_BLUE = 0.0;
    public static double GOAL_Z_BLUE = 0.0;
    public static double GOAL_X_RED = 0.0;
    public static double GOAL_Y_RED = 0.0;
    public static double GOAL_Z_RED = 0.0;

    public static double INITIAL_Z = 0.0;

    static double PIVOT_THETA_TO_ENCODER = 0.0;
    static double LIFT_INDEX_TO_ENCODER = 0.0;
    static double POWER_TO_VELOCITY = 0.0;

    LinearOpMode opMode;

    private DcMotor shooter;
    private DcMotor pivot;
    private DcMotor lift;
    private Servo mag;

    private double currentAngle;

    private double currentLiftHeightIndex;

    public ShooterHardware(LinearOpMode opMode)
    {
        this.opMode = opMode;

        shooter = opMode.hardwareMap.dcMotor.get("shooter");
        pivot = opMode.hardwareMap.dcMotor.get("pivot");
        lift = opMode.hardwareMap.dcMotor.get("lift");
        mag = opMode.hardwareMap.servo.get("mag");

        pivot.setDirection(DcMotor.Direction.FORWARD);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }


    public void setPivotAngle()
    {
        double encoder = 100;
        while (pivot.getCurrentPosition() < encoder)
        {
            pivot.setPower(.5);
        }
        pivot.setPower(0);
    }

    public void setLift()
    {
        double encoder = 100;
        while (lift.getCurrentPosition() < encoder)
        {
            if (lift.getCurrentPosition() < (encoder / 2)) {
                lift.setPower(.4);
            }
            else
            {
                lift.setPower(.19);
            }
        }

    }

    public void ignite()
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

    public void shoot(double current_x, double current_y, double goal_x, double goal_y, double goal_z)
    {
        //EVERYTHING IS IN METERS!!!
        double distance = Math.sqrt(Math.pow((goal_x - current_x), 2) + Math.pow((goal_y - current_y), 2));
        double z = goal_z - INITIAL_Z;

        double theta = Shooter.calcThetaPivot(distance, z, 1 * POWER_TO_VELOCITY);

        ignite();
        //loadShooter();



    }

    public void hitRing() {
        mag.setPosition(0);
        sleep(200);
        mag.setPosition(1);
    }

}
