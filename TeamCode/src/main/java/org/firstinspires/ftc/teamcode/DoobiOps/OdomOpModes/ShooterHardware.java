package org.firstinspires.ftc.teamcode.DoobiOps.OdomOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;
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

    public boolean liftReady;
    static double PIVOT_THETA_TO_ENCODER = 0.0;
    static double LIFT_INDEX_TO_ENCODER = 0.0;
    static double POWER_TO_VELOCITY = 0.0;

    LinearOpMode opMode;

    DcMotor shooter;
    DcMotor pivot;
    DcMotor lift;
    Servo mag;
    Sensors sensors;

    private double currentAngle;

    private double currentLiftHeightIndex;

    public ShooterHardware(LinearOpMode opMode)
    {
        this.opMode = opMode;
        sensors = new Sensors(opMode);

        shooter = opMode.hardwareMap.dcMotor.get("shooter");
        pivot = opMode.hardwareMap.dcMotor.get("pivot");
        lift = opMode.hardwareMap.dcMotor.get("lift");
        mag = opMode.hardwareMap.servo.get("mag");

        pivot.setDirection(DcMotor.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setPivotAngle();

        initPivot();

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftReady = false;
    }


    public void initPivot(){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        pivot.setPower(.2);
        while(time.milliseconds() < 200){}

        while (opMode.opModeIsActive() && !sensors.button.isPressed())
        {

            pivot.setPower(-.2);
        }
        pivot.setPower(0);
    }

    public void setPivotAngle()
    {
        double encoder = 650;
        while (pivot.getCurrentPosition() < encoder && opMode.opModeIsActive())
        {
            pivot.setPower(.8);
        }
        pivot.setPower(0);
        //opMode.telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());
        opMode.telemetry.update();

    }public void setPivotAngle2()
    {
        double encoder = 645;
        while (pivot.getCurrentPosition() > encoder && opMode.opModeIsActive())
        {
            pivot.setPower(-.05);
        }
        pivot.setPower(0);
        //opMode.telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());
        opMode.telemetry.update();

    }
    public void setPivotAngle3()
    {
        double encoder = 640;
        while (pivot.getCurrentPosition() > encoder && opMode.opModeIsActive())
        {
            pivot.setPower(-.05);
        }
        pivot.setPower(0);
        //opMode.telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());
        opMode.telemetry.update();

    }

    public void pivotDown()
    {
        double encoder = 0;
        while (pivot.getCurrentPosition() > encoder && opMode.opModeIsActive())
        {
            pivot.setPower(-.3);
        }
        pivot.setPower(0);
        //opMode.telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());
        opMode.telemetry.update();

    }

    public void pivotPID (double pos, boolean moveUp, double kP, double kI, double kD, double timeout)
    {
        ElapsedTime time = new ElapsedTime();
        ElapsedTime timeoutTimer = new ElapsedTime();

        double error;
        double power;

        double proportional;
        double integral = 0;
        double derivative;

        double prevRunTime;

        double initPos = pivot.getCurrentPosition();

        double lastError = pos - initPos;

        time.reset();
        timeoutTimer.reset();
        while (Math.abs(pos - pivot.getCurrentPosition()) > 10 && timeoutTimer.seconds() < timeout && opMode.opModeIsActive()) {
            prevRunTime = time.seconds();

            error = pos - pivot.getCurrentPosition();

            proportional = error * kP;

            integral += (error * (time.seconds() - prevRunTime)) * kI;


            derivative = ((error - lastError) / (time.seconds() - prevRunTime)) * kD;


            power = proportional + integral + derivative;

            if (moveUp)
            {
                pivot.setPower(power);
            }
            else
            {
                pivot.setPower(-power);
            }

            opMode.telemetry.addData("error ", error);
            opMode.telemetry.addData("P", proportional);
            opMode.telemetry.addData("I", integral);
            opMode.telemetry.addData("D", derivative);
            opMode.telemetry.addData("power", power);
            opMode.telemetry.update();

            lastError = error;

            opMode.idle();
            if (Math.abs(pos - pivot.getCurrentPosition()) < 10)
            {
                break;
            }

        }
        opMode.telemetry.addLine("exited");
        opMode.telemetry.update();

        pivot.setPower(0);
    }

    public void setLift()
    {
        //TODO: encoder pos now 300 so make sure to start the auto at the highest pos the lift can be without falling
        double encoder = 250;
        while (lift.getCurrentPosition() < encoder && opMode.opModeIsActive())
        {
            if (lift.getCurrentPosition() < (encoder * .75)) {
                lift.setPower(.6);
            }
            else
            {
                lift.setPower(.4);
            }
            //opMode.telemetry.addData("lift encoder pos: ", lift.getCurrentPosition());
            opMode.telemetry.update();

        }
        liftReady = true;


        if (!opMode.opModeIsActive()){
            lift.setPower(0);
        }else{
            lift.setPower(.19);
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
        opMode.sleep(200);
        mag.setPosition(1);
    }

}
