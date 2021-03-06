package org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Holonomic;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;

import java.util.ArrayList;

public class JankOdomDriveTrain {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;
    private DcMotor shooter;
    private DcMotor pivot;
    private DcMotor lift;
    private Servo hook;
    private Servo wobble;
    private Servo mag;


    public OdometryGlobalCoordinatePosition globalPositionUpdate;
    public Thread global;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "fr", rbName = "br", lfName = "fl", lbName = "bl";
    String verticalLeftEncoderName = "fr", verticalRightEncoderName = "fl", horizontalEncoderName = "bl";
    double theta;

    final double COUNTS_PER_INCH = 308.876;

    LinearOpMode opMode;
    Sensors sensors;

    public JankOdomDriveTrain(LinearOpMode opMode)
    {
        this.opMode = opMode;
        sensors = new Sensors(opMode);

        left_front = opMode.hardwareMap.dcMotor.get("fl");
        right_front = opMode.hardwareMap.dcMotor.get("fr");
        left_back = opMode.hardwareMap.dcMotor.get("bl");
        right_back = opMode.hardwareMap.dcMotor.get("br");

        pivot = opMode.hardwareMap.dcMotor.get("pivot");
        shooter = opMode.hardwareMap.dcMotor.get("shooter");
        lift = opMode.hardwareMap.dcMotor.get("lift");
        //intake = hardwareMap.dcMotor.get("intake");

        hook = opMode.hardwareMap.servo.get("whook");
        wobble = opMode.hardwareMap.servo.get("wobble");
        mag = opMode.hardwareMap.servo.get("mag");

        pivot = opMode.hardwareMap.dcMotor.get("pivot");
        shooter = opMode.hardwareMap.dcMotor.get("shooter");
        lift = opMode.hardwareMap.dcMotor.get("lift");
        //intake = hardwareMap.dcMotor.get("intake");


        mag = opMode.hardwareMap.servo.get("mag");

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        verticalLeft = right_front;
        verticalRight = left_front;
        horizontal = left_back;

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 12);

        global = new Thread(globalPositionUpdate);
        global.start();

        opMode.telemetry.addData("Status", "Hardware Map Init Complete");
        opMode.telemetry.update();
    }

    public void end()
    {
        choop();
        global.interrupt();
    }
    public void resetEncoders() {

        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void splineMove(ArrayList<Point> spline, double power, double timeout, double allowed_error){
        for(Point p : spline)
        {
            goToPoint(p.getX(), p.getY(), 0, power, allowed_error, timeout);
        }
        choop();
    }

    public void goToPoint(double targetX, double targetY, double face, double power,  double allowedDistanceError, double timeout) {

        ElapsedTime time = new ElapsedTime();
        time.reset();

        //distance to x and y for trig calculations
        double distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
        double distanceToY = targetY - globalPositionUpdate.returnYCoordinate();

        //gets total distance needed to travel
        double distance = Math.hypot(distanceToX, distanceToY);
        double[] motor = new double[4];
        double k = 0;
        //distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
        // distanceToY = targetY - globalPositionUpdate.returnYCoordinate();
        distance = Math.hypot(distanceToX, distanceToY);
        //uses right triange trig to figure out what ange the robot needs to move at
        //maybe could integrate Nir's holonomic odom math into?
        double moveAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));
        double angleCorrection = 0;

        boolean crossed = false;

        while (opMode.opModeIsActive() && distance > allowedDistanceError && time.seconds() < timeout) {


            distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToY = targetY - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToX, distanceToY);

            angleCorrection = face - globalPositionUpdate.returnOrientation();
            motor = Holonomic.calcPowerAuto(moveAngle, globalPositionUpdate.returnOrientation(), 0);


            left_front.setPower(motor[0] * power);
            right_front.setPower(motor[1] * power);
            left_back.setPower(motor[2] * power);
            right_back.setPower(motor[3] * power);

            if(distanceToY < 7){
                crossed = true;
            }

            //figures out what power to set the motors to so we can move at this angle
            opMode.telemetry.addData("Angle : ", angleCorrection);
            opMode.telemetry.addData("X Position : ", globalPositionUpdate.returnXCoordinate());
            opMode.telemetry.addData("Y Position : ", globalPositionUpdate.returnYCoordinate());
            opMode.telemetry.update();

        }

        //choop();
    }

    public void turnPID(double angleChange, boolean turnRight, double kP, double kI, double kD, double timeout) {

        ElapsedTime time = new ElapsedTime();
        ElapsedTime timeoutTimer = new ElapsedTime();

        double error;
        double power;

        double proportional;
        double integral = 0;
        double derivative;

        double prevRunTime;

        double initAngle = sensors.getGyroYaw();
        double lastError = angleChange - Math.abs(sensors.getGyroYaw() - initAngle);

        time.reset();
        timeoutTimer.reset();

        while (Math.abs(sensors.getGyroYaw() - (angleChange + initAngle)) > 1 && timeoutTimer.seconds() < timeout && opMode.opModeIsActive()) {
            prevRunTime = time.seconds();

            error = angleChange - Math.abs(sensors.getGyroYaw() - initAngle);


            proportional = error * kP;

            integral += (error * (time.seconds() - prevRunTime)) * kI;


            derivative = ((error - lastError) / (time.seconds() - prevRunTime)) * kD;


            power = proportional + integral + derivative;

            turn(power, turnRight);

            opMode.telemetry.addData("error ", error);
            opMode.telemetry.addData("P", proportional);
            opMode.telemetry.addData("I", integral);
            opMode.telemetry.addData("D", derivative);
            opMode.telemetry.addData("power", power);
            opMode.telemetry.update();

            lastError = error;

            opMode.idle();

        }
        choop();

    }
    public void turn(double power, boolean isRight)
    {
        if(isRight)
        {
            right_front.setPower(-power);
            right_back.setPower(-power);
            left_front.setPower(power);
            left_back.setPower(power);
        }
        else
        {
            right_front.setPower(power);
            right_back.setPower(power);
            left_front.setPower(-power);
            left_back.setPower(-power);
        }
    }

    public double getEncoderAverage() {
        double count = 4.0;
        if(right_front.getCurrentPosition() == 0)
        {
            count--;
        }
        if(left_front.getCurrentPosition() == 0)
        {
            count--;
        }
        if(right_back.getCurrentPosition() == 0)
        {
            count--;
        }
        if(left_back.getCurrentPosition() == 0)
        {
            count--;
        }
        if(count == 0)
        {
            return 0;
        }
        return (left_front.getCurrentPosition() + right_front.getCurrentPosition()
                + right_back.getCurrentPosition() + left_back.getCurrentPosition()) / count;
    }
    private double getStrafeEncoderAverage(double direction) {

        double count = 4.0;
        double average = 0;

        if(right_front.getCurrentPosition() == 0)
        {
            count--;
        }
        if(left_front.getCurrentPosition() == 0)
        {
            count--;
        }
        if(right_back.getCurrentPosition() == 0)
        {
            count--;
        }
        if(left_back.getCurrentPosition() == 0)
        {
            count--;
        }
        if(direction > 0)
        {
            average = (((-1*left_front.getCurrentPosition() + right_front.getCurrentPosition()
                    + -1*right_back.getCurrentPosition() + left_back.getCurrentPosition())) ) / count;
        }
        else if(direction < 0)
        {
            average = (((left_front.getCurrentPosition() + -1*right_front.getCurrentPosition()
                    + right_back.getCurrentPosition() + -1*left_back.getCurrentPosition())))  / count;
        }
        return average;
    }

    public void setMotorsPower(double power)
    {

        left_front.setPower(power);
        right_front.setPower(power);
        right_back.setPower(power);
        left_back.setPower(power);
    }

    public void setStrafePower(double power)
    {
        right_front.setPower(-power);
        right_back.setPower(power);
        left_front.setPower(power);
        left_back.setPower(-power);
    }

    public double getTargetPercentile(double reading) {
        return Math.abs(getEncoderAverage() / reading);
    }
    public void encoderMove(double power, double distance, double runtime)
    {
        resetEncoders();
        ElapsedTime time = new ElapsedTime();

        double initEncoder = getEncoderAverage();

        time.reset();

        distance = distance * COUNTS_PER_INCH;

        while (Math.abs(getEncoderAverage() - initEncoder) < distance && time.seconds() < runtime && opMode.opModeIsActive()) {
            setMotorsPower(power);

            opMode.telemetry.addData("Encoder distance left", (distance - getEncoderAverage()));
            opMode.telemetry.update();

        }
        choop();

    }
    public void gyroStrafe(double power, double distance, boolean left, double timeout)
    {

        ElapsedTime time = new ElapsedTime();
        resetEncoders();
        double pos = 1;
        if(!left)
        {
            pos = -1;
        }


        double pfr = -power * pos;
        double pfl = power * pos;
        double pbl = -power * pos;
        double pbr = power * pos;



        double angle = sensors.getGyroYaw();
        double average = 0;
        while(opMode.opModeIsActive() && Math.abs(average) < Math.abs(distance) * COUNTS_PER_INCH && time.seconds() < timeout)
        {

            average = getStrafeEncoderAverage(pos);

            if(angle > 2)
            {
                if(left)
                {
                    pfr = -power;
                    pfl = power;
                    pbr = power * .86;
                    pbl = -power * .89;
                }
                else if(!left)
                {
                    pfr = power * .9;
                    pfl = -power * .9;
                    pbr = -power;
                    pbl = power;
                }
            }
            else if(angle < -2)
            {
                if(left)
                {
                    pfr = -power * .86;
                    pfl = power * .89;
                    pbr = power;
                    pbl = -power;
                }
                else if(!left)
                {
                    pfr = power;
                    pfl = -power;
                    pbr = -power * .9;
                    pbl = power * .9;
                }
            }
            else {

                pfr = -power * pos;
                pfl = power * pos;
                pbl = -power * pos;
                pbr = power * pos;
            }


            right_front.setPower(pfr);
            left_front.setPower(pfl);
            left_back.setPower(pbl);
            right_back.setPower(pbr);
            angle = sensors.getGyroYaw();
            opMode.telemetry.addData("Angle", angle);
            opMode.telemetry.update();
        }
        choop();
    }

    public void choop()
    {
        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }


}
