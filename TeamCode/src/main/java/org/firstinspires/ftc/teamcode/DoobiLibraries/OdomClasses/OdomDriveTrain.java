package org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Holonomic;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;

import java.util.ArrayList;

public class OdomDriveTrain {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    public OdometryGlobalCoordinatePosition globalPositionUpdate;
    public Thread global;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "fr", rbName = "br", lfName = "fl", lbName = "bl";
    String verticalLeftEncoderName = "bl", verticalRightEncoderName = "br", horizontalEncoderName = "fl";

    double theta;

    final double COUNTS_PER_INCH = 308.876;

    LinearOpMode opMode;

    public OdomDriveTrain(LinearOpMode opMode)
    {

        this.opMode = opMode;

        right_front = opMode.hardwareMap.dcMotor.get(rfName);
        right_back = opMode.hardwareMap.dcMotor.get(rbName);
        left_front = opMode.hardwareMap.dcMotor.get(lfName);
        left_back = opMode.hardwareMap.dcMotor.get(lbName);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        verticalLeft = left_back;
        verticalRight = right_back;
        horizontal = left_front;

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 35);

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



    public void splineMove(ArrayList<Point> spline, double power, double timeout){
        for(Point p : spline)
        {
            goToPoint(p.getX(), p.getY(), p.getFace(), 1, 7, 2);
        }
        

    }


    public void goToPoint(double targetX, double targetY, double face, double power,  double allowedDistanceError, double timeout) {

        ElapsedTime time = new ElapsedTime();
        time.reset();

        //distance to x and y for trig calculations
        double distanceToX = (targetX * COUNTS_PER_INCH)- globalPositionUpdate.returnXCoordinate();
        double distanceToY = (targetY * COUNTS_PER_INCH) - globalPositionUpdate.returnYCoordinate();

        //gets total distance needed to travel
        double distance = Math.hypot(distanceToX, distanceToY);
        double[] motor = new double[4];
        double k = 0;
        distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
        distanceToY = targetY - globalPositionUpdate.returnYCoordinate();
        distance = Math.hypot(distanceToX, distanceToY);
        //uses right triange trig to figure out what ange the robot needs to move at
        //maybe could integrate Nir's holonomic odom math into?
        double moveAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));
        double angleCorrection = 0;

        while (opMode.opModeIsActive() && distance > allowedDistanceError && time.seconds() < timeout) {


            distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToY = targetY - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToX, distanceToY);

            angleCorrection = face - globalPositionUpdate.returnOrientation();
            motor = Holonomic.calcPowerAuto(moveAngle, globalPositionUpdate.returnOrientation(), angleCorrection);


            left_front.setPower(motor[0]);
            right_front.setPower(motor[1]);
            left_back.setPower(motor[2]);
            right_back.setPower(motor[3]);
            //figures out what power to set the motors to so we can move at this angle
            opMode.telemetry.addData("Angle : ", angleCorrection);
            opMode.telemetry.addData("X Position : ", globalPositionUpdate.returnXCoordinate());
            opMode.telemetry.addData("Y Position : ", globalPositionUpdate.returnYCoordinate());
            opMode.telemetry.update();

        }

       //choop();
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
