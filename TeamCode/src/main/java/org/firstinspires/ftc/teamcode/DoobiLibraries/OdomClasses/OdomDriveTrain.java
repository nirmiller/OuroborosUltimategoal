package org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OdomDriveTrain {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "fr", rbName = "br", lfName = "fl", lbName = "bl";
    String verticalLeftEncoderName = "bl", verticalRightEncoderName = "br", horizontalEncoderName = "fl";

    double theta;

    final double COUNTS_PER_INCH = 308.876;

    LinearOpMode opMode;

    public OdomDriveTrain(LinearOpMode opMode)
    {

        this.opMode = opMode;
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        right_front = opMode.hardwareMap.dcMotor.get(rfName);
        right_back = opMode.hardwareMap.dcMotor.get(rbName);
        left_front = opMode.hardwareMap.dcMotor.get(lfName);
        left_back = opMode.hardwareMap.dcMotor.get(lbName);

        verticalLeft = opMode.hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = opMode.hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = opMode.hardwareMap.dcMotor.get(horizontalEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.REVERSE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        opMode.telemetry.addData("Status", "Hardware Map Init Complete");
        opMode.telemetry.update();
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

    public void goToPosition(LinearOpMode opMode, double targetX, double targetY, double power, double orientation, double allowedDistanceError) {

        //distance to x and y for trig calculations
        double distanceToX = (targetX * COUNTS_PER_INCH)- globalPositionUpdate.returnXCoordinate();
        double distanceToY = (targetY * COUNTS_PER_INCH) - globalPositionUpdate.returnYCoordinate();

        //gets total distance needed to travel
        double distance = Math.hypot(distanceToX, distanceToY);

        while (opMode.opModeIsActive() && distance > allowedDistanceError) {
            distanceToX = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToY = targetY - globalPositionUpdate.returnYCoordinate();

            //uses right triange trig to figure out what ange the robot needs to move at
            //maybe could integrate Nir's holonomic odom math into?
            double moveAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY));

            //figures out what power to set the motors to so we can move at this angle
            double movementX = calculateX(moveAngle, power);
            double movementY = calculateY(moveAngle, power);

            double angleCorrection = orientation - globalPositionUpdate.returnOrientation();

        }
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
