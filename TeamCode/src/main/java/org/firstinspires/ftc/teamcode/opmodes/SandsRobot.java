package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePositionTask;
import org.firstinspires.ftc.teamcode.odometry.Utils;
import org.firstinspires.ftc.teamcode.purepursuit.Pose2d;

import java.util.Arrays;
import java.util.List;

public class SandsRobot {
    public final double COUNTS_PER_INCH = 306.381642027851;
    final int POSITION_THREAD_DELAY = 75;

    // Robot constructor creates robot object and sets up all the actuators and sensors
    public SandsRobot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, Pose2d pose) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.pose = pose;

        leftFront = hardwareMap.get(DcMotorEx.class, "motor0");
        leftRear = hardwareMap.get(DcMotorEx.class, "motor2");
        rightRear = hardwareMap.get(DcMotorEx.class, "motor3");
        rightFront = hardwareMap.get(DcMotorEx.class, "motor1");

        verticalLeft = hardwareMap.get(DcMotorEx.class, "motor3B");
        verticalRight = hardwareMap.get(DcMotorEx.class, "motor2B");
        horizontal = hardwareMap.get(DcMotorEx.class, "motor1B");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront, verticalLeft, verticalRight, horizontal);

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //THIS IS WHERE THE ROBOT'S POSITION BEGINS BEING TRACKED BY A THREAD
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPosition = new OdometryGlobalCoordinatePositionTask(this);
        //positionThread = new Thread(globalPosition);
        //positionThread.start();

        // initialize hardware and data
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        readSensors();
        sendTelemetry();
        initImu();
    }

    public SandsRobot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        this(hardwareMap, telemetry, opMode, new Pose2d(0,0,0));
    }

    // Sends messages and values to bottom of driver's screen
    void sendTelemetry() {
        telemetry.update();
    }

    /** **********************************************************
     * Reads and sets sensor values on robot for OpMode to
     **********************************************************/
    void readSensors() {

    }

    public void draw(Canvas canvas) {
        canvas.setStroke("blue");
        canvas.strokeCircle(globalPosition.getXCoordinateInches(), globalPosition.getYCoordinateInches(),2);
    }

    /*public void finalize() {
        globalPosition.stop();
    }*/

    public void stop() {
        //Stop the robot
        setPowerAll(0, 0, 0, 0);
    }

    /**************************
     * Navigation to a position
     ****************************/
    public void goToPosition(double targetX, double targetY, double robotPower, double desiredRobotOrientation, Double allowableDistanceError) {
        double targetXPostion = targetX * COUNTS_PER_INCH;
        double targetYPostion = targetY * COUNTS_PER_INCH;
        double distanceToXTarget = targetXPostion - globalPosition.getXCoordinate();
        double distanceToYTarget = targetYPostion - globalPosition.getYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while(opMode.opModeIsActive() && distance > allowableDistanceError){
            distanceToXTarget = targetXPostion - globalPosition.getXCoordinate();
            distanceToYTarget = targetYPostion - globalPosition.getYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = Utils.calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = Utils.calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = desiredRobotOrientation - globalPosition.getOrientation();
        }
    }

    public void setPowerAll(double rf, double rb, double lf, double lb){
        rightFront.setPower(rf);
        rightRear.setPower(rb);
        leftFront.setPower(lf);
        leftRear.setPower(lb);
    }

    // Initialize the imu within the expansion hub
    private void initImu() {
        BNO055IMU.Parameters imuParameters;
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // vertical IMU - upward (normal to the floor) using a command like the following:
        // Utils.remapAxes(imu, AxesOrder.XYZ, Utils.AxesSigns.NPN);
        // Initialize IMU.
        imu.initialize(imuParameters);
    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    public double getHeading() {
        Orientation angles;
        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // Variable Definitions for Robot
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    public OdometryGlobalCoordinatePositionTask globalPosition;
    protected Thread positionThread;
    protected DcMotorEx leftFront, leftRear, rightRear, rightFront;
    // encoders
    public DcMotorEx verticalLeft, verticalRight, horizontal;
    List<DcMotorEx> motors;
    private BNO055IMU imu;
    private LinearOpMode opMode;
    public Pose2d pose;
}
