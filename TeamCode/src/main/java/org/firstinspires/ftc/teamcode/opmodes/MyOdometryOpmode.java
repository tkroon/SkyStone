package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "My Odometry Opmode")
public class MyOdometryOpmode extends LinearOpMode {
    private SandsRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SandsRobot(hardwareMap, telemetry, this);
        waitForStart();

        while(opModeIsActive()){
            double left = (-gamepad1.left_stick_y + gamepad1.left_stick_x);
            double right = (-gamepad1.left_stick_y - gamepad1.left_stick_x);
            robot.setPowerAll(right,right,left,left);

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", robot.globalPosition.getXCoordinateInches());
            telemetry.addData("Y Position", robot.globalPosition.getYCoordinateInches());
            telemetry.addData("Orientation (Degrees)", robot.globalPosition.getOrientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", robot.positionThread.isAlive());
            telemetry.update();
        }
        robot.stop();
    }
}
