package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.purepursuit.Path2d;
import org.firstinspires.ftc.teamcode.purepursuit.Persuit;
import org.firstinspires.ftc.teamcode.purepursuit.Pose2d;

@TeleOp(name = "Robot Persuit")
public class RobotPersuit extends LinearOpMode {
    private SandsRobot robot;
    public float followerSpeed = 2.5f;
    public float lookaheadDistance = 6;
    //FtcDashboard dashboard;
    Path2d path;
    //PathFollower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        //dashboard = FtcDashboard.getInstance();
        //dashboard.setTelemetryTransmissionInterval(25);
        //TelemetryPacket packet = new TelemetryPacket();
        //Canvas canvas = packet.fieldOverlay();

        path = new Path2d(); //gets a default path
        Pose2d initialPose = new Pose2d(0,0,0);

        //canvas.clear();
        //dashboard.sendTelemetryPacket(packet);
        //path.draw(canvas);
        //dashboard.sendTelemetryPacket(packet);

        waitForStart();

        robot = new SandsRobot(hardwareMap, telemetry, this, initialPose);

        while(opModeIsActive()){
            robot.globalPosition.globalCoordinatePositionUpdate();
            // get the point where we are heading next
           /* float[] lookahead = Persuit.getLookaheadPoint(path, robot.pose.getX(), robot.pose.getY(), lookaheadDistance);

            if (lookahead != null) {
                // calculate the distance to the lookahead point
                double deltaX = lookahead[0] - robot.pose.getX();
                double deltaY = lookahead[1] - robot.pose.getY();
                float heading = (float) Math.atan2(deltaY, deltaX);
                // Pythagorean theorem to calculate the direct line (hypotenuse)
                float distance = 2 * (float) Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
                canvas.setStroke("yellow");
                canvas.strokeCircle(lookahead[0], lookahead[1], 2);
            }
            else {
                packet.put("Message", "Can't acquire path with lookahead distance:" + lookaheadDistance);
            }
*/
            double controlDrive = -gamepad1.left_stick_y;
            double controlStrafe = gamepad1.left_stick_x;
            double controlRotate = gamepad1.right_stick_x;

            //setting power levels based on drive controls
            double leftFrontPower = controlDrive + controlStrafe + controlRotate;
            double rightFrontPower = controlDrive - controlStrafe - controlRotate;
            double leftRearPower = controlDrive - controlStrafe + controlRotate;
            double rightRearPower = controlDrive + controlStrafe - controlRotate;

            robot.setPowerAll(rightFrontPower, rightRearPower, leftFrontPower, leftRearPower);

            //robot.draw(canvas);

            telemetry.addData("x", robot.pose.getX());
            telemetry.addData("y", robot.pose.getY());
            telemetry.addData("ODO heading", robot.pose.getHeading());
            telemetry.addData("IMU heading", robot.getHeading());
            telemetry.update();

            /*packet.put("x", robot.pose.getX());
            packet.put("y", robot.pose.getY());
            packet.put("heading", robot.pose.getHeading());
            dashboard.sendTelemetryPacket(packet);
            sleep(50);*/
        }
        //canvas.clear();
        //dashboard.sendTelemetryPacket(packet);

    }
}
