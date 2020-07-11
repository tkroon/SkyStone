package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.purepursuit.Path2d;
import org.firstinspires.ftc.teamcode.purepursuit.PathFollower;
import org.firstinspires.ftc.teamcode.purepursuit.Persuit;
import org.firstinspires.ftc.teamcode.purepursuit.Pose2d;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "TestPersuit")
public class TestPersuit extends LinearOpMode {
    //private SandsRobot robot;
    public static float followerSpeed = 2.5f;
    public static float lookaheadDistance = 6;
    FtcDashboard dashboard;
    Pose2d currentPose;
    Path2d path;
    PathFollower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        //robot = new SandsRobot(hardwareMap, telemetry, this);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        path = new Path2d(); //gets a default path
        path.draw(fieldOverlay);

        waitForStart();

        float[] firstPointCoordinates = path.get(0);
        follower = new PathFollower(firstPointCoordinates[0], firstPointCoordinates[1], followerSpeed, 0);

        while(opModeIsActive()){
            currentPose = follower.getFollowerPosition();
            // get the point where we are heading next
            float[] lookahead = Persuit.getLookaheadPoint(path, currentPose.getX(), currentPose.getY(), lookaheadDistance);
            // calculate the distance to the lookahead point
            double deltaX = lookahead[0] - currentPose.getX();
            double deltaY = lookahead[1] - currentPose.getY();
            float heading = (float)Math.atan2(deltaY,deltaX);
            // Pythagorean theorem to calculate the direct line (hypotenuse)
            float distance = 2 * (float) Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

            // only follow when distance is farther than 1 inch
            if (distance > 1) {
                follower.moveFollowerTowardsPoint(lookahead[0], lookahead[1], heading);
                follower.draw(fieldOverlay);
                currentPose = follower.getFollowerPosition();
            }
            
            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", Math.toDegrees(currentPose.getHeading()));

            dashboard.sendTelemetryPacket(packet);
            sleep(100);
        }
    }
}
