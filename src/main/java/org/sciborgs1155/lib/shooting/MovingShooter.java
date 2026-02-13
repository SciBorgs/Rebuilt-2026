package org.sciborgs1155.lib.shooting;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.FieldConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class MovingShooter implements ShootingAlgorithm {

    // Adjust this to change the arc. (Meters)
    private static final double APEX_HEIGHT_ABOVE_TARGET = 2.0;
    
    private static final double G = 9.81;
    private static final double SHOOTER_HEIGHT = 0.6; // Vertical offset of  shooter exit

    @Override
    public Vector<N3> calculate(Translation3d robotPose, Vector<N2> robotVelocity) {
        Translation3d target = new Translation3d(BLUE_HUB.getX(), BLUE_HUB.getY(), HUB_HEIGHT.in(Meters));
        
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        double horizontalDist = Math.hypot(dx, dy);

        double zStart = robotPose.getZ() + SHOOTER_HEIGHT;
        double zTarget = target.getZ();

        double zApex = Math.max(zStart, zTarget) + APEX_HEIGHT_ABOVE_TARGET;

        double vZ = Math.sqrt(2 * G * (zApex - zStart));

        double timeToPeak = vZ / G;
        double timeToFall = Math.sqrt(2 * (zApex - zTarget) / G);
        double totalTime = timeToPeak + timeToFall;

        double vXY = horizontalDist / totalTime;

        double unitX = dx / horizontalDist;
        double unitY = dy / horizontalDist;

        return VecBuilder.fill(
            (unitX * vXY) - robotVelocity.get(0, 0),
            (unitY * vXY) - robotVelocity.get(1, 0),
            vZ
        );
    }

    @Override
    public double[] calculateToDoubleArray(Pose3d robotPose, ChassisSpeeds robotVelocity) {
        return calculate(robotPose.getTranslation(), 
               VecBuilder.fill(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)).getData();
    }
}