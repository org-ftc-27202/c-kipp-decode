package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class Config {
    public enum AllianceColors {RED, BLUE}
    public static AllianceColors allianceColor = AllianceColors.RED;
    public enum GoalOptions {NEAR, FAR}
    public static GoalOptions goalOption = GoalOptions.NEAR;
    public enum MotifPatterns {GPP, PGP, PPG}
    public static MotifPatterns motifPattern = MotifPatterns.GPP;
    public enum Colors {PURPLE, GREEN}
    public static Colors catapult01Color = Colors.GREEN;
    public static Colors catapult02Color = Colors.PURPLE;
    public static Colors catapult03Color = Colors.PURPLE;
    public static Pose2d localizePose = new Pose2d(112, 138, Math.toRadians(90));
    public static Pose2d autoEndPose = new Pose2d(112, 138, Math.toRadians(90));
    public static boolean isDriverControlled = true;
    public static boolean isTeleOpStartButtonPressed = false;
    public static boolean isGoalTagDetected = false;
    public static int cameraLeftTagX, cameraLeftTagY, cameraRightTagX, cameraRightTagY;
    public enum opModeOptions {AUTO, TELEOP}
    public static opModeOptions activeOpMode = opModeOptions.TELEOP;
}
