package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class auto_03_FarFromGoal_base extends NextFTCOpMode {
    public auto_03_FarFromGoal_base() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Catapult.INSTANCE, Camera.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private final ElapsedTime opModeTimer = new ElapsedTime();
    private boolean telemetryOnFlag;
    MecanumDrive drive;
    Command driveCommand;
    private Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));
    private Pose2d getPatternPose = new Pose2d(startPose.position.x, startPose.position.y + 24, startPose.heading.toDouble());
    private Pose2d LeavePose = new Pose2d(44, startPose.position.y - 12, startPose.heading.toDouble());
    @Override
    public void onInit() {
        telemetryOnFlag = true;
//        telemetryOnFlag = false;
        opModeTimer.reset();

        Camera.INSTANCE.mapCameraHardware(hardwareMap);
        Intake.INSTANCE.mapIntakeStopperHardware(hardwareMap);

        if (Config.allianceColor == Config.AllianceColors.BLUE) {
            startPose = new Pose2d(startPose.position.x, startPose.position.y * -1.0, startPose.heading.inverse().toDouble());
            getPatternPose = new Pose2d(getPatternPose.position.x, getPatternPose.position.y * -1.0, getPatternPose.heading.inverse().toDouble());
            LeavePose = new Pose2d(LeavePose.position.x, LeavePose.position.y * -1.0, LeavePose.heading.inverse().toDouble());
        }

        drive = new MecanumDrive(hardwareMap, startPose);
        driveCommand = drive.commandBuilder(startPose)
                // Get Motif
                .strafeToConstantHeading(getPatternPose.position)
                .stopAndAdd(Camera.INSTANCE.capturePattern)

                // Leave Pose
                .strafeToConstantHeading(LeavePose.position)
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        opModeTimer.reset();
        driveCommand.schedule();
    }

    @Override
    public void onUpdate() {
        Intake.INSTANCE.CountBalls();

        if (telemetryOnFlag) {
            telemetry.addData("run #", 1);
            telemetry.addData("alliance", Config.allianceColor.toString());
            telemetry.addData("pattern", Config.motifPattern.toString());
            telemetry.addData("intake (power)", "%.0f", Intake.INSTANCE.getPower());
            telemetry.addData("balls", "%d", Intake.INSTANCE.ballCounter);
            telemetry.addData("catapults (pos)", "01: %.0f | 02: %.0f | 03: %.0f", Catapult.INSTANCE.getPosition01(), Catapult.INSTANCE.getPosition02(), Catapult.INSTANCE.getPosition03());
            telemetry.addData("catapults (pattern)", "%s%s%s", Config.catapult01Color.toString().charAt(0), Config.catapult02Color.toString().charAt(0), Config.catapult03Color.toString().charAt(0));
            telemetry.addData("Timer", "%.1f", opModeTimer.seconds());
            telemetry.update();
        }
    }
}