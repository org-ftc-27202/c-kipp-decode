package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class auto_01_NearFromGoal_base extends NextFTCOpMode {
    public auto_01_NearFromGoal_base() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Catapult.INSTANCE, Camera.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private final ElapsedTime opModeTimer = new ElapsedTime();
    private boolean telemetryOnFlag;

    private final Pose2d startPose = new Pose2d(54, 72, Math.toRadians(90));
    private final Pose2d getPatternPose = new Pose2d(20, 24, Math.toRadians(90));
    private final Pose2d launchNear1Pose = new Pose2d(getPatternPose.position.x, getPatternPose.position.y, Math.toRadians(48));
    private final Pose2d launchNear2Pose = new Pose2d(22, 26, Math.toRadians(48));
    private final Pose2d launchNear3Pose = new Pose2d(launchNear2Pose.position.x, launchNear2Pose.position.y, launchNear2Pose.heading.toDouble());
    private final Pose2d spike2StartPose = new Pose2d(42, -8, Math.toRadians(0));
    private final Pose2d spike2EndPose = new Pose2d(spike2StartPose.position.x + 14, spike2StartPose.position.y, spike2StartPose.heading.toDouble());
    private final Pose2d spike1StartPose = new Pose2d(spike2StartPose.position.x - 6, 26, spike2StartPose.heading.toDouble());
    private final Pose2d spike1EndPose = new Pose2d(spike1StartPose.position.x + 14, spike1StartPose.position.y, spike1StartPose.heading.toDouble());
    private final Pose2d LeavePose = new Pose2d(50, 8, Math.toRadians(0));

    MecanumDrive drive;
    Command driveCommand;
    @Override
    public void onInit() {
        telemetryOnFlag = true;
//        telemetryOnFlag = false;

        Camera.INSTANCE.mapCameraHardware(hardwareMap);
        Intake.INSTANCE.mapIntakeStopperHardware(hardwareMap);

        drive = new MecanumDrive(hardwareMap, startPose);
        driveCommand = drive.commandBuilder(startPose)
                // Get Motif and Launch Preloads
                .stopAndAdd(new ParallelGroup(
                        Intake.INSTANCE.initIntakeStopper,
                        Intake.INSTANCE.wiperToLaunchPosition))
                .strafeToConstantHeading(getPatternPose.position)
                .stopAndAdd(Camera.INSTANCE.capturePattern)
                .turnTo(launchNear1Pose.heading)
                .stopAndAdd(Camera.INSTANCE.getCatapultArtifactColors)
                .stopAndAdd(Catapult.INSTANCE.LaunchByPattern)

                // Grab balls from Spike 2 and launch
                .strafeToLinearHeading(spike2StartPose.position, spike2StartPose.heading)
                .stopAndAdd(new ParallelGroup(
                        Intake.INSTANCE.initIntakeStopper,
                        Intake.INSTANCE.Inwards))
                .strafeToConstantHeading(spike2EndPose.position, new TranslationalVelConstraint(4))
                .stopAndAdd(new SequentialGroup(
                        new Delay(1.000),
                        Intake.INSTANCE.Stop))
                .strafeToLinearHeading(launchNear2Pose.position, launchNear2Pose.heading)
                .stopAndAdd(Camera.INSTANCE.getCatapultArtifactColors)
                .stopAndAdd(Catapult.INSTANCE.LaunchByPattern)

                // Grab balls from Spike 1 and launch
                .strafeToLinearHeading(spike1StartPose.position, spike1StartPose.heading)
                .stopAndAdd(new ParallelGroup(
                        Intake.INSTANCE.initIntakeStopper,
                        Intake.INSTANCE.Inwards))
                .strafeToConstantHeading(spike1EndPose.position, new TranslationalVelConstraint(4))
                .stopAndAdd(new SequentialGroup(
                        new Delay(1.000),
                        Intake.INSTANCE.Stop))
                .strafeToLinearHeading(launchNear3Pose.position, launchNear3Pose.heading)
                .stopAndAdd(Camera.INSTANCE.getCatapultArtifactColors)
                .stopAndAdd(Catapult.INSTANCE.LaunchByPattern)

                // Leave Pose
                .strafeToLinearHeading(LeavePose.position, LeavePose.heading)
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