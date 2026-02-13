package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public abstract class auto_02_NearFromGoalOpenGate_base extends NextFTCOpMode {
    public auto_02_NearFromGoalOpenGate_base() {
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
    private Pose2d startPose = new Pose2d(54, 72, Math.toRadians(90));
    private Pose2d getPatternPose = new Pose2d(24, 30, Math.toRadians(90));
    private Pose2d launchNear1Pose = new Pose2d(getPatternPose.position.x, getPatternPose.position.y, Math.toRadians(50));
    private Pose2d launchNear2Pose = new Pose2d(30, 30, Math.toRadians(50));
    private Pose2d launchNear3Pose = new Pose2d(30, 30, Math.toRadians(50));
    private Pose2d spike2StartPose = new Pose2d(45, -2, Math.toRadians(0));
    private Pose2d spike2EndPose = new Pose2d(spike2StartPose.position.x + 20, spike2StartPose.position.y, spike2StartPose.heading.toDouble());
    private Pose2d spike2EndControlPose = new Pose2d(spike2EndPose.position.x -10, spike2EndPose.position.y, spike2EndPose.heading.toDouble());
    private Pose2d spike1StartPose = new Pose2d(spike2StartPose.position.x, 26, spike2StartPose.heading.toDouble());
    private Pose2d spike1EndPose = new Pose2d(spike1StartPose.position.x + 16, spike1StartPose.position.y, spike1StartPose.heading.toDouble());
    private Pose2d LeavePose = new Pose2d(60, 8, Math.toRadians(0));

    @Override
    public void onInit() {
        Config.activeOpMode = Config.opModeOptions.AUTO;

        telemetryOnFlag = true;
//        telemetryOnFlag = false;

        Camera.INSTANCE.mapCameraHardware(hardwareMap);
        Intake.INSTANCE.mapIntakeStopperHardware(hardwareMap);

        if (Config.allianceColor == Config.AllianceColors.BLUE) {
            startPose = new Pose2d(startPose.position.x, startPose.position.y * -1.0, startPose.heading.inverse().toDouble());
            getPatternPose = new Pose2d(getPatternPose.position.x, getPatternPose.position.y * -1.0, getPatternPose.heading.inverse().toDouble());
            launchNear1Pose = new Pose2d(launchNear1Pose.position.x, launchNear1Pose.position.y * -1.0, launchNear1Pose.heading.inverse().toDouble());
            launchNear2Pose = new Pose2d(launchNear2Pose.position.x, launchNear2Pose.position.y * -1.0, launchNear2Pose.heading.inverse().toDouble());
            launchNear3Pose = new Pose2d(launchNear3Pose.position.x, launchNear3Pose.position.y * -1.0, launchNear3Pose.heading.inverse().toDouble());
            spike2StartPose = new Pose2d(spike2StartPose.position.x, spike2StartPose.position.y * -1.0, spike2StartPose.heading.inverse().toDouble());
            spike2EndPose = new Pose2d(spike2EndPose.position.x, spike2EndPose.position.y * -1.0, spike2EndPose.heading.inverse().toDouble());
            spike2EndControlPose = new Pose2d(spike2EndControlPose.position.x, spike2EndControlPose.position.y * -1.0, spike2EndControlPose.heading.inverse().toDouble());
            spike1StartPose = new Pose2d(spike1StartPose.position.x, spike1StartPose.position.y * -1.0, spike1StartPose.heading.inverse().toDouble());
            spike1EndPose = new Pose2d(spike1EndPose.position.x, spike1EndPose.position.y * -1.0, spike1EndPose.heading.inverse().toDouble());
            LeavePose = new Pose2d(LeavePose.position.x, LeavePose.position.y * -1.0, LeavePose.heading.inverse().toDouble());
        }

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
                .stopAndAdd(Catapult.INSTANCE.LaunchInParallel)

                // Grab balls from Spike 2 and launch
                .strafeToLinearHeading(spike2StartPose.position, spike2StartPose.heading)
                .stopAndAdd(new ParallelGroup(
                        Intake.INSTANCE.initIntakeStopper,
                        Intake.INSTANCE.Inwards))
                .strafeToConstantHeading(spike2EndPose.position, new TranslationalVelConstraint(5))
                .strafeToConstantHeading(spike2EndControlPose.position)
                .strafeToLinearHeading(launchNear2Pose.position, launchNear2Pose.heading)
                .stopAndAdd(Intake.INSTANCE.Stop)
                .waitSeconds(0.750)
                .stopAndAdd(Camera.INSTANCE.getCatapultArtifactColors)
                .stopAndAdd(Catapult.INSTANCE.LaunchByPattern)

                // Grab balls from Spike 1 and launch
                .strafeToLinearHeading(spike1StartPose.position, spike1StartPose.heading)
                .stopAndAdd(new ParallelGroup(
                        Intake.INSTANCE.initIntakeStopper,
                        Intake.INSTANCE.Inwards))
                .strafeToConstantHeading(spike1EndPose.position, new TranslationalVelConstraint(5))
                .strafeToLinearHeading(launchNear3Pose.position, launchNear3Pose.heading)
                .stopAndAdd(Intake.INSTANCE.Stop)
                .waitSeconds(0.750)
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