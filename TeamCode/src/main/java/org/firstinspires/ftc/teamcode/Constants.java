package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15.5)
//            .forwardZeroPowerAcceleration(200)
//            .lateralZeroPowerAcceleration(170)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0.0, 0.0))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.0, 0, 0.0, 0.0))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.1,0.06,0.0))
//            .centripetalScaling(0.00065)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(
        0.99,
        100,
        1,
        1
    );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(232.75551065444398)
            .yVelocity(190.61872750843932)
            ;

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .rightFrontEncoderDirection(Encoder.REVERSE)
            .rightRearEncoderDirection(Encoder.REVERSE)
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .robotWidth(15.039)
            .robotLength(10.394)
            .forwardTicksToInches(0.005948649027246637)
            .strafeTicksToInches(0.0061878359614794595)
            .turnTicksToInches(0.005974064522692344);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}