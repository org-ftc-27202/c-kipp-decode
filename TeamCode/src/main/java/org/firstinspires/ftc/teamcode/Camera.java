package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;

public class Camera implements Subsystem {
    // HuskyLens https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#
    // Frame Width: 320
    // Frame Height: 240
    // Frame Center: 160, 120

    public static final Camera INSTANCE = new Camera();
    public Camera() { }
    private HuskyLens huskyLensFront, huskyLensLeft, huskyLensRight;
    private boolean flagCapturePatternComplete = false;
    private boolean flagAlignToGoalComplete = false;
    private boolean flagGetCatapultArtifactColors = false;
    public int deltaToCenterX = 0, deltaToCenterY = 0;
    public double turnPIDPower = 0.0, forwardPIDPower = 0.0;;

    public void mapCameraHardware(HardwareMap hardwareMap) {
        huskyLensFront = hardwareMap.get(HuskyLens.class, "huskylensFront");
        huskyLensFront.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        huskyLensFront.knock();

        huskyLensLeft = hardwareMap.get(HuskyLens.class, "huskylensLeft");
        huskyLensLeft.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        huskyLensLeft.knock();

        huskyLensRight = hardwareMap.get(HuskyLens.class, "huskylensRight");
        huskyLensRight.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        huskyLensRight.knock();
    }
    public Command capturePattern = new LambdaCommand("capturePattern")
            .setStart(() -> {
                flagCapturePatternComplete = false;
                int blockLength, blockIndex, minX, maxX;
                HuskyLens.Block[] blocks = huskyLensFront.blocks();
                blockLength = blocks.length;

                if (blockLength != 0) {
                    blockIndex = 0;
                    minX = blocks[0].x;
                    maxX = blocks[0].x;

                    // if RED alliance, then get the leftmost, otherwise, go rightmost
                    if (Config.allianceColor == Config.AllianceColors.RED) {
                        for (int i = 0; i < blockLength; i++) {
                            if (blocks[i].x < minX) {
                                minX = blocks[i].x;
                                blockIndex = i;
                            }
                        }
                    } else if (Config.allianceColor == Config.AllianceColors.BLUE) {
                        for (int i = 0; i < blockLength; i++) {
                            if (blocks[i].x > maxX) {
                                maxX = blocks[i].x;
                                blockIndex = i;
                            }
                        }
                    }

                    if (blocks[blockIndex].id == 1) {
                        Config.motifPattern = Config.MotifPatterns.GPP;
                    } else if (blocks[blockIndex].id == 2) {
                        Config.motifPattern = Config.MotifPatterns.PGP;
                    } else if (blocks[blockIndex].id == 3) {
                        Config.motifPattern = Config.MotifPatterns.PPG;
                    }
                }
                flagCapturePatternComplete = true;
                })
            .setUpdate(() -> {})
            .setIsDone(() -> flagCapturePatternComplete)
            .setStop(interrupted -> {})
            .setInterruptible(false)
            .requires(this);
    public Command alignToGoal_byTurn = new LambdaCommand("alignToGoal_byTurn")
            .setStart(() -> {
                flagAlignToGoalComplete = false;
                deltaToCenterX = 0;
                turnPIDPower = 0.0;
                forwardPIDPower = 0.0;
            })
            .setUpdate(() -> {
                int blockLength, tagX = 0;
                HuskyLens.Block[] blocksFront = huskyLensFront.blocks();
                blockLength = blocksFront.length;
                deltaToCenterX = 0;
                if (blockLength != 0) {
                    // if RED alliance, then get the leftmost, otherwise, go rightmost
                    if (Config.allianceColor == Config.AllianceColors.RED && blocksFront[0].id == 4
                            || Config.allianceColor == Config.AllianceColors.BLUE && blocksFront[0].id == 5) {
                        tagX = blocksFront[0].x;
                    }
                    deltaToCenterX = tagX - 160; // based on frame width of 320
                }
                if (deltaToCenterX > 8) {
                    Config.isDriverControlled = false;
                    turnPIDPower = 0.12;
                }
                else if (deltaToCenterX < -8) {
                    Config.isDriverControlled = false;
                    turnPIDPower = -0.12;
                }
                else {
                    flagAlignToGoalComplete = true;
                    Config.isDriverControlled = true;
                }
            })
            .setIsDone(() -> flagAlignToGoalComplete)
            .setStop(interrupted -> {})
            .setInterruptible(false)
            .requires(this);
    public Command alignToGoal_byForward = new LambdaCommand("alignToGoal_byForward")
            .setStart(() -> {
                flagAlignToGoalComplete = false;
                deltaToCenterY = 0;
                turnPIDPower = 0.0;
                forwardPIDPower = 0.0;
            })
            .setUpdate(() -> {
                int blockLength, tagY = 0;
                HuskyLens.Block[] blocksFront = huskyLensFront.blocks();
                blockLength = blocksFront.length;
                deltaToCenterX = 0;
                deltaToCenterY = 0;
                if (blockLength != 0) {
                    // if RED alliance, then get the leftmost, otherwise, go rightmost
                    if (Config.allianceColor == Config.AllianceColors.RED && blocksFront[0].id == 4
                            || Config.allianceColor == Config.AllianceColors.BLUE && blocksFront[0].id == 5) {
                        tagY = blocksFront[0].y;
                    }
                    deltaToCenterY = tagY - 58; // based on ideal height of 36
                }
                if (deltaToCenterY > 8) {
                    Config.isDriverControlled = false;
                    forwardPIDPower = 0.50;
                }
                else if (deltaToCenterY < -8) {
                    Config.isDriverControlled = false;
                    forwardPIDPower = -0.50;
                }
                else {
                    flagAlignToGoalComplete = true;
                    Config.isDriverControlled = true;
                }
            })
            .setIsDone(() -> flagAlignToGoalComplete)
            .setStop(interrupted -> {})
            .setInterruptible(false)
            .requires(this);
    public double getAngleTowardsGoalCenter() {
        deltaToCenterX = 0;
        int blockLength, tagX = 0;
        HuskyLens.Block[] blocksFront = huskyLensFront.blocks();
        blockLength = blocksFront.length;
        deltaToCenterX = 0;
        if (blockLength != 0) {
            // if RED alliance, then get the leftmost, otherwise, go rightmost
            if (Config.allianceColor == Config.AllianceColors.RED && blocksFront[0].id == 4
                    || Config.allianceColor == Config.AllianceColors.BLUE && blocksFront[0].id == 5) {
                tagX = blocksFront[0].x;
            }
            deltaToCenterX = tagX - 160; // based on frame width of 320
        }
        return deltaToCenterX * 0.02;  // multiplier factor to degrees
    }
    public Command getCatapultArtifactColors = new LambdaCommand("getCatapultArtifactColors")
            .setStart(() -> {
                int blockLength, blockIndex;
                flagGetCatapultArtifactColors = false;
                HuskyLens.Block[] huskyLensRightBlocks = huskyLensRight.blocks();
                HuskyLens.Block[] huskyLensLeftBlocks = huskyLensLeft.blocks();

                Config.catapult01Color = Config.Colors.PURPLE;
                Config.catapult02Color = Config.Colors.PURPLE;
                Config.catapult03Color = Config.Colors.PURPLE;

                // Right-Side Camera to get Catapult01 and Catapult02 color
                blockLength = huskyLensRightBlocks.length;
                for (blockIndex = 0; blockIndex < blockLength; blockIndex++) {
                    Config.camera01TagX = huskyLensRightBlocks[0].x;
                    Config.camera01TagY = huskyLensRightBlocks[0].y;

                    if (huskyLensRightBlocks[blockIndex].id == 1
                            && huskyLensRightBlocks[blockIndex].x >= 155
                            && huskyLensRightBlocks[blockIndex].x <= 200
                            && huskyLensRightBlocks[blockIndex].y >= 100
                            && huskyLensRightBlocks[blockIndex].y <= 140) {
                        Config.catapult01Color = Config.Colors.GREEN;
                    }
                    if (huskyLensRightBlocks[blockIndex].id == 1
                            && huskyLensRightBlocks[blockIndex].x >= 40
                            && huskyLensRightBlocks[blockIndex].x < 155
                            && huskyLensRightBlocks[blockIndex].y >= 100
                            && huskyLensRightBlocks[blockIndex].y <= 140) {
                        Config.catapult02Color = Config.Colors.GREEN;
                    }
                }

                // Left-Side Camera to get Catapult03 color
                blockLength = huskyLensLeftBlocks.length;
                for (blockIndex = 0; blockIndex < blockLength; blockIndex++) {
                    Config.camera02TagX = huskyLensLeftBlocks[0].x;
                    Config.camera02TagY = huskyLensLeftBlocks[0].y;

                    if (huskyLensLeftBlocks[blockIndex].id == 1
                            && huskyLensLeftBlocks[blockIndex].x >= 80
                            && huskyLensLeftBlocks[blockIndex].x <= 180
                            && huskyLensLeftBlocks[blockIndex].y >= 100
                            && huskyLensLeftBlocks[blockIndex].y <= 180) {
                        Config.catapult03Color = Config.Colors.GREEN;
                    }
                }
                flagGetCatapultArtifactColors = true;

            })
            .setUpdate(() -> {})
            .setIsDone(() -> {
                return flagGetCatapultArtifactColors;
            })
            .setStop(interrupted -> {})
            .setInterruptible(false)
            .requires(this);
}
