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
                flagGetCatapultArtifactColors = false;
                HuskyLens.Block[] huskyLensRightBlocks = huskyLensRight.blocks();
                HuskyLens.Block[] huskyLensLeftBlocks = huskyLensLeft.blocks();

                Config.catapult01Color = Config.Colors.PURPLE;
                Config.catapult02Color = Config.Colors.PURPLE;
                Config.catapult03Color = Config.Colors.PURPLE;

                // Right-Side Camera
                if (huskyLensRightBlocks != null && huskyLensRightBlocks.length > 0) {
                    int huskylensRightBlockLength = huskyLensRightBlocks.length;
                    int minX, maxX, blockMinIndex, blockMaxIndex;

                    // Right-side Camera to get Catapult01 and Catapult02 colors
                    minX = huskyLensRightBlocks[0].x;
                    maxX = huskyLensRightBlocks[0].x;
                    blockMinIndex = 0;
                    blockMaxIndex = 0;
                    for (int i = 0; i < huskylensRightBlockLength; i++) {
                        if (huskyLensRightBlocks[i].x < minX) {
                            minX = huskyLensRightBlocks[i].x;
                            blockMinIndex = i;
                        }
                        if (huskyLensRightBlocks[i].x > maxX) {
                            maxX = huskyLensRightBlocks[i].x;
                            blockMaxIndex = i;
                        }
                    }

                    if (huskylensRightBlockLength == 2) {
                        if (huskyLensRightBlocks[blockMinIndex].id == 2) {
                            Config.catapult02Color = Config.Colors.GREEN;
                        }
                        if (huskyLensRightBlocks[blockMaxIndex].id == 2) {
                            Config.catapult01Color = Config.Colors.GREEN;
                        }
                    }
                    else if (huskylensRightBlockLength == 1) {
                        if (huskyLensRightBlocks[0].id == 2) {
                            if (huskyLensRightBlocks[0].x < 140)  // huskylens's center position is (160, 120)
                                Config.catapult02Color = Config.Colors.GREEN;
                            else Config.catapult01Color = Config.Colors.GREEN;
                        }
                    }
                }

                // Left-Side Camera
                if (huskyLensLeftBlocks != null && huskyLensLeftBlocks.length > 0) {
                    int huskylensLeftBlockLength = huskyLensLeftBlocks.length;
                    int minX, maxX, blockMinIndex, blockMaxIndex;

                    // Right-side Camera to get Catapult01 and Catapult02 colors
                    minX = huskyLensLeftBlocks[0].x;
                    maxX = huskyLensLeftBlocks[0].x;
                    blockMinIndex = 0;
                    blockMaxIndex = 0;
                    for (int i = 0; i < huskylensLeftBlockLength; i++) {
                        if (huskyLensLeftBlocks[i].x < minX) {
                            minX = huskyLensLeftBlocks[i].x;
                            blockMinIndex = i;
                        }
                        if (huskyLensLeftBlocks[i].x > maxX) {
                            maxX = huskyLensLeftBlocks[i].x;
                            blockMaxIndex = i;
                        }
                    }

                    if (huskylensLeftBlockLength == 2) {
                        if (huskyLensLeftBlocks[blockMinIndex].id == 2) {
                            Config.catapult03Color = Config.Colors.GREEN;
                        }
                        if (huskyLensLeftBlocks[blockMaxIndex].id == 2) {
                            Config.catapult02Color = Config.Colors.GREEN;
                        }
                    }
                    else if (huskylensLeftBlockLength == 1) {
                        if (huskyLensLeftBlocks[0].id == 2) {
                            if (huskyLensLeftBlocks[0].x < 140)  // huskylens's center position is (160, 120)
                                Config.catapult03Color = Config.Colors.GREEN;
                            else Config.catapult02Color = Config.Colors.GREEN;
                        }
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
