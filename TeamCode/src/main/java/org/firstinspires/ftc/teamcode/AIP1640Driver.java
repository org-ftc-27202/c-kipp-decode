package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class AIP1640Driver {
    private DigitalChannel din;
    private DigitalChannel sclk;

    // Pre-allocate the buffer once to avoid memory churn
    private final byte[] displayBuffer = new byte[16];

    // Command Constants
    private final int CMD_DATA_SET = 0x40; // Fixed address mode
    private final int CMD_ADDR_SET = 0xC0; // Start address 00H
    private final int CMD_DISP_CTRL = 0x88; // Display ON, brightness mid

    public AIP1640Driver(DigitalChannel din, DigitalChannel sclk) {
        this.din = din;
        this.sclk = sclk;

        this.din.setMode(DigitalChannel.Mode.OUTPUT);
        this.sclk.setMode(DigitalChannel.Mode.OUTPUT);

        // Initial Idle State
        this.din.setState(true);
        this.sclk.setState(true);
    }

    private void start() {
        din.setState(true);
        sclk.setState(true);
        din.setState(false); // DIN drops while SCLK is high
    }

    private void stop() {
        sclk.setState(false);
        din.setState(false);
        sclk.setState(true);
        din.setState(true); // DIN rises while SCLK is high
    }

    private void writeByte(int b) {
        for (int i = 0; i < 8; i++) {
            sclk.setState(false);
            // Set Data
            din.setState((b & (1 << i)) != 0);
            // Let signal stabilize (The SDK call itself is a natural delay)
            sclk.setState(true);
        }
    }

    /**
     * Updates the 128 LEDs.
     * @param buffer Byte array of length 16 (16 bytes * 8 bits = 128 LEDs)
     */
    public void updateDisplay(byte[] buffer) {
        // 1. Set Data Mode
        start();
        writeByte(CMD_DATA_SET);
        stop();

        // 2. Set Address and Send 16 bytes of data
        start();
        writeByte(CMD_ADDR_SET);
        for (int i = 0; i < 16; i++) {
            writeByte(buffer[i]);
        }
        stop();

        // 3. Display Control (Brightness/ON)
        start();
        writeByte(CMD_DISP_CTRL);
        stop();
    }

    public void displayColumnGroups(int mode) {
        // Start with a clean slate
        java.util.Arrays.fill(displayBuffer, (byte)0x00);
        byte ON = (byte)0xFF;

        if (mode >= 4) {
            java.util.Arrays.fill(displayBuffer, ON);
        }
        else {
            // Standard grouped modes
            switch (mode) {
                case 3:
                    displayBuffer[12] = ON;
                    displayBuffer[13] = ON;
                    displayBuffer[14] = ON;
                    // fall through
                case 2:
                    displayBuffer[7] = ON;
                    displayBuffer[8] = ON;
                    displayBuffer[9] = ON;
                    // fall through
                case 1:
                    displayBuffer[2] = ON;
                    displayBuffer[3] = ON;
                    displayBuffer[4] = ON;
                    break;
            }
        }

        updateDisplay(displayBuffer);
    }
}