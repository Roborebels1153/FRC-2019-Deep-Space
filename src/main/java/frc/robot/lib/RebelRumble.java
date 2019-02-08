package frc.robot.lib;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Provides fancy rumble pattern functionality
 */
public class RebelRumble {

    private GenericHID mController;

    private int mRunningPattern = -1;
    private long mPatternNextFrameMillis = -1;
    private int mPatternFrame = 0;

    private static final long WARNING_TOLERANCE = 250;
    private static final long FRAME_LENGTH = 250;

    private static final int[][][] FRAMES = {
        {
            {1, 0}, {0, 1}, {0, 0}
        }, {
            {0, 1}, {1, 0}, {0, 0}
        }, {
            {1, 1}, {0, 0}, {1, 1}, {0, 0}
        }
    };

    public static final int PATTERN_LEFT_TO_RIGHT = 0;
    public static final int PATTERN_RIGHT_TO_LEFT = 1;
    public static final int PATTERN_PULSE = 2;

    public RebelRumble(GenericHID controller) {
        mController = controller;
    }

    /**
     * Executes the different rumble pattern frames at the correct intervals,
     * must be called periodically or rumble patterns will fail to work
     */
    public void loop() {
        // Escape if no pattern is running
        if (mRunningPattern == -1) {
            return;
        }

        if (mPatternFrame == 0) {
            // Begin the pattern execution
            mPatternNextFrameMillis = System.currentTimeMillis() + FRAME_LENGTH;
            mPatternFrame++;
            setFrame(mPatternFrame);
        } else {
            long diff = System.currentTimeMillis() - mPatternNextFrameMillis;
            if (diff >= 0) {
                // Provide a friendly warning if we are updating the frame over 250ms late
                if (diff >= WARNING_TOLERANCE) {
                    System.err.println("RebelVibrate loop() not called often enough");
                }

                // Continue pattern exection if we have reached the next frame checkpoint (every 500ms)
                mPatternFrame++;
                setFrame(mPatternFrame);

                // Exit if we have reached the last frame in the pattern
                if (getPatternNumFrames(mRunningPattern) == mPatternFrame) {
                    mRunningPattern = -1;
                    mPatternNextFrameMillis = -1;
                    mPatternFrame = 0;
                } else {
                    // Set the next frame checkpoint
                    mPatternNextFrameMillis = System.currentTimeMillis() + FRAME_LENGTH;
                }
            }
        }
    }

    /**
     * Begins a specified pattern, immediately stopping and preceding any already
     * executing pattern
     * @param pattern The pattern to run
     */
    public void rumble(int pattern) {
        mPatternNextFrameMillis = -1;
        mPatternFrame = 0;
        mRunningPattern = pattern;
    }

    private void setRumble(double left, double right) {
        mController.setRumble(RumbleType.kLeftRumble, left);
        mController.setRumble(RumbleType.kRightRumble, right);
    }

    private void setFrame(int frameNum) {
        int[][] pattern = FRAMES[mRunningPattern];
        int[] frame = pattern[frameNum - 1];
        setRumble(frame[0], frame[1]);
    }

    private int getPatternNumFrames(int pattern) {
        return FRAMES[pattern].length;
    }
}
