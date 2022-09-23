package frc.robot.leds.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.leds.LEDConstants;
import frc.robot.leds.LEDs;

public class AuroraLEDCommand extends CommandBase {
    /** Creates a new AuroraLEDCommand. */
    private final LEDs ledSubsystem;

    private final int NUM_LED_GROUPS = 6;
    private final int NUM_COLORS = 4;

    private final int[] boundaries = calcBoundaries();
    private final int mintGreenPinkBoundary = boundaries[0];
    private final int pinkLightBlueBoundary = boundaries[1];
    private final int lightBlueDarkBlueBoundary = boundaries[2];
    private final int darkBlueMintGreenBoundary = boundaries[3];
    // RGB
    private final int[] mintGreen = {30, 222, 32};
    private final int[] pink = {199, 68, 235};
    private final int[] lightBlue = {4, 255, 219};
    private final int[] darkBlue = {62, 0, 216};
    private final int gradientLength = 5;
    private final int[][] mintGreenPinkGradient = calcGradientColors(mintGreen, pink);
    private final int[][] pinkLightBlueGradient = calcGradientColors(pink, lightBlue);
    private final int[][] lightBlueDarkBlueGradient = calcGradientColors(lightBlue, darkBlue);
    private final int[][] darkBlueMintGreenGradient = calcGradientColors(darkBlue, mintGreen);
    private final int waitTime = 50;
    private int position = 0;
    private long startTime = System.currentTimeMillis();
    private int[][][] ledStates = new int[LEDConstants.LED_COUNT][LEDConstants.LED_COUNT][3];

    public AuroraLEDCommand(LEDs ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ledSubsystem);

        for (int position = 0; position < LEDConstants.LED_COUNT; position += 1) {
            int[][] state = new int[LEDConstants.LED_COUNT][3];
            for (int j = 0;
                    j < LEDConstants.LED_COUNT + 1;
                    j += (LEDConstants.LED_COUNT / NUM_LED_GROUPS)) {
                int i = 0;
                int offset = j + position;
                // int gradientOffset = j - position; // 0

                for (i = i + offset; i < (mintGreenPinkBoundary - gradientLength) + offset; i++) {
                    state[wrapValues(i)] = mintGreen;
                }
                for (; i < mintGreenPinkBoundary + offset; i++) {
                    // makes i (0 through gradientLength - 1) for selecting each color out of the
                    // gradient
                    int gradientPos = i - j - position - (mintGreenPinkBoundary - gradientLength);
                    state[wrapValues(i)] = mintGreenPinkGradient[gradientPos];
                }
                for (; i < (pinkLightBlueBoundary - gradientLength) + offset; i++) {
                    state[wrapValues(i)] = pink;
                }
                for (; i < pinkLightBlueBoundary + offset; i++) {
                    int gradientPos = i - j - position - (pinkLightBlueBoundary - gradientLength);
                    state[wrapValues(i)] = pinkLightBlueGradient[gradientPos];
                }
                for (; i < (lightBlueDarkBlueBoundary - gradientLength) + offset; i++) {
                    state[wrapValues(i)] = lightBlue;
                }
                for (; i < lightBlueDarkBlueBoundary + offset; i++) {
                    int gradientPos =
                            i - j - position - (lightBlueDarkBlueBoundary - gradientLength);
                    state[wrapValues(i)] = lightBlueDarkBlueGradient[gradientPos];
                }
                for (; i < (darkBlueMintGreenBoundary - gradientLength) + offset; i++) {
                    state[wrapValues(i)] = darkBlue;
                }
                for (; i < darkBlueMintGreenBoundary + offset; i++) {
                    int gradientPos =
                            i - j - position - (darkBlueMintGreenBoundary - gradientLength);
                    state[wrapValues(i)] = darkBlueMintGreenGradient[gradientPos];
                }
            }
            ledStates[position] = state;
        }
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    private int[] calcBoundaries() {
        int[] result = new int[NUM_COLORS];
        for (int i = 0; i < NUM_COLORS; i++) {
            result[i] = ((i + 1) * LEDConstants.LED_COUNT) / (NUM_COLORS * NUM_LED_GROUPS);
        }
        return result;
    }

    private int wrapValues(int num) {
        return num % LEDConstants.LED_COUNT;
    }

    private int[] getAvgValue(int[] color1, int[] color2) {
        return new int[] {
            (color1[0] + color2[0]) / 2, (color1[1] + color2[1]) / 2, (color1[2] + color2[2]) / 2
        };
    }

    private int[][] calcGradientColors(int[] color1, int[] color2) {
        int[][] gradientColors = new int[gradientLength][3];

        final double centerIndex = (gradientLength - 1.0) / 2.0;

        int[] leftColor = getAvgValue(color1, color2);
        int[] rightColor = getAvgValue(color1, color2);

        // [[], [], [], []]
        // [[], [], [], [], [], [], []]
        // if odd num of gradient leds set the middle to avg of both sides
        if (gradientLength % 2 != 0) gradientColors[(int) centerIndex] = leftColor;

        for (int i = 0; i < Math.floor(gradientLength / 2.0); i++) {
            rightColor = getAvgValue(rightColor, color2);
            leftColor = getAvgValue(color1, leftColor);
            int placementIndex = i + 1;
            gradientColors[(int) Math.floor(centerIndex + placementIndex)] = rightColor;
            gradientColors[(int) Math.ceil(centerIndex - placementIndex)] = leftColor;
        }

        return gradientColors;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (System.currentTimeMillis() - startTime >= waitTime) {

            for (int i = 0; i < LEDConstants.LED_COUNT; i += 1) {
                ledSubsystem.setRGB(
                        wrapValues(i),
                        ledStates[position][i][0],
                        ledStates[position][i][1],
                        ledStates[position][i][2]);
            }

            ledSubsystem.sendData();
            position = wrapValues(position + 1);
            startTime = System.currentTimeMillis();
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
