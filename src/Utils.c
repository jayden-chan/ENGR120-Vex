/*
    Author: Jayden Chan
    Date Created: Jan 12 2018
    Details: Utility class to support PIDController and other files
*/

/**
 * Returns the sign of the input. If the input
 * is positive, its sign is (+1), if it's
 * negative, its sign is (-1). If it is zero,
 * its sign is (0).
 *
 * @param input The value to get the sign of.
 * @return The sign of the input.
 */
float sign(float input) {
    return input == 0 ? 0 : (abs(input) / input);
}

/**
 * Clamps the input value between the specified
 * limit.
 *
 * @param input The input value to clamp.
 * @param clamp The min and max value to clamp at.
 * @return The clamped value of the input.
 */
float clamp(float input, float clamp) {
    return abs(input) > clamp ? clamp * sign(input) : input;
}

/**
 * Clamps the input value between the specified
 * minimum and maximum.
 *
 * @param input The input value to clamp.
 * @param min   The lower bound of the clamp.
 * @param max   The upper bound of the clamp.
 * @return The clamped value of the input.
 */
float clamp2(float input, float min, float max) {
    if(input > max) {
        return max;
    }
    else if(input < min) {
        return min;
    }
    else {
        return input;
    }
}
