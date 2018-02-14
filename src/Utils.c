/*
	Author: Jayden Chan
	Date Created: Jan 12 2018
	Last Modified: Feb 13 2018
	Details: Utility class to support PIDController and other files
*/

// Returns the sign of the input, and 0 if the input is 0
float sign(float input) {
    return input == 0 ? 0 : (abs(input) / input);
}

// Clamps the input value between +clamp and -clamp
float clamp(float input, float clamp) {
	return abs(input) > clamp ? clamp * sign(input) : input;
}
