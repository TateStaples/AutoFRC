package Utilities.Automation

class KalmanFilter(var state: DoubleArray, val transformationMatrix: Array<DoubleArray>, val variance: DoubleArray) {

}

/*
Example Code:
- create predicted location based on movement
- get distribution based on movement
- actual = guessed + measurement_certainty * offset

 */

/*
Sources of Uncertainty:
 initial variances
 process variance
 measurement variance
 */
