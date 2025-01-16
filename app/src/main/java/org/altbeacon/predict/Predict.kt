package org.altbeacon.predict

import org.altbeacon.beaconreference.BeaconPosition
import org.altbeacon.beaconreference.calculateDevicePosition
import org.altbeacon.beaconreference.generateSubsets
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

data class SensorData(val heading: Double, val speed: Double)

// Predict the next position based on current position, heading, and speed
fun predictNextPosition(
    currentPosition: Pair<Double, Double>,
    sensorData: SensorData,
    deltaTime: Double
): Pair<Double, Double> {
    val (x, y) = currentPosition
    val angle = sensorData.heading.toRadians()

    val dx = sensorData.speed * deltaTime * cos(angle)
    val dy = sensorData.speed * deltaTime * sin(angle)

    return Pair(x + dx, y + dy)
}
fun calculateAveragePositionWithPrediction(
    beacons: List<BeaconPosition>,
    currentPosition: Pair<Double, Double>,
    sensorData: SensorData,
    deltaTime: Double
): Pair<Double, Double>? {
    val predictedPosition = predictNextPosition(currentPosition, sensorData, deltaTime)

    // Calculate positions using subsets of beacons
    val subsets = generateSubsets(beacons)
    val positions = subsets.mapNotNull { calculateDevicePosition(it) }

    if (positions.isEmpty()) return null

    // Select the most consistent position with the predicted position
    val bestPosition = positions.minByOrNull { position ->
        calculateDistance(predictedPosition, position)
    }

    return bestPosition
}

// Helper function to calculate the distance between two points
fun calculateDistance(pos1: Pair<Double, Double>, pos2: Pair<Double, Double>): Double {
    val (x1, y1) = pos1
    val (x2, y2) = pos2
    return kotlin.math.sqrt((x1 - x2).pow(2) + (y1 - y2).pow(2))
}
fun Double.toRadians(): Double = this * Math.PI / 180.0


