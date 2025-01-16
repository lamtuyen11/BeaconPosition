package org.altbeacon.beaconreference

import kotlin.math.*
import kotlin.random.Random

data class Particle(var x: Double, var y: Double, var weight: Double)

fun initializeParticles(numParticles: Int, areaWidth: Double, areaHeight: Double): List<Particle> {
    return List(numParticles) {
        Particle(
            x = Random.nextDouble(0.0, areaWidth),
            y = Random.nextDouble(0.0, areaHeight),
            weight = 1.0 / numParticles
        )
    }
}

fun predictParticles(particles: List<Particle>, movementNoise: Double) {
    particles.forEach { particle ->
        particle.x += Random.nextDouble(-movementNoise, movementNoise)
        particle.y += Random.nextDouble(-movementNoise, movementNoise)
    }
}

fun updateParticleWeights(particles: List<Particle>, beacons: List<BeaconPosition>) {
    particles.forEach { particle ->
        var weight = 1.0
        for (beacon in beacons) {
            val dx = particle.x - beacon.x
            val dy = particle.y - beacon.y
            val predictedDistance = sqrt(dx.pow(2) + dy.pow(2))
            val error = abs(predictedDistance - beacon.distance)
            weight *= exp(-error) // Exponential decay of error
        }
        particle.weight = weight
    }

    // Normalize weights
    val totalWeight = particles.sumOf { it.weight }
    particles.forEach { it.weight /= totalWeight }
}

fun resampleParticles(particles: List<Particle>): List<Particle> {
    val cumulativeWeights = particles.runningFold(0.0) { acc, particle -> acc + particle.weight }
    val resampledParticles = mutableListOf<Particle>()
    repeat(particles.size) {
        val randomValue = Random.nextDouble()
        val index = cumulativeWeights.indexOfFirst { it >= randomValue }
        resampledParticles.add(particles[index - 1].copy(weight = 1.0 / particles.size))
    }
    return resampledParticles
}

fun estimatePosition(particles: List<Particle>): Pair<Double, Double> {
    val avgX = particles.sumOf { it.x * it.weight }
    val avgY = particles.sumOf { it.y * it.weight }
    return Pair(avgX, avgY)
}

//// Main function that integrates particle filtering
//fun applyParticleFiltering(beacons: List<BeaconPosition>, numParticles: Int = 1000, iterations: Int = 10): Pair<Double, Double> {
//    val areaWidth = 100.0 // Define the area size (in meters or other units)
//    val areaHeight = 100.0
//
//    var particles = initializeParticles(numParticles, areaWidth, areaHeight)
//
//    repeat(iterations) {
//        predictParticles(particles, movementNoise = 0.5)
//        updateParticleWeights(particles, beacons)
//        particles = resampleParticles(particles)
//    }
//
//    return estimatePosition(particles)
//}
fun applyParticleFiltering(
    beacons: List<BeaconPosition>,
    numParticles: Int = 1000,
    iterations: Int = 10,
    initialEstimate: Pair<Double, Double>
): Pair<Double, Double> {
    val (initialX, initialY) = initialEstimate

    // Initialize particles around the initial estimate
    var particles = initializeParticlesAroundEstimate(numParticles, initialX, initialY)

    repeat(iterations) {
        predictParticles(particles, movementNoise = 0.5)
        updateParticleWeights(particles, beacons)
        particles = resampleParticles(particles)
    }

    return estimatePosition(particles)
}

// Helper function to initialize particles around an initial estimate
fun initializeParticlesAroundEstimate(
    numParticles: Int,
    initialX: Double,
    initialY: Double,
    spread: Double = 2.0 // Controls how widely the particles are spread
): List<Particle> {
    return List(numParticles) {
        Particle(
            x = initialX + Random.nextDouble(-spread, spread),
            y = initialY + Random.nextDouble(-spread, spread),
            weight = 1.0 / numParticles
        )
    }
}
class KalmanFilter(var x: Double, var y: Double, var p: Double = 1.0) {
    private val r = 0.5 // Measurement noise
    private val q = 0.1 // Process noise

    fun update(measurementX: Double, measurementY: Double) {
        // Update step
        val k = p / (p + r) // Kalman gain
        x = x + k * (measurementX - x)
        y = y + k * (measurementY - y)
        p = (1 - k) * p + q
    }

    fun getState(): Pair<Double, Double> {
        return Pair(x, y)
    }
}

