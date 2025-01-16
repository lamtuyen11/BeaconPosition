package org.altbeacon.beaconreference

import android.app.AlertDialog
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.ListView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.Observer
import org.altbeacon.beacon.Beacon
import org.altbeacon.beacon.BeaconManager
import org.altbeacon.beacon.MonitorNotifier
import android.content.Intent
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import org.altbeacon.api.BeaconData
import org.altbeacon.api.sendBeaconDataToApi
import org.altbeacon.beacon.Identifier
import org.altbeacon.beacon.permissions.BeaconScanPermissionsActivity
import org.altbeacon.predict.SensorData
import org.altbeacon.predict.calculateAveragePositionWithPrediction
import org.apache.commons.math3.fitting.leastsquares.*
import org.apache.commons.math3.linear.*
import java.time.Instant
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.math.pow


class MainActivity : AppCompatActivity(), SensorEventListener {
    lateinit var beaconListView: ListView
    lateinit var beaconCountTextView: TextView
    lateinit var monitoringButton: Button
    lateinit var rangingButton: Button
    lateinit var beaconReferenceApplication: BeaconReferenceApplication
    var alertDialog: AlertDialog? = null
    private lateinit var positionView: PositionView
    private lateinit var sensorManager: SensorManager
    private var rotationSensor: Sensor? = null
    private var currentAzimuth: Float = 0f
    val distanceCache = mutableMapOf<Identifier, MutableList<Double>>()
    private var lastKnownPosition: Pair<Double, Double>? = null



    val beaconPositions1 = listOf(
//        Pair(4.2, 2.0),
//        Pair(-3.2, 6.0),
        Pair(0.0, 0.5),
        Pair(-3.2, 1.0),
        Pair(4.2, -2.0),
    )

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        beaconReferenceApplication = application as BeaconReferenceApplication
        positionView = findViewById(R.id.position_view)

        // Set up a Live Data observer for beacon data
        val regionViewModel = BeaconManager.getInstanceForApplication(this).getRegionViewModel(beaconReferenceApplication.region)
        // observer will be called each time the monitored regionState changes (inside vs. outside region)
        regionViewModel.regionState.observe(this, monitoringObserver)
        // observer will be called each time a new list of beacons is ranged (typically ~1 second in the foreground)
        regionViewModel.rangedBeacons.observe(this, rangingObserver)
        rangingButton = findViewById<Button>(R.id.rangingButton)
        monitoringButton = findViewById<Button>(R.id.monitoringButton)
        beaconListView = findViewById<ListView>(R.id.beaconList)
        beaconCountTextView = findViewById<TextView>(R.id.beaconCount)
        beaconCountTextView.text = "No beacons detected"
        beaconListView.adapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, arrayOf("--"))
        positionView.setBeacons(beaconPositions1)
        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

    }

    override fun onPause() {
        Log.d(TAG, "onPause")
        super.onPause()
        sensorManager.unregisterListener(this)
    }
    override fun onResume() {
        Log.d(TAG, "onResume")
        super.onResume()
        rotationSensor?.let { sensor ->
            sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_UI)
        } ?: Log.e(TAG, "Rotation vector sensor not available!")
        // You MUST make sure the following dynamic permissions are granted by the user to detect beacons
        //
        //    Manifest.permission.BLUETOOTH_SCAN
        //    Manifest.permission.BLUETOOTH_CONNECT
        //    Manifest.permission.ACCESS_FINE_LOCATION
        //    Manifest.permission.ACCESS_BACKGROUND_LOCATION // only needed to detect in background
        //
        // The code needed to get these permissions has become increasingly complex, so it is in
        // its own file so as not to clutter this file focussed on how to use the library.

        if (!BeaconScanPermissionsActivity.allPermissionsGranted(this,
                true)) {
            val intent = Intent(this, BeaconScanPermissionsActivity::class.java)
            intent.putExtra("backgroundAccessRequested", true)
            startActivity(intent)
        }
        else {
            // All permissions are granted now.  In the case where we are configured
            // to use a foreground service, we will not have been able to start scanning until
            // after permissions are graned.  So we will do so here.
            if (BeaconManager.getInstanceForApplication(this).monitoredRegions.size == 0) {
                (application as BeaconReferenceApplication).setupBeaconScanning()
            }
        }
    }

    val monitoringObserver = Observer<Int> { state ->
        var dialogTitle = "Beacons detected"
        var dialogMessage = "didEnterRegionEvent has fired"
        var stateString = "inside"
        if (state == MonitorNotifier.OUTSIDE) {
            dialogTitle = "No beacons detected"
            dialogMessage = "didExitRegionEvent has fired"
            stateString == "outside"
            beaconCountTextView.text = "Outside of the beacon region -- no beacons detected"
            beaconListView.adapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, arrayOf("--"))
        }
        else {
            beaconCountTextView.text = "Inside the beacon region."
        }
        Log.d(TAG, "monitoring state changed to : $stateString")
        val builder =
            AlertDialog.Builder(this)
        builder.setTitle(dialogTitle)
        builder.setMessage(dialogMessage)
        builder.setPositiveButton(android.R.string.ok, null)
        alertDialog?.dismiss()
        alertDialog = builder.create()
        alertDialog?.show()
    }

    val rangingObserver = Observer<Collection<Beacon>> { beacons ->
        Log.d(TAG, "Ranged: ${beacons.count()} beacons")
        if (BeaconManager.getInstanceForApplication(this).rangedRegions.isNotEmpty()) {

            beaconCountTextView.text = "Ranging enabled: ${beacons.count()} beacon(s) detected"
//            beaconListView.adapter = ArrayAdapter(this, android.R.layout.simple_list_item_1,
//                beacons
//                    .sortedBy { it.distance }
//                    .map { "${it.bluetoothAddress}\nid2: ${it.id2} id3: ${it.id3} rssi: ${it.rssi}\nest. distance: ${it.distance} m"}.toTypedArray()
//            )
        }
        beacons.forEach { beacon ->
            val rawBytes = beacon.lastPacketRawBytes
            if (rawBytes != null && rawBytes.size > 29) {
                val byte29 = rawBytes[29]
                Log.d(TAG, "Byte 29: $byte29")
                val bit5 = (byte29.toInt() shr 5) and 0x01 // Mask for bit 5
                val bit6 = (byte29.toInt() shr 6) and 0x01 // Mask for bit 6
                val bit7 = (byte29.toInt() shr 7) and 0x01 // Mask for bit 7

                // Log the values of the bits
                Log.d(TAG, "Bit 0: $bit5, Bit 1: $bit6, Bit 2: $bit7")

                // Combine bits to form a value between 0 and 7
                val batteryBits = (bit7 shl 2) or (bit6 shl 1) or bit5

                // Map the batteryBits to a level between 1 and 6
                val batteryLevel = when (batteryBits) {
                    0 -> 1 // Map 000 (0) to level 1
                    1 -> 2 // Map 001 (1) to level 2
                    2 -> 3 // Map 010 (2) to level 3
                    3 -> 4 // Map 011 (3) to level 4
                    4 -> 5 // Map 100 (4) to level 5
                    5 -> 6 // Map 101 (5) to level 6
                    else -> 6 // Treat 110 (6) and 111 (7) as level 6 (or could handle differently)
                }
                Log.d(TAG, "Device address: ${beacon.bluetoothAddress}")
                Log.d(TAG, "Battery Level: $batteryLevel")

            } else {
                Log.d(TAG, "Raw bytes are unavailable or too short to access byte 29.")
            }
            val beaconDetails = beacons.map { beacon ->
                "Address: ${beacon.bluetoothAddress}, ID2: ${beacon.id2}, ID3: ${beacon.id3}\nDistance: ${beacon.distance}m"
            }

            beaconListView.adapter = ArrayAdapter(
                this,
                android.R.layout.simple_list_item_1,
                beaconDetails
            )

        }
        if (BeaconManager.getInstanceForApplication(this).rangedRegions.size > 0) {
            beaconCountTextView.text = "Ranging enabled: ${beacons.count()} beacon(s) detected"

            // Filter beacons with id3 (minor) in the range 1 to 6
            val relevantBeacons = beacons
                .filter { it.id3.toInt() in 1..200 }
                .sortedBy { it.distance }

            // Display detected beacons
            beaconListView.adapter = ArrayAdapter(
                this, android.R.layout.simple_list_item_1,
                relevantBeacons.map { "${it.bluetoothAddress}\nid2: ${it.id2} id3: ${it.id3} rssi: ${it.rssi}\nest. distance: ${it.distance} m" }
                    .toTypedArray()
            )

            // Proceed if we have at least 3 beacons
            if (relevantBeacons.size >= 3) {
                val beaconPositions1 = relevantBeacons.take(3).mapNotNull { beacon ->
                    // Safely find the corresponding beacon by id3 (minor) value
                    val matchingBeacon = beaconsMap.find { it.minor == beacon.id3.toInt() }
                    matchingBeacon?.let {

                        BeaconPosition(it.minor,it.position.x, it.position.y, beacon.distance)

                    }
//                    org.altbeacon.api.Beacon(
//                        MacAddress =beacon.bluetoothAddress , // Add real data if available
//                        UUID = beacon.id1.toString(),       // Add real data if available
//                        Major = beacon.id2.toString(),      // Add real data if available
//                        Minor = beacon.id3.toString(),
//                        RSSI = beacon.rssi,        // Add real data if available
//                        Distance = beacon.distance.toString()
//                    )
                }
                val updatedBeacons =relevantBeacons.take(3).map { beacon ->
                    org.altbeacon.api.Beacon(
                        MacAddress =beacon.bluetoothAddress , // Add real data if available
                        UUID = beacon.id1.toString(),       // Add real data if available
                        Major = beacon.id2.toString(),      // Add real data if available
                        Minor = beacon.id3.toString(),
                        RSSI = beacon.rssi,        // Add real data if available
                        Distance = beacon.distance.toString()
                    )
                }
                val updatedBeaconData = BeaconData(
                    Beacons = updatedBeacons,
                    Timestamp = Instant.now().toString()
                )
                sendBeaconDataToApi(updatedBeaconData)
                val position1 = calculateAveragePosition(beaconPositions1)
                val kalmanFilter = position1?.let { KalmanFilter(it.first, position1.second) }

                if (position1 != null) {
                    println("Initial Position from Average: $position1")

                    // Step 2: Use the initial position to focus the particle filter
                    val refinedPosition = applyParticleFiltering(
                        beacons = beaconPositions1,
                        numParticles = 1000,
                        iterations = 10,
                        initialEstimate = position1
                    )
                    if (kalmanFilter != null) {
                        kalmanFilter.update(refinedPosition.first, refinedPosition.second)
                    }
                    val smoothedPosition = kalmanFilter?.getState()

                    if (smoothedPosition != null) {
                        positionView.setDevicePosition(smoothedPosition)
                    }
//                    val updatedBeacons = beaconPositions1.map { position ->
//                        org.altbeacon.api.Beacon(
//                            MacAddress = "", // Add real data if available
//                            UUID = "",       // Add real data if available
//                            Major = "",      // Add real data if available
//                            Minor = position.minor.toString(),
//                            RSSI = 0,        // Add real data if available
//                            Distance = position.distance.toString()
//                        )
//                    }



                    // Function to send data to API




                    println("Refined Position from Particle Filtering: $refinedPosition")
                } else {
                    println("Insufficient beacons for trilateration.")
                }
                if (position1 != null) {
                    positionView.setDevicePosition1(position1)
                    Log.d("API","position: ${position1}")
                }

            } else {
                Log.d(TAG, "Not enough beacons for trilateration.")
            }
        }
    }

    fun rangingButtonTapped(view: View) {
        val beaconManager = BeaconManager.getInstanceForApplication(this)
        if (beaconManager.rangedRegions.size == 0) {
            beaconManager.startRangingBeacons(beaconReferenceApplication.region)
            rangingButton.text = "Stop Ranging"
            beaconCountTextView.text = "Ranging enabled -- awaiting first callback"
        }
        else {
            beaconManager.stopRangingBeacons(beaconReferenceApplication.region)
            rangingButton.text = "Start Ranging"
            beaconCountTextView.text = "Ranging disabled -- no beacons detected"
            beaconListView.adapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, arrayOf("--"))
        }
    }

    fun monitoringButtonTapped(view: View) {
        var dialogTitle = ""
        var dialogMessage = ""
        val beaconManager = BeaconManager.getInstanceForApplication(this)
        if (beaconManager.monitoredRegions.size == 0) {
            beaconManager.startMonitoring(beaconReferenceApplication.region)
            dialogTitle = "Beacon monitoring started."
            dialogMessage = "You will see a dialog if a beacon is detected, and another if beacons then stop being detected."
            monitoringButton.text = "Stop Monitoring"

        }
        else {
            beaconManager.stopMonitoring(beaconReferenceApplication.region)
            dialogTitle = "Beacon monitoring stopped."
            dialogMessage = "You will no longer see dialogs when beacons start/stop being detected."
            monitoringButton.text = "Start Monitoring"
        }
        val builder =
            AlertDialog.Builder(this)
        builder.setTitle(dialogTitle)
        builder.setMessage(dialogMessage)
        builder.setPositiveButton(android.R.string.ok, null)
        alertDialog?.dismiss()
        alertDialog = builder.create()
        alertDialog?.show()

    }

    companion object {
        val TAG = "MainActivity"
        val PERMISSION_REQUEST_BACKGROUND_LOCATION = 0
        val PERMISSION_REQUEST_BLUETOOTH_SCAN = 1
        val PERMISSION_REQUEST_BLUETOOTH_CONNECT = 2
        val PERMISSION_REQUEST_FINE_LOCATION = 3
    }

    // Function to add a new distance sample to the cache
    fun addDistanceSample(id3: Identifier, distance: Double) {
        val distances = distanceCache.getOrPut(id3) { mutableListOf() }
        // Keep only the last 4 samples to avoid memory overflow
        if (distances.size >= 4) distances.removeAt(0)
        distances.add(distance)
    }

    // Function to retrieve the last 4 distance samples (or fewer if not available)
    fun fetchDistanceSample(id3: Identifier): List<Double> {
        return distanceCache[id3]?.toList() ?: emptyList()
    }
    fun onBeaconDistanceReceived(beacon: Beacon) {
//        addDistanceSample(beacon.id3, beacon.distance)
        val distances = distanceCache.getOrPut(beacon.id3) { mutableListOf() }

        // Add the new distance to the cache
        distances.add(beacon.distance)

        // Calculate the average distance
        val averageDistance = if (distances.isNotEmpty()) distances.average() else beacon.distance

        // Remove distances that are greater than the average
        distances.removeIf { it > averageDistance }

        // Keep only the last 4 samples
        if (distances.size > 4) distances.removeAt(0)
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event != null) {
            if (event.sensor.type == Sensor.TYPE_ROTATION_VECTOR) {
                val rotationMatrix = FloatArray(9)
                SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)

                val orientation = FloatArray(3)
                SensorManager.getOrientation(rotationMatrix, orientation)

                // Calculate azimuth (device's heading)
                currentAzimuth = Math.toDegrees(orientation[0].toDouble()).toFloat()
                Log.d(TAG, "Current Azimuth: $currentAzimuth")

                // Update the marker on the map with the new heading
                positionView.setDeviceDirection(currentAzimuth)
            }
        }
    }

    override fun onAccuracyChanged(p0: Sensor?, p1: Int) {
    }
    fun applyMovingAverage(beaconPositions: List<BeaconPosition>): List<BeaconPosition> {
        return beaconPositions.map { beacon ->
            // Use the minor identifier (converted to Identifier) as the key in the cache
            val distances = distanceCache.getOrPut(Identifier.parse(beacon.minor.toString())) { mutableListOf() }
            distances.add(beacon.distance)

            // Keep only the last 5 samples to avoid memory overflow
            if (distances.size > 10) distances.removeAt(0)

            // Calculate the average distance
            val avgDistance = distances.average()

            // Filter distances to keep only those below or equal to the average
            val validDistances = distances.filter { it <= avgDistance }

            // If we have valid distances, use the smallest one; otherwise, fallback to avgDistance
            val bestDistance = validDistances.minOrNull() ?: avgDistance

            // Return a new BeaconPosition with the filtered distance
            BeaconPosition(beacon.minor, beacon.x, beacon.y, bestDistance)
        }
    }


    fun adjustPositionBasedOnDirection(currentPosition: Pair<Double, Double>?): Pair<Double, Double>? {
        if (currentPosition == null || lastKnownPosition == null) return currentPosition

        val (lastX, lastY) = lastKnownPosition!!
        val (currentX, currentY) = currentPosition

        // Calculate the movement direction vector
        val deltaX = currentX - lastX
        val deltaY = currentY - lastY

        // Calculate the angle of movement in degrees
        val movementAngle = Math.toDegrees(Math.atan2(deltaY, deltaX))

        // Compare the movement angle with the current device azimuth
        val angleDifference = Math.abs(movementAngle - currentAzimuth)

        // If the movement aligns with the heading (within 30 degrees), accept it
        if (angleDifference <= 30) {
            return currentPosition
        } else {
            // If the movement doesn't align, predict the next position based on the last known heading
            val predictedX = lastX + Math.cos(Math.toRadians(currentAzimuth.toDouble())) * 0.5// Step size = 0.5 meters
            val predictedY = lastY + Math.sin(Math.toRadians(currentAzimuth.toDouble())) * 0.5

            return Pair(predictedX, predictedY)
        }
    }
}
// Data class for beacon position
data class BeaconPosition(  val minor: Int, val x: Double, val y: Double, val distance: Double)

// Sample predefined beacon positions
private val beaconsMap = listOf(
//    BeaconP(1, BeaconPosition(4.2, 2.0, 1.0)),
    BeaconP(1, BeaconPosition(1,4.2, -2.0, 1.0)),
    BeaconP(2, BeaconPosition(2,0.0, 0.5, 1.0)),
    BeaconP(3, BeaconPosition(3,-3.2, 1.0, 1.0)),

    BeaconP(4, BeaconPosition(4,-3.0, -2.0, 1.0)),
    BeaconP(6, BeaconPosition(6,4.2, 6.0, 1.0)) // Example additional beacon
)

// Trilateration function to calculate position
private fun calculateDevicePosition(
    beacon1: BeaconPosition, beacon2: BeaconPosition, beacon3: BeaconPosition
): Pair<Double, Double>? {
    val (x1, y1, d1) = beacon1
    val (x2, y2, d2) = beacon2
    val (x3, y3, d3) = beacon3

    val A = 2 * (x2 - x1)
    val B = 2 * (y2 - y1)
    val C = 2 * (x3 - x2)
    val D = 2 * (y3 - y2)

    val E = d1 * d1 - d2 * d2 + x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1
    val F = d2 * d2 - d3 * d3 + x3 * x3 - x2 * x2 + y3 * y3 - y2 * y2

    val denominator = A * D - B * C
    if (denominator == 0.0) return null

    val x = (E * D - B * F) / denominator
    val y = (A * F - E * C) / denominator

    return Pair(x, y)
}
data class BeaconP(
    val minor: Int,
    val position: BeaconPosition,
    var distance: Double? = null
)
fun calculateDevicePosition(beacons: List<BeaconPosition>): Pair<Double, Double>? {
    if (beacons.size < 3) return null // Need at least 3 beacons for trilateration

    // Initial guess: Average of beacon positions
    val initialGuess = doubleArrayOf(
        beacons.map { it.x }.average(),
        beacons.map { it.y }.average()
    )

    // Define the model function (residuals) and return the expected Pair from Apache Commons Math
    val modelFunction = MultivariateJacobianFunction { point ->
        val (x, y) = point.toArray()
        val residuals = DoubleArray(beacons.size)
        val jacobian = Array(beacons.size) { DoubleArray(2) }

        beacons.forEachIndexed { index, beacon ->
            val dx = x - beacon.x
            val dy = y - beacon.y
            val dist = sqrt(dx.pow(2) + dy.pow(2))

            // Residual: Difference between measured and actual distance
            residuals[index] = dist - beacon.distance

            // Fill in Jacobian matrix with partial derivatives
            if (dist != 0.0) {
                jacobian[index][0] = dx / dist
                jacobian[index][1] = dy / dist
            } else {
                jacobian[index][0] = 0.0
                jacobian[index][1] = 0.0
            }
        }

        // Return the correct type of Pair
        org.apache.commons.math3.util.Pair(ArrayRealVector(residuals), Array2DRowRealMatrix(jacobian))
    }

    // Create the least squares problem
    val problem = LeastSquaresBuilder()
        .start(initialGuess)
        .model(modelFunction)
        .target(DoubleArray(beacons.size)) // Target residuals are zero
        .lazyEvaluation(false)
        .maxEvaluations(1000)
        .maxIterations(1000)
        .build()

    // Optimize using Levenberg-Marquardt algorithm
    val optimum = LevenbergMarquardtOptimizer().optimize(problem)
    val point = optimum.point.toArray()

    return Pair(point[0], point[1])
}

// Extension function for power operation
private fun Double.pow(exp: Int) = this.pow(exp.toDouble())
fun calculateAveragePosition(beacons: List<BeaconPosition>): Pair<Double, Double>? {
    if (beacons.size < 3) return null

    val allPositions = mutableListOf<Pair<Double, Double>>()

    // Generate all possible subsets of 3 or 4 beacons
    val subsets = generateSubsets(beacons)

    // Calculate the position for each subset
    for (subset in subsets) {
        val position = calculateDevicePosition1(subset)
        if (position != null) {
            allPositions.add(position)
        }
    }

    // Compute the average position from all valid results
    val avgX = allPositions.map { it.first }.average()
    val avgY = allPositions.map { it.second }.average()

    return Pair(avgX, avgY)
}

// Helper function to generate all subsets of size 3 or 4
fun generateSubsets(beacons: List<BeaconPosition>): List<List<BeaconPosition>> {
    val subsets = mutableListOf<List<BeaconPosition>>()
    val n = beacons.size

    // Generate subsets of size 3 and 4
    for (i in 0 until n) {
        for (j in i + 1 until n) {
            for (k in j + 1 until n) {
                subsets.add(listOf(beacons[i], beacons[j], beacons[k]))
                if (n >= 4) {
                    for (l in k + 1 until n) {
                        subsets.add(listOf(beacons[i], beacons[j], beacons[k], beacons[l]))
                    }
                }
            }
        }
    }
    return subsets
}

// Calculate the position using least squares for a given subset
fun calculateDevicePosition1(beacons: List<BeaconPosition>): Pair<Double, Double>? {
    val initialGuess = doubleArrayOf(
        beacons.map { it.x }.average(),
        beacons.map { it.y }.average()
    )

    val modelFunction = MultivariateJacobianFunction { point ->
        val (x, y) = point.toArray()
        val residuals = DoubleArray(beacons.size)
        val jacobian = Array(beacons.size) { DoubleArray(2) }

        beacons.forEachIndexed { index, beacon ->
            val dx = x - beacon.x
            val dy = y - beacon.y
            val dist = sqrt(dx.pow(2) + dy.pow(2))

            residuals[index] = dist - beacon.distance

            if (dist != 0.0) {
                jacobian[index][0] = dx / dist
                jacobian[index][1] = dy / dist
            } else {
                jacobian[index][0] = 0.0
                jacobian[index][1] = 0.0
            }
        }
        org.apache.commons.math3.util.Pair(ArrayRealVector(residuals), Array2DRowRealMatrix(jacobian))
    }

    val problem = LeastSquaresBuilder()
        .start(initialGuess)
        .model(modelFunction)
        .target(DoubleArray(beacons.size))
        .lazyEvaluation(false)
        .maxEvaluations(1000)
        .maxIterations(1000)
        .build()

    return try {
        val optimum = LevenbergMarquardtOptimizer().optimize(problem)
        val point = optimum.point.toArray()
        Pair(point[0], point[1])
    } catch (e: Exception) {
        null // Handle cases where optimization fails
    }
}
// Step 3: Outlier detection using Z-score filtering
fun filterOutliers(positions: List<Pair<Double, Double>>): List<Pair<Double, Double>> {
    val meanX = positions.map { it.first }.average()
    val meanY = positions.map { it.second }.average()

    val stdDevX = sqrt(positions.map { (it.first - meanX).pow(2) }.average())
    val stdDevY = sqrt(positions.map { (it.second - meanY).pow(2) }.average())

    // Filter positions with a Z-score > 2 (common threshold for outliers)
    return positions.filter { (x, y) ->
        val zScoreX = abs((x - meanX) / stdDevX)
        val zScoreY = abs((y - meanY) / stdDevY)
        zScoreX <= 2.0 && zScoreY <= 2.0
    }
}
