package org.altbeacon.beaconreference

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View

class PositionView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null
) : View(context, attrs) {

    private var beacons = mutableListOf<Pair<Double, Double>>()
    private var devicePosition: Pair<Double, Double>? = null
    private var devicePosition1: Pair<Double, Double>? = null
    private var devicePosition2: Pair<Double, Double>? = null
    private var devicePositionKalmanFilter: Pair<Double, Double>? = null

    private val xRangeInMeters = 10 // The total X range: from -2.305 to 4.734 meters
    private val yRangeInMeters =15
    private var deviceDirection: Float = 0f // Azimuth in degrees


    private val beaconPaint = Paint().apply {
        color = Color.BLUE
        style = Paint.Style.FILL
    }


    private val devicePaint = Paint().apply {
        color = Color.RED
        style = Paint.Style.FILL
    }

    private val devicePaint1 = Paint().apply {
        color = Color.GREEN
        style = Paint.Style.FILL
    }
    private val devicePaint2 = Paint().apply {
        color = Color.BLACK
        style = Paint.Style.FILL
    }

    private val deviceKalmanPaint = Paint().apply {
        color = Color.GREEN
        style = Paint.Style.FILL
    }

    private val gridPaint = Paint().apply {
        color = Color.GRAY
        style = Paint.Style.STROKE
        strokeWidth = 2f
    }

    private val axisPaint = Paint().apply {
        color = Color.BLACK
        style = Paint.Style.STROKE
        strokeWidth = 5f
    }

    // Assume we want 10 meters in both x and y directions
    private val metersInGrid = 2  // You can adjust this value

    // Function to set the beacons' positions
    fun setBeacons(beaconPositions: List<Pair<Double, Double>>) {
        beacons.clear()
        beacons.addAll(beaconPositions)
        invalidate() // Re-draw the view
    }

    // Function to set the device position
    fun setDevicePosition(position: Pair<Double, Double>) {
        devicePosition = position
        invalidate() // Re-draw the view
    }
    fun setDevicePosition1(position1: Pair<Double, Double>) {
        devicePosition1 = position1
        invalidate() // Re-draw the view
    }
    fun setDevicePosition2(position2: Pair<Double, Double>) {
        devicePosition2 = position2
        invalidate() // Re-draw the view
    }

    fun setDeviceKalmanPosition(position: Pair<Double, Double>) {
        devicePosition = position
        invalidate() // Re-draw the view
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        // Draw grid and axes
        drawGrid(canvas)
        drawAxes(canvas)

        // Draw beacons
        beacons.forEach { (x, y) ->
            val screenX = convertToScreenX(x)
            val screenY = convertToScreenY(y)
            // Save the current state of the canvas



            canvas.drawCircle(screenX, screenY, 20f, beaconPaint) // Draw beacon as a circle
            // Restore the canvas to its original state

        }

        // Draw the device position if available
        devicePosition?.let { (x, y) ->
            val screenX = convertToScreenX(x)
            val screenY = convertToScreenY(y)
            canvas.save()
            // Rotate the canvas around the device's position
            canvas.rotate(deviceDirection, screenX, screenY)
            drawArrow(canvas, screenX, screenY, devicePaint)
//            canvas.drawRect(screenX - 10, screenY - 10, screenX + 10, screenY + 10, devicePaint) // Draw device as a square
            canvas.restore()
        }

        devicePosition1?.let { (x,y) ->
            val screenX = convertToScreenX(x)
            val screenY = convertToScreenY(y)
            canvas.drawRect(screenX - 10, screenY - 10, screenX + 10, screenY + 10, devicePaint1) // Draw device as a square
//            drawDeviceMarker(it, canvas, devicePaint1)
//            canvas.save()
//            // Rotate the canvas around the device's position
//            canvas.rotate(deviceDirection, screenX, screenY)
//            drawArrow(canvas, screenX, screenY, devicePaint)
////            canvas.drawRect(screenX - 10, screenY - 10, screenX + 10, screenY + 10, devicePaint) // Draw device as a square
//            canvas.restore()
        }

        devicePosition2?.let { (x, y) ->
            val screenX = convertToScreenX(x)
            val screenY = convertToScreenY(y)
            canvas.drawRect(screenX - 10, screenY - 10, screenX + 10, screenY + 10, devicePaint2) // Draw device as a square
        }

        devicePositionKalmanFilter?.let { (x, y) ->
            val screenX = convertToScreenX(x)
            val screenY = convertToScreenY(y)
            canvas.drawRect(screenX - 10, screenY - 10, screenX + 10, screenY + 10, deviceKalmanPaint) // Draw device as a square
        }
    }

    // Function to draw a grid for visual clarity
    private fun drawGrid(canvas: Canvas?) {
        val metersInPixels = width / metersInGrid  // This gives the pixel size for 1 meter

        for (i in 0..width step metersInPixels) {
            canvas?.drawLine(i.toFloat(), 0f, i.toFloat(), height.toFloat(), gridPaint)
        }
        for (i in 0..height step metersInPixels) {
            canvas?.drawLine(0f, i.toFloat(), width.toFloat(), i.toFloat(), gridPaint)
        }
    }

    // Function to draw x-axis and y-axis
    private fun drawAxes(canvas: Canvas) {
        // Draw x-axis (horizontal line through center)
        canvas.drawLine(0f, (height / 2).toFloat(), width.toFloat(), (height / 2).toFloat(), axisPaint)

        // Draw y-axis (vertical line through center)
        canvas.drawLine((width / 2).toFloat(), 0f, (width / 2).toFloat(), height.toFloat(), axisPaint)
    }

    // Convert real-world X to screen X (scaling based on real meters)
    private fun convertToScreenX(x: Double): Float {
        val metersInPixels = width / xRangeInMeters  // This gives the pixel size for 1 meter
        return (width / 2 + (x * metersInPixels)).toFloat()
    }

    // Convert real-world Y to screen Y (scaling based on real meters)
    private fun convertToScreenY(y: Double): Float {
        val metersInPixels = height / yRangeInMeters  // This gives the pixel size for 1 meter
        return (height / 2 - (y * metersInPixels)).toFloat()
    }
    fun setDeviceDirection(azimuth: Float) {
        deviceDirection = azimuth
        invalidate() // Redraw the view with the updated direction
    }
    // Helper function to draw a device marker
    private fun drawDeviceMarker(position: Pair<Double, Double>, canvas: Canvas, paint: Paint) {
        val (x, y) = position
        val screenX = convertToScreenX(x)
        val screenY = convertToScreenY(y)
        canvas.drawRect(screenX - 10, screenY - 10, screenX + 10, screenY + 10, paint)
    }
    private fun drawArrow(canvas: Canvas, x: Float, y: Float, paint: Paint) {
        val arrowLength = 50f  // Length of the arrow shaft
        val arrowWidth = 20f    // Width of the arrowhead

        // Draw the shaft of the arrow (a vertical line)
        canvas.drawLine(x, y, x, y - arrowLength, paint)

        // Draw the arrowhead as a triangle
        val path = android.graphics.Path()
        path.moveTo(x - arrowWidth / 2, y - arrowLength) // Left point of the arrowhead
        path.lineTo(x + arrowWidth / 2, y - arrowLength) // Right point of the arrowhead
        path.lineTo(x, y - arrowLength - arrowWidth)     // Tip of the arrowhead
        path.close()

        // Fill the arrowhead
        canvas.drawPath(path, paint)
    }


}


