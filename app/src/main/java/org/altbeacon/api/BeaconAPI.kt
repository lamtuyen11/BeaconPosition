package org.altbeacon.api

import android.util.Log
import com.google.gson.Gson
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody
import java.security.cert.X509Certificate
import javax.net.ssl.SSLContext
import javax.net.ssl.TrustManager
import javax.net.ssl.X509TrustManager

// Data classes for JSON structure
data class Beacon(
    var MacAddress: String = "",
    var UUID: String = "",
    var Major: String = "",
    var Minor: String = "",
    var RSSI: Int = 0,
    var Distance: String = ""
)
data class BeaconPOS(
    var MacAddress: String = "",
    var UUID: String = "",
    var Major: String = "",
    var Minor: String = "",
    var RSSI: Int = 0,
    var Distance: String = ""
)

data class BeaconData(
    var Beacons: List<Beacon>,
    var Timestamp: String
)
fun getUnsafeOkHttpClient(): OkHttpClient {
    val trustAllCerts = arrayOf<TrustManager>(
        object : X509TrustManager {
            override fun checkClientTrusted(chain: Array<out X509Certificate>?, authType: String?) {}
            override fun checkServerTrusted(chain: Array<out X509Certificate>?, authType: String?) {}
            override fun getAcceptedIssuers(): Array<X509Certificate> = arrayOf()
        }
    )

    val sslContext = SSLContext.getInstance("SSL")
    sslContext.init(null, trustAllCerts, java.security.SecureRandom())

    val sslSocketFactory = sslContext.socketFactory

    return OkHttpClient.Builder()
        .sslSocketFactory(sslSocketFactory, trustAllCerts[0] as X509TrustManager)
        .hostnameVerifier { _, _ -> true }
        .build()
}
fun sendBeaconDataToApi(beaconData: BeaconData) {
    val gson = Gson()
    val jsonData = gson.toJson(beaconData)
    println("Prepared JSON Data: $jsonData")

    val token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpcEFkZHJlc3MiOiI6OjEiLCJ1c2VyTmFtZSI6IkFwcFVzZXIiLCJleHAiOjE3MzU4ODgyMjR9.7j6FKb6r2_LU-94P3B7Rm3tMy4auqaNoMGTw4VsW9QY" // Replace with your actual token
    println("Token: $token")
//    val client = OkHttpClient.Builder()
//        .hostnameVerifier { _, _ -> true } // For SSL certificate issues (e.g., self-signed)
//        .build()
    val client = getUnsafeOkHttpClient()

    val mediaType = "application/json; charset=utf-8".toMediaType()
    val body = RequestBody.create(mediaType, jsonData)

    val request = Request.Builder()
        .url("https://59.120.37.49:9713/API/GetUserPos")
        .addHeader("Authorization", "Bearer $token")
        .post(body)
        .build()

    Thread {
        try {
            client.newCall(request).execute().use { response ->
                if (!response.isSuccessful) {
                    Log.d("API","Failed to post data: ${response.code}")
                } else {
                    Log.d("API","Response: ${response.body?.string()}")
                }
            }
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }.start()
}