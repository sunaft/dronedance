package sk.uniba.krucena

import android.content.Context
import android.util.Log
import java.io.File

class Config(val ctx : Context) {

    val maxDroneID = 6
    val fromHotspot = 1
    val UDPbroadcast = 2

    var isServer: Boolean = false
    var obtainServerIP : Int = UDPbroadcast
    var musicServerIP : String = ""
    var musicServerPort : Int = 4212
    var droneId: Int = -1
    var expectedNumberOfDrones = 0
    var northCorrection: Float = 0.0f
    var black_maxRGB_t: Int = 151
    var black_chroma_t: Int = 90
    var red_t: Int = 63
    var green_t: Int = 48
    var blue_t: Int = 48
    var yellow_t: Int = 25
    var show_contours: Int = 1
    var cpp_debug: Int = 0
    var position_debug: Int = 0

    fun readConfig() {
        for (i in 0 until 2) {
            try {
                val file = File(ctx.filesDir, "config.txt")
                Log.i("Config", "Looking for config in: ${ctx.filesDir.absolutePath}")


                Log.i("Config", "config load")
                val lines = file.readLines()
                for (rawLine in lines) {

                    val l = rawLine.trim()
                    if (l.isEmpty()) continue
                    if (l.startsWith('#')) continue

                    val ln = l.split("=", limit = 2)
                    if (ln.size != 2) {
                        Log.w("Config", "Skipping invalid config line: '$rawLine'")
                        continue
                    }
                    val key = ln[0].trim()
                    val value = ln[1].trim()

                    when (key) {
                        "isServer" -> {
                            isServer = value.toBoolean()
                            Log.i("Config", "isServer=${isServer}")
                        }

                        "obtainServerIP" -> {
                            if (value == "fromHotspot") obtainServerIP = fromHotspot
                            else if (value == "UDPbroadcast") obtainServerIP = UDPbroadcast
                            else Log.w("Config", "obtainServerIP=${value} not recognized")
                            Log.i("Config", "obtainServerIP=${value}")
                        }

                        "musicServerIP" -> {
                            if (value.matches(Regex("^\\d{1,3}(\\.\\d{1,3}){3}$")))
                                musicServerIP = value
                            else if (value == "none") musicServerIP = value
                            else Log.w("Config", "musicServerIP=${value} not recognized")
                            Log.i("Config", "musicServerIP=${value}")
                        }

                        "musicServerPort" -> {
                            musicServerPort = Integer.parseInt(value)
                            Log.i("Config", "musicServerPort=${value}")
                        }

                        "droneId" -> {
                            droneId = Integer.parseInt(value)
                            Log.i("Config", "droneId=${droneId}")
                        }

                        "expectedNumberOfDrones" -> {
                            expectedNumberOfDrones = Integer.parseInt(value)
                            Log.i("Config", "expectedNumberOfDrones=${expectedNumberOfDrones}")
                        }

                        "north" -> {
                            northCorrection = value.toFloat()
                            Log.i("Config", "north=${northCorrection}")
                        }

                        "black_maxRGB_t" -> {
                            black_maxRGB_t = Integer.parseInt(value)
                            Log.i("Config", "black_maxRGB_t=${black_maxRGB_t}")
                        }

                        "black_chroma_t" -> {
                            black_chroma_t = Integer.parseInt(value)
                            Log.i("Config", "black_chroma_t=${black_chroma_t}")
                        }

                        "red_t" -> {
                            red_t = Integer.parseInt(value)
                            Log.i("Config", "red_t=${red_t}")
                        }

                        "green_t" -> {
                            green_t = Integer.parseInt(value)
                            Log.i("Config", "green_t=${green_t}")
                        }

                        "blue_t" -> {
                            blue_t = Integer.parseInt(value)
                            Log.i("Config", "blue_t=${blue_t}")
                        }

                        "yellow_t" -> {
                            yellow_t = Integer.parseInt(value)
                            Log.i("Config", "yellow_t=${yellow_t}")
                        }

                        "show_contours" -> {
                            show_contours = Integer.parseInt(value)
                            Log.i("Config", "show_contours=${show_contours}")
                        }

                        "cpp_debug" -> {
                            cpp_debug = Integer.parseInt(value)
                            Log.i("Config", "cpp_debug=${cpp_debug}")
                        }

                        "position_debug" -> {
                            position_debug = Integer.parseInt(value)
                            Log.i("Config", "position_debug=${position_debug}")
                        }
                    }
                }
                break
            } catch (e: Exception) {
                Log.e("Config", "Failed to read config file: ${e.message}")
                copyConfigFromAssetsIfMissing()
            }
        }
    }

    fun updateConfig() {
        val file = File(ctx.filesDir, "config.txt")
        if (!file.exists()) {
            Log.e("Config", "Config file not found for update")
            return
        }

        val originalLines = file.readLines()
        val updatedLines = mutableListOf<String>()

        for (line in originalLines) {
            val trimmed = line.trim()
            if (trimmed.isEmpty() || trimmed.startsWith('#')) {
                updatedLines.add(line)
                continue
            }

            val parts = trimmed.split("=", limit = 2)
            if (parts.size != 2) {
                updatedLines.add(line)
                continue
            }

            val key = parts[0].trim()
            val newValue: String? = when (key) {
                "isServer" -> isServer.toString()
                "obtainServerIP" -> if (obtainServerIP == fromHotspot) "fromHotspot" else "UDPbroadcast"
                "musicServerIP" -> musicServerIP
                "musicServerPort" -> musicServerPort.toString()
                "droneId" -> droneId.toString()
                "expectedNumberOfDrones" -> expectedNumberOfDrones.toString()
                "north" -> northCorrection.toString()
                "black_maxRGB_t" -> black_maxRGB_t.toString()
                "black_chroma_t" -> black_chroma_t.toString()
                "red_t" -> red_t.toString()
                "green_t" -> green_t.toString()
                "blue_t" -> blue_t.toString()
                "yellow_t" -> yellow_t.toString()
                "show_contours" -> show_contours.toString()
                "cpp_debug" -> cpp_debug.toString()
                "position_debug" -> position_debug.toString()
                else -> null
            }

            if (newValue != null) {
                updatedLines.add("${key}=${newValue}")
            } else {
                updatedLines.add(line) // Unknown key — leave as is
            }
        }

        file.writeText(updatedLines.joinToString("\n"))
        Log.i("Config", "Config file updated.")
    }

    fun copyConfigFromAssetsIfMissing() {
        val configFile = File(ctx.filesDir, "config.txt")
        if (!configFile.exists()) {
            try {
                ctx.assets.open("config.txt").use { input ->
                    configFile.outputStream().use { output ->
                        input.copyTo(output)
                    }
                }
                Log.i("Config", "Copied config.txt from assets to ${configFile.absolutePath}")
            } catch (e: Exception) {
                Log.e("Config", "Failed to copy config: ${e.message}")
            }
        } else {
            Log.i("Config", "Config file already exists at ${configFile.absolutePath}")
        }
    }
}