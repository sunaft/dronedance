package sk.uniba.krucena

object NativeBridge {
    init {
        System.loadLibrary("fastimglib") // this matches CMake target name
    }

    external fun localization(matAddrInput: Long,
                            cameraPosition : FloatArray, droneId: Int)

    external fun setMode(visualization_mode : Int,
                         show_contours : Int,
                         cpp_debug : Int,
                         position_debug : Int)

    external fun setupColors(new_black_maxRGB_t : Int,
                              new_black_chroma_t : Int,
                              new_red_t : Int,
                              new_green_t : Int,
                              new_blue_t : Int,
                              new_yellow_t : Int)
}