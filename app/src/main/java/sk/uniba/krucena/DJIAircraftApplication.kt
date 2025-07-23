package sk.uniba.krucena

import android.content.Context
import android.util.Log

class DJIAircraftApplication : MyApplication() {

    override fun attachBaseContext(base: Context?) {
        super.attachBaseContext(base)
        com.cySdkyc.clx.Helper.install(this)
        Log.i("DJIAircraftApplication", "attachBaseContext()")
    }
}