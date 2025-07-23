package sk.uniba.krucena

import android.content.Context
import android.net.ConnectivityManager
import android.util.Log
import android.widget.Toast
import androidx.lifecycle.ViewModel
import dji.v5.common.error.IDJIError
import dji.v5.common.register.DJISDKInitEvent
import dji.v5.manager.SDKManager
import dji.v5.manager.interfaces.SDKManagerCallback
import dji.v5.network.DJINetworkManager

class MSDKManagerVM : ViewModel() {

    var isInit = false
    private val TAG = this::class.simpleName

    fun isOnline(context: Context): Boolean {
        val cm = context.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
        return cm.activeNetworkInfo?.isConnected == true
    }

    fun initMobileSDK(appContext: Context) {

        if (!isOnline(appContext))
        {
            Log.e("DJI", "Internet required for DJI SDK startup")
            Toast.makeText(appContext, "Internet required to use the drone features", Toast.LENGTH_LONG).show()
            return
        }

        SDKManager.getInstance().init(appContext, object: SDKManagerCallback {

            override fun onRegisterSuccess() {
                Log.i(TAG, "onRegisterSuccess: ")
            }
            override fun onRegisterFailure(error: IDJIError?) {
                Log.i(TAG, "onRegisterFailure: ")
            }
            override fun onProductConnect(productId: Int) {
                Log.i(TAG, "onProductConnect: ")
            }
            override fun onProductDisconnect(productId: Int) {
                Log.i(TAG, "onProductDisconnect: ")
            }
            override fun onProductChanged(productId: Int)
            {
                Log.i(TAG, "onProductChanged: ")
            }
            override fun onDatabaseDownloadProgress(current: Long, total: Long) {
                Log.i(TAG, "onDatabaseDownloadProgress: ${current/total}")
            }

            override fun onInitProcess(event: DJISDKInitEvent?, totalProcess: Int) {
                Log.i(TAG, "onInitProcess: ")
                if (event == DJISDKInitEvent.INITIALIZE_COMPLETE) {
                    isInit = true
                    SDKManager.getInstance().registerApp()
                }
            }
        })

        DJINetworkManager.getInstance().addNetworkStatusListener { isAvailable ->
            if (isInit && isAvailable && !SDKManager.getInstance().isRegistered) {
                SDKManager.getInstance().registerApp()
            }
        }
    }

    fun destroyMobileSDK() {
        SDKManager.getInstance().destroy()
    }
}