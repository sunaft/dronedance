package sk.uniba.krucena

import android.app.Application
import android.util.Log
import androidx.annotation.MainThread
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelLazy
import androidx.lifecycle.ViewModelProvider
import androidx.lifecycle.ViewModelStore

open class MyApplication : Application() {

    private val TAG = this::class.simpleName

    // ViewModelStore for global use
    val globalViewModelStore = ViewModelStore()

    // Create a global ViewModel in the application
    @MainThread
    inline fun <reified VM : ViewModel> globalViewModels(): Lazy<VM> {
        val factory = ViewModelProvider.AndroidViewModelFactory.getInstance(this)
        return ViewModelLazy(VM::class, { globalViewModelStore }, { factory })
    }

    // Use globalViewModels() to get the MSDKManagerVM instance
    private val msdkManagerVM: MSDKManagerVM by globalViewModels()


    override fun onCreate() {
        super.onCreate()
        Log.i("MyApplication", "onCreate()")
        msdkManagerVM.initMobileSDK(this)
    }
}