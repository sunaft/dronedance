package sk.uniba.krucena

import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.common.utils.CallbackUtils
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ILiveStreamManager
import dji.v5.manager.datacenter.livestream.*
import dji.v5.manager.interfaces.ICameraStreamManager

/**
 * ClassName : LiveStreamVM
 * Description :
 * Author : daniel.chen
 * CreateDate : 2022/3/23 11:04 上午
 * Copyright : ©2022 DJI All Rights Reserved.
 */
class LiveStreamVM : ViewModel() {
    private val availableCameraUpdatedListener: ICameraStreamManager.AvailableCameraUpdatedListener
    private val liveStreamStatusListener: LiveStreamStatusListener
    val liveStreamStatus = MutableLiveData<LiveStreamStatus?>()
    val liveStreamError = MutableLiveData<IDJIError?>()
    val availableCameraList = MutableLiveData<List<ComponentIndexType>>()
    val streamManager: ILiveStreamManager = MediaDataCenter.getInstance().liveStreamManager
    val cameraManager: ICameraStreamManager = MediaDataCenter.getInstance().cameraStreamManager

    init {
        liveStreamStatusListener = object : LiveStreamStatusListener {
            override fun onLiveStreamStatusUpdate(status: LiveStreamStatus?) {
                status?.let {
                    liveStreamStatus.postValue(it)
                }
            }

            override fun onError(error: IDJIError?) {
                error?.let {
                    liveStreamError.postValue(it)
                }
            }
        }

        availableCameraUpdatedListener = ICameraStreamManager.AvailableCameraUpdatedListener { list ->
            availableCameraList.postValue(list)
        }

        addListener()
    }

    override fun onCleared() {
        super.onCleared()
        removeListener()
    }

    private fun reset() {
        liveStreamError.postValue(null)
        liveStreamStatus.postValue(null)
    }

    fun stopStream(callback: CommonCallbacks.CompletionCallback?) {
        streamManager.stopStream(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                CallbackUtils.onSuccess(callback)
                reset()
            }

            override fun onFailure(error: IDJIError) {
                CallbackUtils.onFailure(callback, error)
            }

        })
    }

    fun isStreaming(): Boolean {
        return streamManager.isStreaming;
    }

    fun setCameraIndex(cameraIndex: ComponentIndexType) {
        streamManager.cameraIndex = cameraIndex
    }

    fun addListener() {
        streamManager.addLiveStreamStatusListener(liveStreamStatusListener)
        cameraManager.addAvailableCameraUpdatedListener(availableCameraUpdatedListener)
    }

    fun removeListener() {
        streamManager.removeLiveStreamStatusListener(liveStreamStatusListener)
        cameraManager.removeAvailableCameraUpdatedListener(availableCameraUpdatedListener)
    }

    fun getAircraftStreamFrameInfo(cameraIndex: ComponentIndexType) = cameraManager.getAircraftStreamFrameInfo(cameraIndex)
}