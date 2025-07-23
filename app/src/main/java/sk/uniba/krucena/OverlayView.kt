package sk.uniba.krucena

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View

import org.opencv.android.Utils
import org.opencv.core.Mat


class OverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0) : View(context, attrs, defStyleAttr) {

    private val paint = Paint().apply {
        color = Color.RED
        strokeWidth = 5f
        style = Paint.Style.STROKE
    }

    private val emptyBitmap = Bitmap.createBitmap(1, 1, Bitmap.Config.ARGB_8888)
    private var btmp : Bitmap = emptyBitmap
    private val LEFT_BORDER = 31.0f

    fun updateBitmap(bmp: Mat)
    {
        val bitmap = Bitmap.createBitmap(bmp.cols(), bmp.rows(), Bitmap.Config.ARGB_8888)
        Utils.matToBitmap(bmp, bitmap)
        btmp = bitmap
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawBitmap(btmp, LEFT_BORDER, 0.0f, paint)  //this may need to be tuned for each drone
        btmp.recycle()
        btmp = emptyBitmap
    }
}