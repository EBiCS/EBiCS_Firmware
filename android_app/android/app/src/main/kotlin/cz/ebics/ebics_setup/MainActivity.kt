package cz.ebics.ebics_setup

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbConstants
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbEndpoint
import android.hardware.usb.UsbInterface
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.view.WindowManager
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.MethodChannel
import java.util.concurrent.Executors

/**
 * Raw USB bulk access to an ST-Link programmer (USB host / OTG).
 *
 * Dart side: lib/usb/usb_bridge.dart. All transfers run in order on one background thread.
 */
class MainActivity : FlutterActivity() {
    companion object {
        private const val CHANNEL = "cz.ebics/usb"
        private const val ACTION_USB_PERMISSION = "cz.ebics.ebics_setup.USB_PERMISSION"
        private const val ST_VID = 0x0483
        // ST-Link V2, V2-1, V2-1 (no MSD), V3 variants
        private val ST_PIDS = setOf(0x3748, 0x374B, 0x3752, 0x374E, 0x374F, 0x3753, 0x3754)
    }

    private val io = Executors.newSingleThreadExecutor()
    private val main = Handler(Looper.getMainLooper())
    private var connection: UsbDeviceConnection? = null
    private var iface: UsbInterface? = null
    private var epOut: UsbEndpoint? = null
    private var epIn: UsbEndpoint? = null
    private var openPid = 0
    private var pendingPermission: MethodChannel.Result? = null

    private val usbManager: UsbManager by lazy { getSystemService(Context.USB_SERVICE) as UsbManager }

    private val permissionReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action != ACTION_USB_PERMISSION) return
            val granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
            pendingPermission?.success(granted)
            pendingPermission = null
        }
    }

    private fun findDevice(): UsbDevice? =
        usbManager.deviceList.values.firstOrNull { it.vendorId == ST_VID && it.productId in ST_PIDS }

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)
        val filter = IntentFilter(ACTION_USB_PERMISSION)
        if (Build.VERSION.SDK_INT >= 33) {
            registerReceiver(permissionReceiver, filter, Context.RECEIVER_NOT_EXPORTED)
        } else {
            @Suppress("UnspecifiedRegisterReceiverFlag")
            registerReceiver(permissionReceiver, filter)
        }
        MethodChannel(flutterEngine.dartExecutor.binaryMessenger, CHANNEL).setMethodCallHandler { call, result ->
            when (call.method) {
                // -> null, or {name, vid, pid, permission}
                "find" -> {
                    val d = findDevice()
                    result.success(d?.let {
                        mapOf("name" to (it.productName ?: "ST-Link"), "vid" to it.vendorId, "pid" to it.productId,
                              "permission" to usbManager.hasPermission(it))
                    })
                }
                "requestPermission" -> {
                    val d = findDevice()
                    when {
                        d == null -> result.success(false)
                        usbManager.hasPermission(d) -> result.success(true)
                        else -> {
                            pendingPermission?.success(false)
                            pendingPermission = result
                            val intent = Intent(ACTION_USB_PERMISSION).setPackage(packageName)
                            val flags = if (Build.VERSION.SDK_INT >= 31) PendingIntent.FLAG_MUTABLE else 0
                            usbManager.requestPermission(d, PendingIntent.getBroadcast(this, 0, intent, flags))
                        }
                    }
                }
                "open" -> result.success(open())
                "close" -> { close(); result.success(true) }
                // bulkOut(data: ByteArray, timeout: Int) -> bytes written (or -1)
                "bulkOut" -> {
                    val data = call.argument<ByteArray>("data") ?: ByteArray(0)
                    val timeout = call.argument<Int>("timeout") ?: 1000
                    io.execute {
                        val c = connection; val ep = epOut
                        val n = if (c == null || ep == null) -1 else bulk(c, ep, data, data.size, timeout)
                        main.post { result.success(n) }
                    }
                }
                // bulkIn(length: Int, timeout: Int) -> ByteArray (shorter if less arrived) or null on error
                "bulkIn" -> {
                    val length = call.argument<Int>("length") ?: 0
                    val timeout = call.argument<Int>("timeout") ?: 1000
                    io.execute {
                        val c = connection; val ep = epIn
                        var out: ByteArray? = null
                        if (c != null && ep != null) {
                            val buf = ByteArray(length)
                            val n = bulk(c, ep, buf, length, timeout)
                            if (n >= 0) out = buf.copyOf(n)
                        }
                        main.post { result.success(out) }
                    }
                }
                "keepScreenOn" -> {
                    val on = call.arguments as? Boolean ?: false
                    if (on) window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                    else window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                    result.success(true)
                }
                else -> result.notImplemented()
            }
        }
    }

    /** Large transfers in pieces: some phones refuse more than 16 KB per bulkTransfer call. */
    private fun bulk(c: UsbDeviceConnection, ep: UsbEndpoint, buf: ByteArray, length: Int, timeout: Int): Int {
        var done = 0
        while (done < length) {
            val chunk = minOf(16384, length - done)
            val n = c.bulkTransfer(ep, buf, done, chunk, timeout)
            if (n < 0) return if (done > 0) done else -1
            done += n
            if (ep.direction == UsbConstants.USB_DIR_IN && n < chunk) break // short packet = end of data
        }
        return done
    }

    /** -> {ok, pid, error} */
    private fun open(): Map<String, Any> {
        close()
        val d = findDevice() ?: return mapOf("ok" to false, "error" to "no ST-Link connected")
        if (!usbManager.hasPermission(d)) return mapOf("ok" to false, "error" to "no USB permission")
        val c = usbManager.openDevice(d) ?: return mapOf("ok" to false, "error" to "could not open the ST-Link")
        // the debug interface is the one with bulk endpoints (V2: OUT 0x02 / IN 0x81, V2-1/V3: OUT 0x01 / IN 0x81)
        for (i in 0 until d.interfaceCount) {
            val itf = d.getInterface(i)
            var o: UsbEndpoint? = null
            var inn: UsbEndpoint? = null
            for (e in 0 until itf.endpointCount) {
                val ep = itf.getEndpoint(e)
                if (ep.type != UsbConstants.USB_ENDPOINT_XFER_BULK) continue
                if (ep.direction == UsbConstants.USB_DIR_OUT && o == null) o = ep
                if (ep.direction == UsbConstants.USB_DIR_IN && (inn == null || ep.address < inn.address)) inn = ep
            }
            if (o != null && inn != null) {
                if (!c.claimInterface(itf, true)) {
                    c.close()
                    return mapOf("ok" to false, "error" to "could not claim the USB interface")
                }
                connection = c; iface = itf; epOut = o; epIn = inn; openPid = d.productId
                return mapOf("ok" to true, "pid" to d.productId)
            }
        }
        c.close()
        return mapOf("ok" to false, "error" to "no bulk endpoints (not an ST-Link?)")
    }

    private fun close() {
        val c = connection
        if (c != null) {
            iface?.let { c.releaseInterface(it) }
            c.close()
        }
        connection = null; iface = null; epOut = null; epIn = null
    }

    override fun onDestroy() {
        close()
        try { unregisterReceiver(permissionReceiver) } catch (_: Exception) {}
        io.shutdown()
        super.onDestroy()
    }
}
