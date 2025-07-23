package sk.uniba.krucena

import android.content.Context
import android.net.wifi.WifiManager
import android.os.Handler
import android.os.Looper
import android.util.Log
import java.io.InputStream
import java.io.OutputStream
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.Inet4Address
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.net.Socket
import java.nio.ByteBuffer
import java.nio.channels.SelectionKey
import java.nio.channels.Selector
import java.nio.channels.ServerSocketChannel
import java.nio.channels.SocketChannel
import java.util.concurrent.Executors

class Communication(mainActivity: MainActivity) {

    private val MESSAGE_LOGIN : Byte = 1
    private val MESSAGE_START : Byte = 2
    private val MESSAGE_EMERGENCY : Byte = 3

    private final val DATAGRAM_MASTER_IP_ANNOUNCE_PORT : Int = 8993

    private val activity = mainActivity

    val dronesConnected : MutableMap<Int, SocketChannel> = HashMap()

    private fun getHotspotIP(): String? {
        val wifiManager = activity.applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        val dhcp = wifiManager.dhcpInfo
        val gatewayIp = dhcp.gateway
        return String.format(
            "%d.%d.%d.%d",
            gatewayIp and 0xff,
            gatewayIp shr 8 and 0xff,
            gatewayIp shr 16 and 0xff,
            gatewayIp shr 24 and 0xff
        )
    }

    private fun getBroadcastedIP(): String? {
        val socket = DatagramSocket(null)
        socket.reuseAddress = true
        socket.bind(InetSocketAddress(DATAGRAM_MASTER_IP_ANNOUNCE_PORT))

        val buffer = ByteArray(64)
        val packet = DatagramPacket(buffer, buffer.size)
        Log.i("COMMDBG", "waiting datagram")
        socket.receive(packet)
        val msg = String(packet.data, 0, packet.length)
        if (msg.startsWith("MASTER_IP:")) {
            val ip = msg.removePrefix("MASTER_IP:")
            Log.i("COMMDBG", "Determined master IP from broadcast as '${ip}")
            return ip
        }
        Log.w("COMMDBG", "broadcast from server unrecognized content: ${msg}")
        return null
    }

    fun setupCommunication()
    {
        if (activity.config.isServer && (activity.config.expectedNumberOfDrones > 1)) communicatingServer()
        else connectToServer()
    }

    private fun readChannelToBuffer(buffer : ByteBuffer, client: SocketChannel) : Boolean
    {
        val timeoutDelay = 1000
        var bytesRead = 0
        var moreBytesRead = 0
        val toRead = buffer.capacity()
        val tm1 = System.currentTimeMillis()
        try {
            while (bytesRead < toRead) {
                moreBytesRead = client.read(buffer)
                if (moreBytesRead == -1) {
                    Log.i("COMMDBG", "readChannelToBuffer no more bytes")
                    break
                } else if (moreBytesRead == 0) Thread.sleep(10)
                bytesRead += moreBytesRead
                if (System.currentTimeMillis() - tm1 > timeoutDelay) {
                    Log.i("COMMDBG", "readChannelToBuffer timeout")
                    return false
                };
            }
            if (moreBytesRead == -1) {
                client.close()
                return false
            }
            return true
        } catch (e: Exception) {
            Log.e("COMMDBG", "readChannelToBuffer has exception: $e ; ${e.message}")
            return false
        }
    }

    private fun readStreamToBuffer(buffer : ByteBuffer, client: InputStream) : Boolean
    {
        var bytesRead = 0
        var moreBytesRead = 0
        val toRead = buffer.capacity()
        val arrayBuf = ByteArray(toRead)
        try {
            while (bytesRead < toRead) {
                Log.i("COMMDBG", "reading " + (toRead - bytesRead) + " bytes")
                moreBytesRead = client.read(arrayBuf, bytesRead, toRead - bytesRead)
                if (moreBytesRead == -1) {
                    Log.i("COMMDBG", "readStreamToBuffer no more bytes")
                    break
                } else if (moreBytesRead == 0) Thread.sleep(10)
                bytesRead += moreBytesRead
            }
            if (moreBytesRead == -1) {
                client.close()
                return false
            }
            buffer.put(arrayBuf)
            return true
        } catch (e: Exception) {
            Log.e("COMMDBG", "readStreamToBuffer has exception: $e ; ${e.message}")
            return false
        }
    }

    private fun processClientPacket(buffer: ByteBuffer, clientChannel: SocketChannel)
    {
        Log.i("COMMDBG", "Received packet of len: ${buffer.capacity()}")
        val packetType : Byte = buffer.get()
        if (packetType == MESSAGE_LOGIN)
        {
            val newDroneId = buffer.getInt()
            dronesConnected.put(newDroneId, clientChannel)
            Log.i("COMMDBG", "New drone connected: ${newDroneId}")
        }
    }

    private fun sendPacketToClient(buffer: ByteBuffer, channel: SocketChannel)
    {
        try {
            Log.i("COMMDBG", "Sending packet of len: ${buffer.position()} to client, size")
            val size = buffer.position()
            val sizeBuffer = ByteBuffer.allocate(4)
            sizeBuffer.putInt(size)
            sizeBuffer.flip()
            channel.write(sizeBuffer)

            Log.i("COMMDBG", "Sending packet of len: ${size} to client, data")
            buffer.flip()
            channel.write(buffer)
        } catch (e: Exception) {
            Log.e("COMMDBG", "sendPacketToClient has exception: $e ; ${e.message}")
        }
    }

    private fun sendPacketToServer(buffer: ByteBuffer, outputStream: OutputStream)
    {
        try {
            Log.i("COMMDBG", "Sending packet of len: ${buffer.position()}, size")
            val size = buffer.position()
            val sizeBuffer = ByteBuffer.allocate(4)
            sizeBuffer.putInt(size)
            sizeBuffer.flip()
            outputStream.write(sizeBuffer.array())

            Log.i("COMMDBG", "sending packet to server, ${size} data...")
            val outArray = ByteArray(size)
            buffer.flip()
            buffer.get(outArray)
            outputStream.write(outArray)
            outputStream.flush()
        } catch (e: Exception) {
            Log.e("COMMDBG", "sendPacketToServer has exception: $e ; ${e.message}")
        }
    }

    private fun receivePacketFromServer(inStream : InputStream) : ByteBuffer?
    {
        try {
            Log.i("COMMDBG", "receiving packet from server, size...")
            val lenBuffer = ByteBuffer.allocate(4)
            if (!readStreamToBuffer(lenBuffer, inStream)) return null
            lenBuffer.flip()
            val packetSize = lenBuffer.getInt()

            Log.i("COMMDBG", "receiving packet from server, size=${packetSize}...")
            val buffer = ByteBuffer.allocate(packetSize)
            if (!readStreamToBuffer(buffer, inStream)) return null
            buffer.flip()
            return buffer
        } catch (e: Exception) {
            Log.e("COMMDBG", "receivePacketFromServer has exception: $e ; ${e.message}")
            return null
        }
    }

    private fun processPacketFromServer(packet : ByteBuffer)
    {
        val packetType : Byte = packet.get()
        if (packetType == MESSAGE_START)
        {
            Log.i("COMMDBG", "processing message start from server...")
            val danceNumber : Int = packet.get().toInt()
            activity.startChoreography(danceNumber)
        }
        else if (packetType == MESSAGE_EMERGENCY)
        {
            Log.i("COMMDBG", "processing message emergency from server...")
            activity.emergency = true
        }
    }

    fun getLocalIpAddress(): String? {
        try {
            val interfaces = NetworkInterface.getNetworkInterfaces()
            for (intf in interfaces) {
                val addrs = intf.inetAddresses
                for (addr in addrs) {
                    if (!addr.isLoopbackAddress && addr is Inet4Address) {
                        return addr.hostAddress
                    }
                }
            }
        } catch (ex: Exception) {
            //Log.e("COMMDBG", "exception in getLocalIpAddress(). ${ex.message}, stack=${ex.stackTraceToString()}")
            ex.printStackTrace()
        }
        return null
    }

    private fun startBroadcastingIP()
    {
        Thread {
            Log.i("COMMDBG", "broadcasting IP thread")
            val myIpAddress = getLocalIpAddress()
            Log.i("COMMDBG", "Master determined its IP as ${myIpAddress}")

            while (activity.guiState == GUIState.READY_TO_RUN) {
                try {
                    val socket = DatagramSocket()
                    socket.broadcast = true
                    val address = InetAddress.getByName("255.255.255.255")
                    val message = "MASTER_IP:${myIpAddress}"

                    val packet =
                        DatagramPacket(message.toByteArray(), message.length, address, DATAGRAM_MASTER_IP_ANNOUNCE_PORT)
                    Log.i("COMMDBG", "sending...")
                    socket.send(packet)
                } catch (e: Exception) {
                    Log.e("COMMDBG", "IP broadcasting thread: $e ; ${e.message}")
                }
                Thread.sleep(1000)
            }
        }.start()
    }

    private fun communicatingServer()
    {
        startBroadcastingIP()
        val selector = Selector.open()
        val serverChannel = ServerSocketChannel.open()
        serverChannel.bind(InetSocketAddress("0.0.0.0", 2339))
        serverChannel.configureBlocking(false)
        serverChannel.register(selector, SelectionKey.OP_ACCEPT)

        Thread {
            Log.i("COMMDBG", "thread()")

            while (true) {
                try {
                    Log.i("COMMDBG", "waiting for connections and msgs...")
                    selector.select()
                    val selectedKeys = selector.selectedKeys().iterator()

                    while (selectedKeys.hasNext()) {
                        val key = selectedKeys.next()
                        selectedKeys.remove()

                        if (key.isAcceptable) {
                            val server = key.channel() as ServerSocketChannel
                            Log.i("COMMDBG", "accepting...")
                            val client = server.accept()
                            client.configureBlocking(false)
                            client.socket().setTcpNoDelay(true);
                            client.register(selector, SelectionKey.OP_READ)
                            Log.i("COMMDBG", "Accepted connection from ${client.remoteAddress}")
                        }

                        if (key.isReadable) {
                            Log.i("COMMDBG", "reading...")
                            val client = key.channel() as SocketChannel

                            val lenBuffer = ByteBuffer.allocate(4)
                            if (!readChannelToBuffer(lenBuffer, client)) continue
                            lenBuffer.flip()
                            val packetSize = lenBuffer.getInt()
                            Log.i("COMMDBG", "reading packet of size {$packetSize}...")

                            val buffer = ByteBuffer.allocate(packetSize)
                            if (!readChannelToBuffer(buffer, client)) continue
                            buffer.flip()
                            processClientPacket(buffer, client)
                        }
                    }
                } catch (e: Exception) {
                        Log.e("COMMDBG", "Server main network loop terminates: $e ; ${e.message}")
                }
            }
        }.start()
    }

    class NoIssueException: Exception("Leaving catch block seamlessly")

    fun startPlayingMusic(danceNumber : Int, onDone: () -> Unit)
    {
        if (activity.config.musicServerIP == "none")
        {
            Log.w("COMMDBG", "Skipping playback: musicServerIP is 'none'")
            return
        }
        Log.i("COMMDBG", "startPlayingMusic(${1 + danceNumber}) ${activity.config.musicServerIP}")
        Thread {
            try {
                val socket = Socket()
                socket.connect(
                    InetSocketAddress(
                        activity.config.musicServerIP,
                        activity.config.musicServerPort
                    ), 5000
                )
                if (!socket.isConnected) throw NoIssueException()

                val outStream = socket.getOutputStream()

                outStream.write("clear\n".toByteArray())
                outStream.flush()

                outStream.write("add tanec${1 + danceNumber}.mp3\n".toByteArray())
                outStream.flush()

                outStream.write("play\n".toByteArray())
                outStream.flush()

                socket.close()
            } catch (e1: NoIssueException) {
                Log.i("COMMDBG", "startPlayingMusic() socket connect timeout")
            } catch (e: Exception) {
                Log.e(
                    "COMMDBG",
                    "Failed to connect or communicate with music server: $e ; ${e.message}"
                )
            }
            finally {
                Handler(Looper.getMainLooper()).post {
                    onDone()
                }
            }
        }.start()
    }

    fun connectToServer() {

        Thread {
            try {
                Log.i("COMMDBG", "connectToServer() ${activity.config.obtainServerIP}")
                val serverIp = if (activity.config.obtainServerIP == activity.config.fromHotspot)
                    getHotspotIP()
                    else getBroadcastedIP()

                Log.i("COMMDBG", "connectToServer() serverIP=$serverIp")

                val socket = Socket()
                Log.i("COMMDBG", "connectToServer() socket")

                socket.connect(InetSocketAddress(serverIp, 2339), 5000)
                Log.i("COMMDBG", "connectToServer() connect")
                if (!socket.isConnected) {
                    Log.i("COMMDBG", "connectToServer() timeout")
                    throw NoIssueException()
                }
                val outStream = socket.getOutputStream()
                val inStream = socket.getInputStream()

                Log.i("COMMDBG", "connectToServer() sending login message...")
                // Send message
                val msg = ByteBuffer.allocate(5)
                msg.put(MESSAGE_LOGIN)
                msg.putInt(activity.config.droneId)
                sendPacketToServer(msg, outStream)

                while (true) {
                    val packet : ByteBuffer? = receivePacketFromServer(inStream)
                    if (packet == null) break
                    processPacketFromServer(packet)
                }
                Log.i("COMMDBG", "client closing connection to server...")

                socket.close()
            } catch (e1: NoIssueException) {}
            catch (e: Exception) {
                Log.e("COMMDBG", "Failed to connect or communicate with server: $e ; ${e.message}")
            }
        }.start()
    }

    fun sendStartSignalToAllDronesNow(danceToStart : Int)
    {
        val executor = Executors.newCachedThreadPool()
        for (channel in dronesConnected)
        {
            executor.execute {
                Log.i("COMMDBG", "sending start packet to a client ${channel.key}...")
                val startPacket : ByteBuffer = ByteBuffer.allocate(2)
                startPacket.put(MESSAGE_START)
                startPacket.put(danceToStart.toByte())
                Log.d("COMMDBG", "Channel open: ${channel.value.isOpen}, connected: ${channel.value.isConnected}")
                sendPacketToClient(startPacket, channel.value)
            }
        }
        activity.startChoreography(danceToStart)
    }

    fun sendStartSignalWhenReady(danceToStart : Int)
    {
        val handler = Handler(Looper.getMainLooper())
        handler.post(object : Runnable {
            override fun run() {
                if (dronesConnected.size < activity.config.expectedNumberOfDrones - 1) {
                    handler.postDelayed(this, 300)
                } else {
                    handler.removeCallbacks(this)
                    sendStartSignalToAllDronesNow(danceToStart)
                }
            }
        })
    }

    fun sendEmergencySignal() {
        val executor = Executors.newCachedThreadPool()
        for (channel in dronesConnected) {
            executor.execute {
                Log.i("COMMDBG", "sending emergency packet to a client ${channel.key}...")
                val emergencyPacket: ByteBuffer = ByteBuffer.allocate(1)
                emergencyPacket.put(MESSAGE_EMERGENCY)
                Log.d(
                    "COMMDBG",
                    "Channel open: ${channel.value.isOpen}, connected: ${channel.value.isConnected}"
                )
                sendPacketToClient(emergencyPacket, channel.value)
            }
        }
    }
}