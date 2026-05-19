package com.example.bikemonitor

import android.bluetooth.BluetoothDevice
import android.content.Context
import no.nordicsemi.android.ble.BleManager
import no.nordicsemi.android.ble.ktx.asValidUuid
import java.util.*

class BikeBleManager(context: Context) : BleManager(context) {
    // UUID muszą być identyczne jak w Arduino!
    private val serviceUuid = "19B10000-E8F2-537E-4F6C-D104768A1214".asValidUuid()
    private val dataUuid = "19B10001-E8F2-537E-4F6C-D104768A1214".asValidUuid()

    private var dataCharacteristic: android.bluetooth.BluetoothGattCharacteristic? = null

    // To wywoła się, gdy znajdziemy usługi Arduino
    override fun isRequiredServiceSupported(gatt: android.bluetooth.BluetoothGatt): Boolean {
        val service = gatt.getService(serviceUuid)
        dataCharacteristic = service?.getCharacteristic(dataUuid)
        return dataCharacteristic != null
    }

    // Tutaj zapisujemy się na "powiadomienia" (Notify)
    override fun initialize() {
        beginAtomicCondition()
            .add(setNotificationCallback(dataCharacteristic)
                .with { _, data ->
                    val text = data.stringValue ?: ""
                    // Tutaj będziemy przetwarzać tekst na liczby
                    onDataReceived(text)
                })
            .add(enableNotifications(dataCharacteristic))
            .enqueue()
    }

    override fun onDeviceDisconnected() {
        dataCharacteristic = null
    }

    // Funkcja którą nadpiszemy w UI, żeby dostawać dane
    var onDataReceived: (String) -> Unit = {}
}