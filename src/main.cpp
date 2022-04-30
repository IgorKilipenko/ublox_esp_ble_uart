//#include <Arduino.h>
#include <header.h>
#include "Queue.h"
#include "BluetoothSerial.h"

HardwareSerial *RTCM{&Serial1};
HardwareSerial *Receiver{&Serial2};

std::thread receiverThread;

using UartBuffer = Queue<std::vector<uint8_t>>;
// UartBuffer queue_buffer{500};

const std::vector<uint8_t> readReceiverData()
{
    int count = 0;
    if ((count = Receiver->available()) > 0)
    {
        std::vector<uint8_t> buffer(count);
        int len = Receiver->readBytes(buffer.data(), count);
        if (len != count)
        {
            log_e("Not all bytes read from uart, available: [%i], read: [%i]\n", count, len);
        }
        if (len > 0)
        {
            // queue_buffer.push(buffer);
            log_d("data count %d\n", buffer.size());
            return buffer;
        }
    }
    return std::vector<uint8_t>(0);
}

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

void setup()
{
    Serial.begin(BAUD_SERIAL);

    Receiver->begin(BAUND_RECEIVER, SERIAL_8N1, RXD2, TXD2, false, 20000UL, static_cast<uint8_t>(SERIAL_SIZE_RX));
    delay(1000);
    unsigned long detectedBaudRate = Receiver->baudRate();
    if (detectedBaudRate)
        log_d("Receiver uart baudrate detecyed -> [ %lu ]", detectedBaudRate);
    else
    {
        log_w("Receiver baudrate not detected");
        Receiver->begin(BAUD_SERIAL, SERIAL_8N1, RXD2, TXD2, false, 20000UL, SERIAL_SIZE_RX);
    }

    RTCM->begin(38400, SERIAL_8N1, RXD1, TXD1);

    // Receiver->setRxBufferSize(SERIAL_SIZE_RX);

    SerialBT.begin("ESP32test"); // Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");
}

const size_t MAX_BUFFER_SIZE = 1024*3;
std::vector<uint8_t> buffer(MAX_BUFFER_SIZE * 2);
int tryCount = 0;

void loop()
{
    if (SerialBT.hasClient())
    {
        const size_t bufferSize = buffer.size();
        if ((bufferSize >= MAX_BUFFER_SIZE - SERIAL_SIZE_RX) || (bufferSize > 0 && tryCount > 100))
        {
            //log_d("BufferSize = %lu, tryCount = %d", bufferSize, tryCount);
            const size_t writeCount = SerialBT.write(buffer.data(), bufferSize);
            log_d("Write %lu bytes from %lu", writeCount, bufferSize);
            int partSize = bufferSize - writeCount;
            if (partSize <= 0) {
                tryCount = 0;
                buffer.clear();
            }
            else {
                log_d("WARN BT Buffer overflow = %lu, tryCount = %d", partSize, tryCount);
                std::vector<uint8_t> bufferOld(partSize);
                for (int i = partSize - 1; i < bufferSize; i++)
                {
                    bufferOld.push_back(buffer[i]);
                }

                buffer.clear();
                const size_t oldBufferSize = bufferOld.size();
                for (int i = 0; i < oldBufferSize; i++)
                {
                    buffer.push_back(bufferOld[i]);
                }
            }
            if (writeCount >= MAX_BUFFER_SIZE  - SERIAL_SIZE_RX) {
                delay(10);
                SerialBT.flush();
            }else {
                delay(1);
            }
        }
        else
        {
            if (bufferSize > 0)
            {
                tryCount++;
                SerialBT.flush();
            }
        }

        const int len = Receiver->available();
        if (len)
        {
            // SerialBT.write(Serial.read());
            uint8_t bytes[len];
            const size_t sizeRes = Receiver->readBytes(bytes, len);
            for (int i = 0; i < sizeRes; i++)
            {
                buffer.push_back(bytes[i]);
            }
        }
        if (SerialBT.available())
        {
            Serial.write(SerialBT.read());
            delay(10);
        }
    }
    else
    {
        if (buffer.size() > 0)
        {
            buffer.clear();
            tryCount = 0;
        }
    }
    delay(1);
}