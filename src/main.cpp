//#include <Arduino.h>
#include <header.h>
#include <queue>
#include "BluetoothSerial.h"

HardwareSerial *RTCM{&Serial1};
HardwareSerial *Receiver{&Serial2};


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

void receiveFnc() {
    log_d("Received UART Data. Available For Reading = %u\n", Receiver->available());
    while(Receiver->available()) Receiver->read(); // Discard received data.
}

void receiveErrorFnc(hardwareSerial_error_t error){
    log_e("UART Reception Error: ");
    switch(error) {
        case UART_BREAK_ERROR:
            log_e("UART_BREAK_ERROR\n");
            break;
        case UART_BUFFER_FULL_ERROR:
            log_e("UART_BUFFER_FULL_ERROR\n");
            break;
        case UART_FIFO_OVF_ERROR:
            log_e("UART_FIFO_OVF_ERROR\n");
            break;
        case UART_FRAME_ERROR:
            log_e("UART_FRAME_ERROR\n");
            break;
        case UART_PARITY_ERROR:
            log_e("UART_PARITY_ERROR\n");
            break;
    }
}

const size_t MAX_BUFFER_SIZE = 1024 * 2;
std::vector<uint8_t> buffer(MAX_BUFFER_SIZE * 2);
int tryCount = 0;

void onGnssReceiveCb()
{
    if (!SerialBT.hasClient()) {
        while(Receiver->available()) Receiver->read();
        return;
    }
    const int len = Receiver->available();
    if (len)
    {
        //uint8_t bytes[len];
        //const size_t readSize = Receiver->readBytes(bytes, len);

        //const size_t writeCount = SerialBT.write(bytes, readSize);
        //log_d("[ %lu ] bytes from GNSS[incom : %lu bytes] sent to BT SERIAL.\n", writeCount, len);
        //delay(1);

        /*while(Receiver->available()) SerialBT.write(Receiver->read());
        delay(10);*/
        /*for (int i = 0; i < readSize; i++){
            buffer.push_back(bytes[i]);
        }*/

        while(Receiver->available()) buffer.push_back(Receiver->read());
        
    }
}

void onRtcmReceiveCb()
{
    /*if (!SerialBT.hasClient())
    {
        return;
    }

    const int len = RTCM->available();
    if (len)
    {
        uint8_t bytes[len];
        const size_t readSize = RTCM->readBytes(bytes, len);
        const size_t writeCount = SerialBT.write(bytes, readSize);
        log_d("[ %lu ] bytes from RTCM[incom : %lu bytes] sent to BT SERIAL.\n", writeCount, len);
    }*/
}

void setup()
{
    Serial.begin(BAUD_SERIAL);

    Receiver->setRxBufferSize(1024*4);
    Receiver->onReceive(onGnssReceiveCb, true); 
    Receiver->onReceiveError(receiveErrorFnc);
    //Receiver->setRxTimeout(1);

    Receiver->begin(BAUND_RECEIVER, SERIAL_8N1, RXD2, TXD2/*, false, 20000UL, static_cast<uint8_t>(SERIAL_SIZE_RX)*/);
    //delay(500);
    unsigned long detectedBaudRate = Receiver->baudRate();
    if (detectedBaudRate)
        log_d("Receiver uart baudrate detecyed -> [ %lu ]", detectedBaudRate);
    else
    {
        log_w("Receiver baudrate not detected");
        Receiver->begin(BAUD_SERIAL, SERIAL_8N1, RXD2, TXD2/*, false, 20000UL, SERIAL_SIZE_RX*/);
    }
    // Receiver->setHwFlowCtrlMode((uint8_t)3U, SERIAL_SIZE_RX);

    RTCM->begin(38400, SERIAL_8N1, RXD1, TXD1);

    // Receiver->setRxBufferSize(SERIAL_SIZE_RX);

    SerialBT.begin("ESP32test"); // Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");

    /*if (Receiver != NULL)
    {
        Receiver->onReceive(onGnssReceiveCb, true);
    }
    if (RTCM != NULL)
    {
        RTCM->onReceive(onRtcmReceiveCb, true);
    }*/
}



void loop()
{
    
    if (SerialBT.hasClient())
    {
        const size_t bufferSize = buffer.size();
        if ((bufferSize >= MAX_BUFFER_SIZE - SERIAL_SIZE_RX) || (bufferSize > 0 && tryCount > 100))
        {
            // log_d("BufferSize = %lu, tryCount = %d", bufferSize, tryCount);
            const size_t writeCount = SerialBT.write(buffer.data(), bufferSize);
            log_d("Write %lu bytes from %lu", writeCount, bufferSize);
            int partSize = bufferSize - writeCount;
            if (partSize <= 0)
            {
                tryCount = 0;
                buffer.clear();
            }
            else
            {
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
            if (writeCount >= MAX_BUFFER_SIZE - SERIAL_SIZE_RX)
            {
                delay(10);
                SerialBT.flush();
            }
            else
            {
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

        const int inLen = SerialBT.available();
        if (inLen)
        {
            // Serial.write(SerialBT.read());
            uint8_t bytes[inLen];
            const size_t sizeRes = SerialBT.readBytes(bytes, inLen);
            const size_t writeCount = Receiver->write(bytes, inLen);
            if (writeCount > 0)
            {
                delay(10);
            }
            else
            {
                delay(1);
            }
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