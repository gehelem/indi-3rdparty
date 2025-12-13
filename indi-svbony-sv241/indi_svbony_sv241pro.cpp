#include "indi_svbony_sv241pro.h"
#include "indicom.h"
#include "connectionplugins/connectionserial.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/ioctl.h>

// We declare an auto pointer to SvbonySV241P device class
static std::unique_ptr<SvbonySV241P> svbony_sv241p(new SvbonySV241P());

SvbonySV241P::SvbonySV241P() : INDI::WeatherInterface(this), INDI::PowerInterface(this)
{
    setVersion(1, 0);
}

bool SvbonySV241P::initProperties()
{
    INDI::DefaultDevice::initProperties();
    addAuxControls();

    setDriverInterface(AUX_INTERFACE | WEATHER_INTERFACE | POWER_INTERFACE);

    WI::initProperties(MAIN_CONTROL_TAB, MAIN_CONTROL_TAB);

    PI::SetCapability(POWER_HAS_DC_OUT | POWER_HAS_DEW_OUT | POWER_HAS_VARIABLE_OUT |
                      POWER_HAS_VOLTAGE_SENSOR | POWER_HAS_OVERALL_CURRENT | POWER_HAS_USB_TOGGLE);

    // Power Interface properties
    // 5 DC output, 2 DEW outputs, 1 Variable output, 1 Auto Dew ports (Global), 5 USB ports on 2 switches
    PI::initProperties(POWER_TAB, 5, 2, 1, 0, 2);

    VariableChannelVoltsNP[0].setMin(0);
    VariableChannelVoltsNP[0].setMax(15);
    VariableChannelVoltsNP[0].setStep(0.01);

    DewChannelDutyCycleNP[0].setMin(0);
    DewChannelDutyCycleNP[0].setMax(100);
    DewChannelDutyCycleNP[0].setStep(1);

    DewChannelDutyCycleNP[1].setMin(0);
    DewChannelDutyCycleNP[1].setMax(100);
    DewChannelDutyCycleNP[1].setStep(1);

    // Environment Group
    addParameter("WEATHER_TEMPERATURE", "Temperature (°C)", -15, 35, 15);
    addParameter("WEATHER_HUMIDITY", "Humidity (%)", 0, 100, 15);
    addParameter("WEATHER_DEWPOINT", "Dew Point (°C)", 0, 100, 15);
    addParameter("WEATHER_LENS_TEMPERATURE", "Lens Temperature (°C)", -15, 35, 15);
    setCriticalParameter("WEATHER_TEMPERATURE");

    serialConnection = new Connection::Serial(this);

    serialConnection->registerHandshake([&]()
    {
        return true;
    });

    registerConnection(serialConnection);

    return true;
}

bool SvbonySV241P::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
        WI::updateProperties();
        PI::updateProperties();
        setupComplete = true;
        sync();
        readVoltage();
        readPower();
        computeCurrent();
        readEnvironment();
        SetTimer(getCurrentPollingPeriod());

    }
    else
    {

        PI::updateProperties();
        WI::updateProperties();

        setupComplete = false;
    }

    return true;
}

const char * SvbonySV241P::getDefaultName()
{
    return "Svbony SV241 Pro";
}

bool SvbonySV241P::Connect()
{
    if (!openSerialPort())
    {
        LOG_INFO("Failed to open port");
        return false;
    }
    LOG_INFO("Connected");
    return true;
}

bool SvbonySV241P::Disconnect()
{
    closeSerialPort();
    LOG_INFO("Disconnected");
    return true;
}

bool SvbonySV241P::openSerialPort()
{
    const char *portName = serialConnection->port();
    // Open port with O_NOCTTY to prevent it from becoming controlling terminal
    PortFD = open(portName, O_RDWR | O_NOCTTY);
    if (PortFD < 0)
    {
        LOGF_ERROR("Error opening serial port %s: %s", portName, strerror(errno));
        return false;
    }

    // CRITICAL: Set DTR and RTS LOW immediately to prevent ESP32 reset
    // ESP32 boards with CH340/CP2102 use DTR+RTS for auto-reset during programming.
    // Opening the serial port can cause DTR to pulse HIGH, resetting the device.
    int modemBits = 0;
    ioctl(PortFD, TIOCMGET, &modemBits);
    modemBits &= ~TIOCM_DTR;  // Clear DTR (set LOW)
    modemBits &= ~TIOCM_RTS;  // Clear RTS (set LOW)
    ioctl(PortFD, TIOCMSET, &modemBits);

    // Clear non-blocking mode (ensure blocking reads with timeout)
    fcntl(PortFD, F_SETFL, 0);

    // Configure serial port: 115200 8N1
    struct termios options;
    tcgetattr(PortFD, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;       // 8 data bits
    options.c_cflag &= ~HUPCL;    // IMPORTANT: Disable HUPCL to prevent DTR drop on close
    options.c_cflag |= CLOCAL;    // Ignore modem control lines
    options.c_cflag |= CREAD;     // Enable receiver
    options.c_cflag &= ~CRTSCTS;  // Disable hardware flow control

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY);          // No software flow control
    options.c_oflag &= ~OPOST;                            // Raw output

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;  // 1 second timeout

    tcsetattr(PortFD, TCSANOW, &options);
    tcflush(PortFD, TCIOFLUSH);

    // Ensure DTR/RTS stay LOW after termios configuration
    ioctl(PortFD, TIOCMGET, &modemBits);
    modemBits &= ~TIOCM_DTR;
    modemBits &= ~TIOCM_RTS;
    ioctl(PortFD, TIOCMSET, &modemBits);

    // Allow USB serial device to stabilize
    usleep(500000);  // 500ms delay

    LOGF_INFO("Opened serial port %s at 115200 baud (DTR/RTS held LOW)", portName);
    return true;
}

void SvbonySV241P::closeSerialPort()
{
    if (PortFD >= 0)
    {
        close(PortFD);
        PortFD = -1;
        LOG_INFO("Closed serial port");
    }
}

bool SvbonySV241P::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev && !strcmp(dev, getDeviceName()))
    {
        if (PI::processSwitch(dev, name, states, names, n))
            return true;
    }
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool SvbonySV241P::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev && !strcmp(dev, getDeviceName()))
    {
        if (PI::processNumber(dev, name, values, names, n))
            return true;
    }
    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool SvbonySV241P::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev && !strcmp(dev, getDeviceName()))
    {
        if (PI::processText(dev, name, texts, names, n))
            return true;
    }
    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

uint8_t SvbonySV241P::calcChecksum(const uint8_t *packet)
{
    int checksum = 0;
    for (int i = 0; i < 5; i++)
    {
        checksum += packet[i];
    }
    if (checksum > 255)
    {
        checksum = checksum % 255;
    }
    return static_cast<uint8_t>(checksum);
}

bool  SvbonySV241P::sendCommand(Targets target, PowerPorts port, uint8_t value)
{
    if (PortFD < 0)
        return false;
    uint8_t packet[SEND_LENGTH];
    packet[0] = START_BYTE;
    packet[1] = SEND_LENGTH;
    packet[2] = target;
    packet[3] = port;
    packet[4] = value;

    // Calculate checksum
    packet[SEND_LENGTH - 1] = calcChecksum(packet);
    char hexLog[32];
    sprintf(hexLog, "%02X %02X %02X %02X %02X %02X",
            packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);
    LOGF_DEBUG("TX SVBONY -> <%s>", hexLog);

    tcflush(PortFD, TCIFLUSH);

    size_t totalWritten = 0;
    while (totalWritten < 6)
    {
        ssize_t written = write(PortFD, packet + totalWritten, SEND_LENGTH - totalWritten);
        if (written < 0)
        {
            LOGF_ERROR("Write error: %s", strerror(errno));
            return false;
        }
        if (written == 0)
        {
            LOG_ERROR("Write returned 0 bytes");
            return false;
        }
        totalWritten += written;
    }
    usleep(CMD_DELAY);

    return true;
}

bool SvbonySV241P::readResponse(uint8_t *response, size_t len, Targets expectedCmd)
{
    if (PortFD < 0)
        return false;

    struct pollfd pfd;
    pfd.fd = PortFD;
    pfd.events = POLLIN;

    size_t totalRead = 0;

    while (totalRead < len)
    {
        int pollResult = poll(&pfd, 1, READ_TIMEOUT);

        if (pollResult < 0)
        {
            LOGF_ERROR("Error polling serial port: %s.", strerror(errno));
            tcflush(PortFD, TCIOFLUSH);
            return false;
        }

        if (pollResult == 0)
        {
            LOG_ERROR("Timeout reading serial port.");
            break;
        }

        ssize_t bytesRead = read(PortFD, response + totalRead, len - totalRead);

        if (bytesRead < 0)
        {
            LOGF_ERROR("Error reading serial port: %s.", strerror(errno));
            tcflush(PortFD, TCIOFLUSH);
            return false;
        }

        if (bytesRead == 0)
        {
            // EOF - connection closed
            LOG_ERROR("Read returned 0 bytes (EOF)");
            break;
        }

        totalRead += bytesRead;

    }

    if (totalRead > 0)
    {
        LOGF_DEBUG("RX (%zu bytes): %02X %02X %02X %02X %02X %02X %02X %02X",
                   totalRead,
                   totalRead > 0 ? response[0] : 0,
                   totalRead > 1 ? response[1] : 0,
                   totalRead > 2 ? response[2] : 0,
                   totalRead > 3 ? response[3] : 0,
                   totalRead > 4 ? response[4] : 0,
                   totalRead > 5 ? response[5] : 0,
                   totalRead > 6 ? response[6] : 0,
                   totalRead > 7 ? response[7] : 0);
    }

    if (totalRead != len)
    {
        LOGF_ERROR("Expected %zu bytes, got %zu", len, totalRead);
        tcflush(PortFD, TCIOFLUSH);
        return false;
    }

    if (response[2] != expectedCmd)
    {
        LOGF_ERROR("Unexpected response command: expected %02X, got %02X", expectedCmd, response[1]);
        tcflush(PortFD, TCIOFLUSH);
        return false;
    }

    return true;
}

bool SvbonySV241P::sync()
{
    if (!sendCommand(SYNC))
    {
        return false;
    }
    uint8_t packet[RECV_SYNC_LENGTH];
    if (!readResponse(packet, RECV_SYNC_LENGTH, SYNC))
    {
        return false;
    }
    // Parse DC states (bytes 3 - 7)
    for (int i = 0; i < 5; i++)
    {
        PowerChannelsSP[i].setState(packet[3 + i] ? ISS_ON : ISS_OFF);
    }

    USBPortSP[0].setState(packet[5] ? ISS_ON : ISS_OFF);
    USBPortSP[1].setState(packet[6] ? ISS_ON : ISS_OFF);

    VariableChannelVoltsNP[0].setValue(15.3 * packet[7] / 253);

    DewChannelDutyCycleNP[0].setValue(100 * packet[8] / 255);
    DewChannelDutyCycleNP[1].setValue(100 * packet[9] / 255);

    //apply and IPS_OK
    PowerChannelsSP.setState(IPS_OK);
    PowerChannelsSP.apply();
    USBPortSP.setState(IPS_OK);
    USBPortSP.apply();
    VariableChannelVoltsNP.setState(IPS_OK);
    VariableChannelVoltsNP.apply();
    DewChannelDutyCycleNP.setState(IPS_OK);
    DewChannelDutyCycleNP.apply();

    return true;
}

bool SvbonySV241P::readVoltage()
{
    if(!sendCommand(VOLTAGE))
    {
        return false;
    }

    uint8_t packet[RECV_LENGTH];
    if(!readResponse(packet, RECV_LENGTH, VOLTAGE))
    {
        return false;
    }
    uint8_t data[4] = {packet[6], packet[5], packet[4], packet[3]};
    int32_t rawValue;
    memcpy(&rawValue, data, 4);
    double voltage = (rawValue / 100.0);
    PowerSensorsNP[SENSOR_VOLTAGE].setValue(voltage);
    PowerSensorsNP.apply();
    return true;
}

bool SvbonySV241P::readPower()
{
    if(!sendCommand(CURRENT))
    {
        return false;
    }
    uint8_t packet[RECV_LENGTH];
    if(!readResponse(packet, RECV_LENGTH, CURRENT))
    {
        return false;
    }
    double voltage = PowerSensorsNP[SENSOR_VOLTAGE].getValue();
    uint8_t data[4] = {packet[6], packet[5], packet[4], packet[3]};
    int32_t rawValue;
    memcpy(&rawValue, data, 4);
    double power = (15.57 * (voltage * rawValue/100.0) + 269.39) / 1000.0;
    PowerSensorsNP[SENSOR_POWER].setValue(power);
    PowerSensorsNP.apply();
    return true;
}

bool SvbonySV241P::computeCurrent()
{
    double power = PowerSensorsNP[SENSOR_POWER].getValue();
    double voltage = PowerSensorsNP[SENSOR_VOLTAGE].getValue();
    double current = ((power * 1000.0) - 269.39) / (15.57 * voltage);
    PowerSensorsNP[SENSOR_CURRENT].setValue(current);
    return true;
}

bool SvbonySV241P::readEnvironment()
{
    if(!sendCommand(TEMPERATURE))
    {
        return false;
    }
    uint8_t packet[RECV_LENGTH];
    if(!readResponse(packet, RECV_LENGTH, TEMPERATURE))
    {
        return false;
    }
    uint8_t data[4] = {packet[6], packet[5], packet[4], packet[3]};
    uint32_t rawValue;
    memcpy(&rawValue, data, 4);
    double temperature = (rawValue / 100.0) + TEMP_OFFSET;
    setParameterValue("WEATHER_TEMPERATURE", temperature);
    //Humidity
    if(!sendCommand(HUMIDITY))
    {
        return false;
    }
    if(!readResponse(packet, RECV_LENGTH, HUMIDITY))
    {
        return false;
    }
    data[0] = packet[6];
    data[1] = packet[5];
    data[2] = packet[4];
    data[3] = packet[3];
    memcpy(&rawValue, data, 4);
    double humidity = (rawValue / 100.0)  + HUM_OFFSET;
    setParameterValue("WEATHER_HUMIDITY", humidity);
    //Dewpoint
    double dewpoint = (243.04 * (log(humidity / 100.0) + (17.625 * temperature / (243.04 + temperature))) / (17.625 - log(
                           humidity / 100.0) - (17.625 * temperature / (243.04 + temperature))));
    setParameterValue("WEATHER_DEWPOINT", dewpoint);
    // Lens Temperature
    if(!sendCommand(LENS_TEMP))
    {
        return false;
    }
    if(!readResponse(packet, RECV_LENGTH, LENS_TEMP))
    {
        return false;
    }
    data[0] = packet[6];
    data[1] = packet[5];
    data[2] = packet[4];
    data[3] = packet[3];
    memcpy(&rawValue, data, 4);
    double lensTemperature = (rawValue / 100.0) + TEMP_OFFSET;
    setParameterValue("WEATHER_LENS_TEMPERATURE", lensTemperature);
    return true;
}

bool SvbonySV241P::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);

    WI::saveConfigItems(fp);
    PI::saveConfigItems(fp);
    return true;
}

void SvbonySV241P::TimerHit()
{
    if (!isConnected() || setupComplete == false)
    {
        SetTimer(getCurrentPollingPeriod());
        return;
    }

    readVoltage();
    readPower();
    computeCurrent();

    readEnvironment();

    SetTimer(getCurrentPollingPeriod());
}

bool SvbonySV241P::SetPowerPort(size_t port, bool enabled)
{
    PowerPorts DCPort;
    uint8_t stateValue = enabled ? 0xFF : 0x00;
    if (port == 0)
    {
        DCPort = DC_1;
    }
    else if (port == 1)
    {
        DCPort = DC_2;
    }
    else if (port == 2)
    {
        DCPort = DC_3;
    }
    else if (port == 3)
    {
        DCPort = DC_4;
    }
    else if (port == 4)
    {
        DCPort = DC_5;
    }
    else
    {
        return false;
    }
    if(!sendCommand(OUTPUT, DCPort, stateValue))
    {
        return false;
    }
    return true;

}

bool SvbonySV241P::SetDewPort(size_t port, bool enabled, double dutyCycle)
{
    uint8_t stateValue = enabled ? static_cast<uint8_t>(dutyCycle * 253 / 100) : 0x00;
    PowerPorts dewPort;
    if(port == 0)
    {
        dewPort = DEW_A;
    }
    else if(port == 1)
    {
        dewPort = DEW_B;
    }
    else
    {
        return false;
    }
    if(!sendCommand(OUTPUT, dewPort, stateValue))
    {
        return false;
    }
    return true;
}

bool SvbonySV241P::SetVariablePort(size_t port, bool enabled, double voltage)
{
    INDI_UNUSED(port);

    uint8_t stateValue = enabled ? static_cast<uint8_t>(voltage * 253 / 100) : 0x00;
    if(!sendCommand(OUTPUT, ADJ, stateValue))
    {
        return false;
    }
    return true;
}

bool SvbonySV241P::SetUSBPort(size_t port, bool enabled)
{
    PowerPorts USBPort;
    uint8_t stateValue = enabled ? 0xFF : 0x00;
    if (port == 0)
    {
        USBPort = USB_C12;
    }
    else if (port == 1)
    {
        USBPort = USB_345;
    }
    else
    {
        return false;
    }
    if(!sendCommand(OUTPUT, USBPort, stateValue))
    {
        return false;
    }
    return true;

}
