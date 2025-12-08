#include "indi_svbony_sv241pro.h"
#include "indicom.h"
#include "connectionplugins/connectionserial.h"

#include <cerrno>
#include <cstring>
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
    
    setDriverInterface(AUX_INTERFACE | WEATHER_INTERFACE | POWER_INTERFACE);

    WI::initProperties(MAIN_CONTROL_TAB, MAIN_CONTROL_TAB);

    addAuxControls();

    PI::SetCapability(POWER_HAS_DC_OUT | POWER_HAS_DEW_OUT | POWER_HAS_VARIABLE_OUT |
                      POWER_HAS_VOLTAGE_SENSOR | POWER_HAS_OVERALL_CURRENT | POWER_HAS_USB_TOGGLE
                      | POWER_HAS_POWER_CYCLE);

    // Power Interface properties
    // 5 DC output, 2 DEW outputs, 1 Variable output, 1 Auto Dew ports (Global), 5 USB ports on 2 switches
    PI::initProperties(POWER_TAB, 5, 2, 1, 0, 2);

    // Environment Group
    addParameter("WEATHER_TEMPERATURE", "Temperature (°C)", -15, 35, 15);
    addParameter("WEATHER_HUMIDITY", "Humidity (%)", 0, 100, 15);
    addParameter("WEATHER_DEWPOINT", "Dew Point (°C)", 0, 100, 15);
    addParameter("WEATHER_LENS_TEMPERATURE", "Lens Temperature (°C)", -15, 35, 15);
    setCriticalParameter("WEATHER_TEMPERATURE");

    // Serial Connection
    serialConnection = new Connection::Serial(this);
    serialConnection->registerHandshake([&]()
    {
        return Handshake();
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

bool SvbonySV241P::Handshake()
{
    PortFD = serialConnection->getPortFD();
    if (Ack())
    {
        LOG_INFO("Handshake with Svbony SV241 Pro successful.");
        return true;
    }
    else
    {
        LOG_ERROR("Handshake with Svbony SV241 Pro failed.");
        return false;
    }
}

bool SvbonySV241P::Ack()
{
    bool success = false;
    for (int i = 0; i < 3; i++)
    {
        if (getConsumptionData())
        {
            success = true;
            break;
        }
        sleep(1);
    }
    return success;

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

bool SvbonySV241P::sendCommand(Targets target, PowerPorts port , uint8_t value, char *response)
{
    unsigned char packet[SEND_LENGTH];
    int nbytes_written = 0, nbytes_read = 0, rc = -1;
    packet[0] = START_BYTE;
    packet[1] = SEND_LENGTH;
    packet[2] = target;
    packet[3] = port;
    packet[4] = value;

    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 0; i < SEND_LENGTH - 1; i++)
    {
        checksum += packet[i];
    }
    packet[SEND_LENGTH - 1] = (unsigned char)(checksum % 256);
    char hexLog[32];
    sprintf(hexLog, "%02X %02X %02X %02X %02X %02X", 
            packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);
    LOGF_DEBUG("TX SVBONY -> <%s>", hexLog);

    tcflush(PortFD, TCIOFLUSH);

    if ((rc = tty_write(PortFD, (const char *)packet, SEND_LENGTH, &nbytes_written)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", errstr);
        return false;
    }

    if (response == nullptr)
    {
        return true;
    }

    // Read response
    rc = tty_read(PortFD, response, RECV_LENGTH, ML_TIMEOUT, &nbytes_read);
    if (rc != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", errstr);
        return false;
    }

    char hexRes[32];
    sprintf(hexRes, "%02X %02X %02X %02X %02X %02X %02X %02X", 
            response[0], response[1], response[2], response[3], response[4], response[5], response[6], response[7]);
    LOGF_DEBUG("RX SVBONY <- <%s>", hexRes);   
    
    tcflush(PortFD, TCIOFLUSH);
    
    return true;
}



bool SvbonySV241P::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);

    WI::saveConfigItems(fp);
    PI::saveConfigItems(fp);
    return true;
}

void SvbonySV241P::TimerHit(){
    if (!isConnected() || setupComplete == false)
    {
        SetTimer(getCurrentPollingPeriod());
        return;
    }
    getConsumptionData();
    getMetricsData();

    SetTimer(getCurrentPollingPeriod());
}

bool SvbonySV241P::getConsumptionData()
{
    char res[RECV_LENGTH] = {0};
    if (sendCommand(VOLTAGE, ZERO, 0x00, res))
    {
        uint16_t raw_voltage = (res[5] << 8) | res[6];

        double voltage = raw_voltage / 100.0; // Assuming the device sends voltage in centivolts
        PowerSensorsNP[SENSOR_VOLTAGE].setValue(voltage);
        PowerSensorsNP.apply(); 
    }
    if (sendCommand(CURRENT, ZERO, 0x00, res))
    {
        uint16_t raw_current = (res[5] << 8) | res[6];

        double current = raw_current * 0.0002; // Assuming the device sends current in milliamps
        PowerSensorsNP[SENSOR_CURRENT].setValue(current);
        PowerSensorsNP.apply();

        PowerSensorsNP[SENSOR_POWER].setValue(PowerSensorsNP[SENSOR_VOLTAGE].getValue() * PowerSensorsNP[SENSOR_CURRENT].getValue());
        PowerSensorsNP.apply();
    }

    return true;
}

bool SvbonySV241P::getMetricsData()
{
    char res[RECV_LENGTH] = {0};
    if(sendCommand(TEMPERATURE, ZERO, 0x00, res))
    {
        // convert hex value to double (assuming response is in res[5] and res[6])
        uint16_t raw_temp = (res[5] << 8) | res[6];
        double temperature = static_cast<double>(raw_temp) / 100.0; // Assuming the device sends temperature in centi-degrees Celsius
        setParameterValue("WEATHER_TEMPERATURE", temperature);
    }
    if(sendCommand(HUMIDITY, ZERO, 0x00, res))
    {
        // convert hex value to double (assuming response is in res[5] and res[6])
        uint16_t raw_humidity = (res[5] << 8) | res[6];
        double humidity = static_cast<double>(raw_humidity) / 100.0; // Assuming the device sends humidity in centi-percent
        setParameterValue("WEATHER_HUMIDITY", humidity);
    }
    if(sendCommand(DEWPOINT, ZERO, 0x00, res))
    {
        // convert hex value to double (assuming response is in res[5] and res[6])
        uint16_t raw_dewpoint = (res[5] << 8) | res[6];
        double dewpoint = static_cast<double>(raw_dewpoint) / 100.0; // Assuming the device sends dew point in centi-degrees Celsius
        setParameterValue("WEATHER_DEWPOINT", dewpoint);
    }
    if(sendCommand(LENS_TEMP, ZERO, 0x00, res))
    {
        // convert hex value to double (assuming response is in res[5] and res[6])
        uint16_t raw_lens_temp = (res[5] << 8) | res[6];
        double lens_temp = static_cast<double>(raw_lens_temp) / 100.0; // Assuming the device sends lens temperature in centi-degrees Celsius
        setParameterValue("WEATHER_LENS_TEMPERATURE", lens_temp);
    }
    return true;
}

bool SvbonySV241P::SetPowerPort(size_t port, bool enabled)
{
    if (port == 0)
    {
        return sendCommand(OUTPUT, DC_1, enabled ? 0xff : 0x00, nullptr);
    }
    else if (port == 1)
    {
        return sendCommand(OUTPUT, DC_2, enabled ? 0xff : 0x00, nullptr);
    }
    else if (port == 2)
    {
        return sendCommand(OUTPUT, DC_3, enabled ? 0xff : 0x00, nullptr);
    }
    else if (port == 3)
    {
        return sendCommand(OUTPUT, DC_4, enabled ? 0xff : 0x00, nullptr);
    }
    else if (port == 4)
    {
        return sendCommand(OUTPUT, DC_5, enabled ? 0xff : 0x00, nullptr);
    }
    return false;
}

bool SvbonySV241P::SetDewPort(size_t port, bool enabled, double dutyCycle)
{
    // Map dutyCycle (0-100%) to value (0-255) in hex
    dutyCycle = (dutyCycle < 0.0) ? 0.0 : (dutyCycle > 100.0) ? 100.0 : dutyCycle;
    if (port == 0)
    {
            return sendCommand(OUTPUT, DEW_A, enabled ? dutyCycle : 0x00, nullptr);
    }
    else
    {
        return sendCommand(OUTPUT, DEW_B, enabled ? dutyCycle : 0x00, nullptr);
    }
}

bool SvbonySV241P::SetVariablePort(size_t port, bool enabled, double voltage)
{
    INDI_UNUSED(port);


    if(!enabled){
        return sendCommand(OUTPUT, ADJ, 0x00, nullptr);
    }
    // Map voltage (0-12V) to value (0-255)
    uint8_t hex_voltage = static_cast<uint8_t>((voltage / 15.0) * 255.0);
    return sendCommand(OUTPUT, ADJ, hex_voltage, nullptr);
}

bool SvbonySV241P::CyclePower()
{
    sendCommand(OUTPUT, DC_1, 0x00, nullptr);
    sendCommand(OUTPUT, DC_2, 0x00, nullptr);
    sendCommand(OUTPUT, DC_3, 0x00, nullptr);
    sendCommand(OUTPUT, DC_4, 0x00, nullptr);
    sendCommand(OUTPUT, DC_5, 0x00, nullptr);
    sendCommand(OUTPUT, USB_C12, 0x00, nullptr);
    sendCommand(OUTPUT, USB_345, 0x00, nullptr);
    sleep(2); // wait for 2 seconds
    Handshake();
    sendCommand(OUTPUT, DC_1, 0xff, nullptr);
    sendCommand(OUTPUT, DC_2, 0xff, nullptr);
    sendCommand(OUTPUT, DC_3, 0xff, nullptr);
    sendCommand(OUTPUT, DC_4, 0xff, nullptr);
    sendCommand(OUTPUT, DC_5, 0xff, nullptr);
    sendCommand(OUTPUT, USB_C12, 0xff, nullptr);
    sendCommand(OUTPUT, USB_345, 0xff, nullptr);
    return true;
}

bool SvbonySV241P::SetUSBPort(size_t port, bool enabled)
{
    if (port == 0)
    {
        return sendCommand(OUTPUT, USB_C12, enabled ? 0xff : 0x00, nullptr);
    }
    else
    {
        return sendCommand(OUTPUT, USB_345, enabled ? 0xff : 0x00, nullptr);
    }
}