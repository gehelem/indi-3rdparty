#include "indi_svbony_sv241pro.h"
#include "indicom.h"
#include "connectionplugins/connectionserial.h"

#include <cerrno>
#include <cstring>
#include <memory>
#include <termios.h>
#include <fcntl.h>
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
        if (sync())
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

bool  SvbonySV241P::sendCommand(Targets target, PowerPorts port, uint8_t value)
{
    int nbytes_written = 0, rc = -1;
    char packet[SEND_LENGTH];
    packet[0] = START_BYTE;
    packet[1] = SEND_LENGTH;
    packet[2] = target;
    packet[3] = port;
    packet[4] = value;

    // Calculate checksum
    char checksum = 0;
    for (int i = 0; i < SEND_LENGTH - 1; i++)
    {
        checksum += packet[i];
    }
    packet[SEND_LENGTH - 1] = checksum % 256;
    char hexLog[32];
    sprintf(hexLog, "%02X %02X %02X %02X %02X %02X", 
            packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);
    LOGF_DEBUG("TX SVBONY -> <%s>", hexLog);

    tcflush(PortFD, TCIOFLUSH);

    if ((rc = tty_write(PortFD,packet, SEND_LENGTH, &nbytes_written)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", errstr);
        return false;
    }

    usleep(CMD_DELAY);
    
    return true;
}

bool SvbonySV241P::readResponse(uint8_t *response, size_t len, Targets expectedCmd) {
    if (PortFD < 0) return false;

    int nbytes_read = 0, rc = -1;
    
    size_t totalRead = 0;

    rc = tty_read(PortFD, (char*)(response), len, READ_TIMEOUT, &nbytes_read);

    if (rc != TTY_OK && nbytes_read != (int)len) {
        LOGF_ERROR("Incomplete packet: expected %zu bytes, got %d", len, nbytes_read + 1);
        tcflush(PortFD, TCIOFLUSH);
        return false;
    }
    
    totalRead += nbytes_read;

    if (totalRead > 0)
    {
        // Debug Log
        LOGF_DEBUG("RX (%zu bytes): %02X %02X %02X %02X ...", totalRead, response[0], response[1], response[2], response[3]);
    }

    if (totalRead != len)
    {
        LOGF_ERROR("Size mismatch: Expected %zu, Got %zu", len, totalRead);
        return false;
    }

    if (response[2] != expectedCmd)
    {
        LOGF_ERROR("Unexpected response command: expected %02X, got %02X", expectedCmd, response[2]);
        tcflush(PortFD, TCIOFLUSH);
        return false;
    }

    return true;
}

bool SvbonySV241P::sync(){
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

bool SvbonySV241P::readVoltage(){
    sendCommand(VOLTAGE);
    uint8_t packet[RECV_LENGTH];
    if(!readResponse(packet, RECV_LENGTH, VOLTAGE)){
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

bool SvbonySV241P::readCurrent(){
    sendCommand(CURRENT);
    uint8_t packet[RECV_LENGTH];
    if(!readResponse(packet, RECV_LENGTH, CURRENT)){
        return false;
    }
    uint8_t data[4] = {packet[6], packet[5], packet[4], packet[3]};
    int32_t rawValue;
    memcpy(&rawValue, data, 4);
    double current = (rawValue / 100.0);
    PowerSensorsNP[SENSOR_CURRENT].setValue(current);
    PowerSensorsNP.apply();
    return true;
}

bool SvbonySV241P::computePower(){
    double voltage = PowerSensorsNP[SENSOR_VOLTAGE].getValue();
    double current = PowerSensorsNP[SENSOR_CURRENT].getValue();
    double power = (15.57 * (voltage * current) + 269.39) / 1000.0;
    PowerSensorsNP[SENSOR_POWER].setValue(power);
    PowerSensorsNP.apply();
    return true;
}

bool SvbonySV241P::readEnvironment(){
    //Temperature
    sendCommand(TEMPERATURE);
    uint8_t packet[RECV_LENGTH];
    if(!readResponse(packet, RECV_LENGTH, TEMPERATURE)){
        return false;
    }
    uint8_t data[4] = {packet[6], packet[5], packet[4], packet[3]};
    uint32_t rawValue;
    memcpy(&rawValue, data, 4);
    double temperature = (rawValue / 100.0) + TEMP_OFFSET;
    setParameterValue("WEATHER_TEMPERATURE", temperature);
    //Humidity
    sendCommand(HUMIDITY);
    if(!readResponse(packet, RECV_LENGTH, HUMIDITY)){
        return false;
    }
    data[0] = packet[6];
    data[1] = packet[5];
    data[2] = packet[4];
    data[3] = packet[3];
    memcpy(&rawValue, data, 4);
    double humidity = (rawValue / 100.0);
    setParameterValue("WEATHER_HUMIDITY", humidity);
    //Dewpoint
    double dewpoint = (243.04 * (log(humidity / 100.0) + (17.625 * temperature / (243.04 + temperature))) / (17.625 - log(humidity / 100.0) - (17.625 * temperature / (243.04 + temperature))));
    setParameterValue("WEATHER_DEWPOINT", dewpoint);
    // Lens Temperature
    sendCommand(LENS_TEMP);
    if(!readResponse(packet, RECV_LENGTH, LENS_TEMP)){
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

void SvbonySV241P::TimerHit(){
    if (!isConnected() || setupComplete == false)
    {
        SetTimer(getCurrentPollingPeriod());
        return;
    }

    sync();

    readVoltage();
    readCurrent();
    computePower();

    readEnvironment();

    SetTimer(getCurrentPollingPeriod());
}





bool SvbonySV241P::SetPowerPort(size_t port, bool enabled)
{
    if (port == 0)
    {
        return sendCommand(OUTPUT, DC_1, enabled ? 0xff : 0x00);
    }
    else if (port == 1)
    {
        return sendCommand(OUTPUT, DC_2, enabled ? 0xff : 0x00);
    }
    else if (port == 2)
    {
        return sendCommand(OUTPUT, DC_3, enabled ? 0xff : 0x00);
    }
    else if (port == 3)
    {
        return sendCommand(OUTPUT, DC_4, enabled ? 0xff : 0x00);
    }
    else if (port == 4)
    {
        return sendCommand(OUTPUT, DC_5, enabled ? 0xff : 0x00);
    }
    return false;
}

bool SvbonySV241P::SetDewPort(size_t port, bool enabled, double dutyCycle)
{
    // Map dutyCycle (0-100%) to value (0-255) in hex
    dutyCycle = dutyCycle * 255 / 100;
    if (port == 0)
    {
            return sendCommand(OUTPUT, DEW_A, enabled ? dutyCycle : 0x00);
    }
    else
    {
        return sendCommand(OUTPUT, DEW_B, enabled ? dutyCycle : 0x00);
    }
}

bool SvbonySV241P::SetVariablePort(size_t port, bool enabled, double voltage)
{
    INDI_UNUSED(port);


    if(!enabled){
        return sendCommand(OUTPUT, ADJ, 0x00);
    }
    // Map voltage (0-12V) to value (0-255)
    uint8_t hex_voltage = static_cast<uint8_t>((voltage / 15.3) * 253);
    return sendCommand(OUTPUT, ADJ, hex_voltage);
}

bool SvbonySV241P::CyclePower()
{
    sendCommand(OUTPUT, DC_1, 0x00);
    sendCommand(OUTPUT, DC_2, 0x00);
    sendCommand(OUTPUT, DC_3, 0x00);
    sendCommand(OUTPUT, DC_4, 0x00);
    sendCommand(OUTPUT, DC_5, 0x00);
    sendCommand(OUTPUT, USB_C12, 0x00);
    sendCommand(OUTPUT, USB_345, 0x00);
    sleep(2); // wait for 2 seconds
    Handshake();
    sendCommand(OUTPUT, DC_1, 0xff);
    sendCommand(OUTPUT, DC_2, 0xff);
    sendCommand(OUTPUT, DC_3, 0xff);
    sendCommand(OUTPUT, DC_4, 0xff);
    sendCommand(OUTPUT, DC_5, 0xff);
    sendCommand(OUTPUT, USB_C12, 0xff);
    sendCommand(OUTPUT, USB_345, 0xff);
    return true;
}

bool SvbonySV241P::SetUSBPort(size_t port, bool enabled)
{
    if (port == 0)
    {
        return sendCommand(OUTPUT, USB_C12, enabled ? 0xff : 0x00);
    }
    else
    {
        return sendCommand(OUTPUT, USB_345, enabled ? 0xff : 0x00);
    }
}