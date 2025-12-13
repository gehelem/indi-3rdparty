#pragma once

#include "defaultdevice.h"
#include "indiweatherinterface.h"
#include "indipowerinterface.h"

#include <vector>
#include <stdint.h>
#include <poll.h>

#include "basedevice.h"

namespace Connection
{
  class Serial;
}


#define START_BYTE 0x24  // '$'
#define SEND_LENGTH 6 // 6 bytes sent
#define RECV_LENGTH 8 // 8 bytes received
#define RECV_SYNC_LENGTH 14 // 4 bytes received

static constexpr double TEMP_OFFSET = -255.50;
static constexpr double HUM_OFFSET = -254.00;

enum Targets
{
    OUTPUT = 0x01,
    VOLTAGE = 0x03,
    TEMPERATURE = 0x05,
    LENS_TEMP = 0x04,
    CURRENT = 0x07,
    HUMIDITY = 0x06,
    SYNC = 0x08,
};

enum PowerPorts
{
    DC_1 = 0x00,
    DC_2 = 0x01,
    DC_3 = 0x02,
    DC_4 = 0x03,
    DC_5 = 0x04,
    USB_C12 = 0x05,
    USB_345 = 0x06,
    ADJ = 0x07,
    DEW_A = 0x08,
    DEW_B = 0x09,
};

class SvbonySV241P : public INDI::DefaultDevice, public INDI::WeatherInterface, public INDI::PowerInterface 
{

    public:
        SvbonySV241P();

        virtual bool initProperties() override;
        virtual bool updateProperties() override;

        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;

    protected:
        const char *getDefaultName() override;

        virtual bool Connect() override;
        virtual bool Disconnect() override;

        virtual bool saveConfigItems(FILE *fp) override;

        // Event loop
        virtual void TimerHit() override;

        // Power Overrides
        virtual bool SetPowerPort(size_t port, bool enabled) override;
        virtual bool SetDewPort(size_t port, bool enabled, double dutyCycle) override;
        virtual bool SetVariablePort(size_t port, bool enabled, double voltage) override;
        virtual bool SetUSBPort(size_t port, bool enabled) override;

        // Weather Overrides
        virtual IPState updateWeather() override
        {
            return IPS_OK;
        } 

    private:
        bool openSerialPort();
        void closeSerialPort();
        uint8_t calcChecksum(const uint8_t *packet);

        bool readEnvironment();
        bool readCurrent();
        bool readVoltage();
        bool computePower();
        bool readOutput();
        bool sync();


        /**
         * @brief sendCommand Send command to unit.
         * @param cmd Command
         * @param res if nullptr, respones is ignored, otherwise read response and store it in the buffer.
         * @return
         */
        bool sendCommand(Targets target, PowerPorts port = DC_1 , uint8_t value = 0x00);
        bool readResponse(uint8_t *buffer, size_t len, Targets expectedCmd);

        int PortFD { -1 };
        bool setupComplete { false };

        Connection::Serial *serialConnection { nullptr };

        ////////////////////////////////////////////////////////////////////////////////////
        /// Main Control
        ////////////////////////////////////////////////////////////////////////////////////

        static const uint8_t ML_TIMEOUT { 3 };

        static constexpr int CMD_DELAY = 100000;
        static constexpr int READ_TIMEOUT = 3000;

};
