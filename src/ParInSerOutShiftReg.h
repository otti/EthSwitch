#include <stdint.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

namespace ParInSerOut
{

    class ParInSerOut
    {
        #define PIN_IS_NOT_CONFIGURED 0xFF
        private:
            uint8_t _u8CLKPin  = PIN_IS_NOT_CONFIGURED;
            uint8_t _u8DataPin = PIN_IS_NOT_CONFIGURED;
            uint8_t _u8LoadPin = PIN_IS_NOT_CONFIGURED;

        public:
            ParInSerOut(uint8_t u8CLKPin, uint8_t u8DataPin, uint8_t u8LoadPin)
            {
                this->ConfigPins(u8CLKPin, u8DataPin, u8LoadPin);
            }

            ParInSerOut(void){}

            void ConfigPins(uint8_t u8CLKPin, uint8_t u8DataPin, uint8_t u8LoadPin)
            {
                this->_u8CLKPin   = u8CLKPin;
                this->_u8DataPin  = u8DataPin;
                this->_u8LoadPin  = u8LoadPin;

                pinMode(this->_u8CLKPin , OUTPUT);
                digitalWrite(this->_u8CLKPin, LOW);

                pinMode(this->_u8DataPin, INPUT);

                pinMode(this->_u8LoadPin, OUTPUT);
                digitalWrite(this->_u8LoadPin, HIGH);

            }

            uint8_t Read(void)
            {
                uint8_t u8RetVal = 0;

                if( this->_u8CLKPin == PIN_IS_NOT_CONFIGURED )
                    return 0;

                if( this->_u8LoadPin != PIN_IS_NOT_CONFIGURED )
                {
                    // Load data with low level
                    digitalWrite(this->_u8LoadPin, LOW);
                    delayMicroseconds(5); // 74HC165 can handle up to 20 MHz
                    digitalWrite(this->_u8LoadPin, HIGH);
                }

                // Data is shifted on rising edge
                for(uint8_t i; i<8; i++)
                {
                    if( digitalRead(this->_u8DataPin) )
                        u8RetVal += (0x80>>i);

                    digitalWrite(this->_u8CLKPin, HIGH);
                    delayMicroseconds(5);
                    digitalWrite(this->_u8CLKPin, LOW);
                }

                return u8RetVal;
            }

    };
} // namespace ParInSerOut