/**
* Create by Miguel Ángel López on 20/07/19
* and modify by xaxexa
**/

#ifndef TCL_ESP_TCL_H
#define TCL_ESP_TCL_H

#include "esphome.h"

using namespace esphome;
using namespace esphome::climate;

#define SET_TEMP_MASK	0b00001111

#define MODE_POS        7
#define MODE_MASK		0b00111111

#define MODE_AUTO      	0b00110101
#define MODE_COOL       0b00110001
#define MODE_DRY      	0b00110011
#define MODE_FAN_ONLY   0b00110010
#define MODE_HEAT       0b00110100

#define FAN_SPEED_POS	8
#define FAN_QUIET_POS	33


#define FAN_AUTO 		0b10000000	//auto
#define FAN_QUIET 		0x80		//silent
#define FAN_LOW 		0b10010000	//	|
#define FAN_MIDDLE 		0b11000000	//	||
#define FAN_MEDIUM  	0b10100000	//	|||
#define FAN_HIGH  		0b11010000	//	||||
#define FAN_FOCUS  		0b10110000	//	|||||
#define FAN_DIFFUSE		0b10000000	//	POWER [7]
#define FAN_SPEED_MASK	0b11110000	//FAN SPEED MASK


#define SWING_POS			10
#define SWING_OFF       	0b00000000
#define SWING_HORIZONTAL    0b00100000
#define SWING_VERTICAL      0b01000000
#define SWING_BOTH      	0b01100000
#define SWING_MODE_MASK    	0b01100000


#define MIN_SET_TEMPERATURE 16
#define MAX_SET_TEMPERATURE 31
#define STEP_TEMPERATURE 1

class TCL : public Climate, public PollingComponent {

private:

    byte checksum;
    byte dataTX[31];
	byte dataRX[61];
    byte poll[8] = {0xBB,0x00,0x01,0x04,0x02,0x01,0x00,0xBD};

public:

    TCL() : PollingComponent(5 * 1000) {
        checksum = 0;
    }


    
    void setup() override {
        
        Serial.begin(9600,SERIAL_8E1);
        
    }

    void loop() override  {


        if (Serial.available() > 0) {
            
			if (Serial.read() != 0xBB) return;
			dataRX[0] = 0xBB;
			delay(5);
			dataRX[1] = Serial.read();
			delay(5);
			dataRX[2] = Serial.read();
			delay(5);
			dataRX[3] = Serial.read();
			delay(5);
			dataRX[4] = Serial.read();
			
			//auto raw = getHex(dataRX, 5);
			//ESP_LOGD("TCL", "first 5 byte : %s ", raw.c_str());
			
            Serial.readBytes(dataRX+5, dataRX[4]+1);
			
			byte check = getChecksum(dataRX, sizeof(dataRX));
			
			//raw = getHex(dataRX, sizeof(dataRX));
			//ESP_LOGD("TCL", "RX full : %s ", raw.c_str());

			if (check != dataRX[60]) {
				//ESP_LOGD("TCL", "Invalid checksum %x", check);
				return;
			} else {
				//ESP_LOGD("TCL", "checksum OK %x", check);
			}

            readData();

        }

    }

    void update() override {

        Serial.write(poll, sizeof(poll));
        auto raw = getHex(poll, sizeof(poll));
        //ESP_LOGD("TCL", "chek status sended");
    }

protected:
    ClimateTraits traits() override {
        auto traits = climate::ClimateTraits();


        traits.set_supports_action(false);

        traits.set_supported_modes(
        {
            climate::CLIMATE_MODE_AUTO,
            climate::CLIMATE_MODE_COOL,
            climate::CLIMATE_MODE_DRY,
            climate::CLIMATE_MODE_FAN_ONLY,
            climate::CLIMATE_MODE_HEAT,
            climate::CLIMATE_MODE_OFF
        });

        traits.set_supported_fan_modes(
        {
            climate::CLIMATE_FAN_AUTO,			//	auto
            climate::CLIMATE_FAN_QUIET,			//	silent
            climate::CLIMATE_FAN_LOW,			//	|
            climate::CLIMATE_FAN_MIDDLE,		//	||
            climate::CLIMATE_FAN_MEDIUM,		//	|||
			climate::CLIMATE_FAN_HIGH,			//	||||
			climate::CLIMATE_FAN_FOCUS,			//	|||||
			climate::CLIMATE_FAN_DIFFUSE		//	POWER [7]
        });

        traits.set_supported_swing_modes(
        {
            climate::CLIMATE_SWING_OFF,
            climate::CLIMATE_SWING_BOTH,
            climate::CLIMATE_SWING_VERTICAL,
            climate::CLIMATE_SWING_HORIZONTAL
        });

        traits.set_visual_min_temperature(MIN_SET_TEMPERATURE);
        traits.set_visual_max_temperature(MAX_SET_TEMPERATURE);
        traits.set_visual_temperature_step(STEP_TEMPERATURE);
        traits.set_supports_current_temperature(true);
        traits.set_supports_two_point_target_temperature(false);

        return traits;
    }

public:

    void readData() {

        current_temperature = float((( (dataRX[17] << 8) | dataRX[18] ) / 374 - 32)/1.8);
        target_temperature = (dataRX[FAN_SPEED_POS] & SET_TEMP_MASK) + 16;
		
        //ESP_LOGD("TCL", "TEMP: %f ", current_temperature);

        if (dataRX[MODE_POS] & ( 1 << 4)) {			//power ON
			uint8_t modeswitch = MODE_MASK & dataRX[MODE_POS];
			uint8_t fanspeedswitch = FAN_SPEED_MASK & dataRX[FAN_SPEED_POS];
			uint8_t swingmodeswitch = SWING_MODE_MASK & dataRX[SWING_POS];

            switch (modeswitch) {
                case MODE_AUTO:
                    mode = CLIMATE_MODE_AUTO;
                    break;
                case MODE_COOL:
                    mode = CLIMATE_MODE_COOL;
                    break;
                case MODE_DRY:
                    mode = CLIMATE_MODE_DRY;
                    break;
                case MODE_FAN_ONLY:
                    mode = CLIMATE_MODE_FAN_ONLY;
                    break;
				case MODE_HEAT:
                    mode = CLIMATE_MODE_HEAT;
                    break;
                default:
                    mode = CLIMATE_MODE_AUTO;
            }
			
            if ( dataRX[FAN_QUIET_POS] & FAN_QUIET) {
                fan_mode = CLIMATE_FAN_QUIET;
            } else if (dataRX[MODE_POS] & FAN_DIFFUSE){
				fan_mode = CLIMATE_FAN_DIFFUSE;
			} else {
                switch (fanspeedswitch) {
                    case FAN_AUTO:
                        fan_mode = CLIMATE_FAN_AUTO;
                        break;
                    case FAN_LOW:
                        fan_mode = CLIMATE_FAN_LOW;
                        break;
                    case FAN_MIDDLE:
                        fan_mode = CLIMATE_FAN_MIDDLE;
                        break;
                    case FAN_MEDIUM:
                        fan_mode = CLIMATE_FAN_MEDIUM;
                        break;
					case FAN_HIGH:
						fan_mode = CLIMATE_FAN_HIGH;
						break;
					case FAN_FOCUS:
						fan_mode = CLIMATE_FAN_FOCUS;
						break;
                    default:
                        fan_mode = CLIMATE_FAN_AUTO;
                }
            }


            switch (swingmodeswitch) {
                case SWING_OFF: 
                    swing_mode = CLIMATE_SWING_OFF;
					break;
				case SWING_HORIZONTAL:
					swing_mode = CLIMATE_SWING_HORIZONTAL;
					break;
				case SWING_VERTICAL:
					swing_mode = CLIMATE_SWING_VERTICAL;
					break;
                case SWING_BOTH:
                    swing_mode = CLIMATE_SWING_BOTH;
                    break;

            } 
        } else {
            mode = CLIMATE_MODE_OFF;
            fan_mode = CLIMATE_FAN_OFF;
            swing_mode = CLIMATE_SWING_OFF;
        }

        this->publish_state();

    }

// Climate control
    void control(const ClimateCall &call) override {
		
		uint8_t switchvar = 0;
		
		dataTX[7]  = 0b00000000;//eco,display,beep,ontimerenable, offtimerenable,power,0,0
		dataTX[8]  = 0b00000000;//mute,0,turbo,health,mode(4)  0=cool 1=fan  2=dry 3=heat 4=auto 
		dataTX[9]  = 0b00000000;//[9] = 0,0,0,0,temp(4) 31 - value
		dataTX[10] = 0b00000000;//[10] = 0,timerindicator,swingv(3),fan(3) 0=auto 1=low 2=med 3=high
		//																{0,2,3,5,0};

        if (call.get_mode().has_value()){
			switchvar = call.get_mode().value();
		} else {
			switchvar = mode;
		}
            switch (switchvar) {
                case CLIMATE_MODE_OFF:
                    dataTX[7] += 0b00100000;
					dataTX[8] += 0b00000000;
					
                    break;
                case CLIMATE_MODE_AUTO:
                    dataTX[7] += 0b01100100;
                    dataTX[8] += 0b00001000;
					
					break;
                case CLIMATE_MODE_COOL:
                    dataTX[7] += 0b01100100;
                    dataTX[8] += 0b00000011;
					
					break;
                case CLIMATE_MODE_DRY:
                    dataTX[7] += 0b01100100;
                    dataTX[8] += 0b00000010;
					
					break;
                case CLIMATE_MODE_FAN_ONLY:
                    dataTX[7] += 0b01100100;
                    dataTX[8] += 0b00000111;
					
					break;
                case CLIMATE_MODE_HEAT:
                    dataTX[7] += 0b01100100;
                    dataTX[8] += 0b00000001;
					
					break;
            }
		

        //Set fan speed
		if (call.get_fan_mode().has_value()){
			switchvar = call.get_fan_mode().value();
			switch(switchvar) {
                case CLIMATE_FAN_AUTO:
					dataTX[8]	+= 0b00000000;
					dataTX[10]	+= 0b00000000;
                    break;
                case CLIMATE_FAN_QUIET:
					dataTX[8]	+= 0b10000000;
					dataTX[10]	+= 0b00000000;
                    break;
                case CLIMATE_FAN_LOW:
					dataTX[8]	+= 0b00000000;
					dataTX[10]	+= 0b00000001;
                    break;
                case CLIMATE_FAN_MIDDLE:
					dataTX[8]	+= 0b00000000;
					dataTX[10]	+= 0b00000110;
                    break;
                case CLIMATE_FAN_MEDIUM:
					dataTX[8]	+= 0b00000000;
					dataTX[10]	+= 0b00000011;
                    break;
                case CLIMATE_FAN_HIGH:
					dataTX[8]	+= 0b00000000;
					dataTX[10]	+= 0b00000111;
                    break;
                case CLIMATE_FAN_FOCUS:
					dataTX[8]	+= 0b00000000;
					dataTX[10]	+= 0b00000101;
                    break;
                case CLIMATE_FAN_DIFFUSE:
					dataTX[8]	+= 0b01000000;
					dataTX[10]	+= 0b00000000;
                    break;
			}
			
		} else {
			if(fan_mode == CLIMATE_FAN_AUTO){
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000000;
            } else if(fan_mode == CLIMATE_FAN_QUIET){
				dataTX[8]	+= 0b10000000;
				dataTX[10]	+= 0b00000000;
            } else if(fan_mode == CLIMATE_FAN_LOW){
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000001;
            } else if(fan_mode == CLIMATE_FAN_MIDDLE){
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000110;
            } else if(fan_mode == CLIMATE_FAN_MEDIUM){
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000011;
            } else if(fan_mode == CLIMATE_FAN_HIGH){
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000111;
            } else if(fan_mode == CLIMATE_FAN_FOCUS){
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000101;
            } else if(fan_mode == CLIMATE_FAN_DIFFUSE){
				dataTX[8]	+= 0b01000000;
				dataTX[10]	+= 0b00000000;
			}
		}

        //Set swing mode
        if (call.get_swing_mode().has_value()){
			switchvar = call.get_swing_mode().value();
		} else {
			switchvar = swing_mode;
		}	
            switch(switchvar) {
                case CLIMATE_SWING_OFF:
                    dataTX[10]	+= 0b00000000;
					dataTX[14]	+= 0b00000000;
                    break;
                case CLIMATE_SWING_VERTICAL:
                    dataTX[10]	+= 0b00111000;
					dataTX[14]	+= 0b00000000;
                    break;
                case CLIMATE_SWING_HORIZONTAL:
                    dataTX[10]	+= 0b00000000; // ??????
					dataTX[14]	+= 0b00000000;
                    break;
                case CLIMATE_SWING_BOTH:
                    dataTX[10]	+= 0b00000000; // ??????
					dataTX[14]	+= 0b00000000;  
                    break;
			}

        if (call.get_target_temperature().has_value()) {
            dataTX[9] = 31-(int)call.get_target_temperature().value();		//0,0,0,0, temp(4)
        } else {
			dataTX[9] = 31-(int)target_temperature;
		}

        //Values for "send"
        dataTX[0] = 0xBB;	//start byte
		dataTX[1] = 0x00;	//start byte
		dataTX[2] = 0x01;	//start byte
		dataTX[3] = 0x03;	//control type, 0x04 heartbit
		dataTX[4] = 0x19;	//lenght
		dataTX[5] = 0x01;	//??
		dataTX[6] = 0x00;	//??
		//dataTX[7] = 0x64;		//eco,display,beep,ontimerenable, offtimerenable,power,0,0
		//dataTX[8] = 0x08;		//mute,0,turbo,health, mode(4) mode 01 heat, 02 dry, 03 cool, 07 fan, 08 auto, health(+16), 41=turbo-heat 43=turbo-cool (turbo = 0x40+ 0x01..0x08)
		//dataTX[9] = 0x0f;		//0 -31 ;    15 - 16 0,0,0,0, temp(4) settemp 31 - x
		//dataTX[10] = 0x00;	//0,timerindicator,swingv(3),fan(3) fan+swing modes //0=auto 1=low 2=med 3=high
		dataTX[11] = 0x00;		//0,offtimer(6),0
		dataTX[12] = 0x00;		//fahrenheit,ontimer(6),0 cf 80=f 0=c
		dataTX[13] = 0x00;	//??
        dataTX[14] = 0x00;	//0,0,halfdegree,0,swingh,0,0,0
        dataTX[15] = 0x00;	//??
        dataTX[16] = 0x00;	//??
		dataTX[17] = 0x00;	//??
		dataTX[18] = 0x00;	//??
		dataTX[19] = 0x00;	//sleep on = 1 off=0
		dataTX[20] = 0x00;	//??
		dataTX[21] = 0x00;	//??
		dataTX[22] = 0x00;	//??
		dataTX[23] = 0x00;	//??
		dataTX[24] = 0x00;	//??
		dataTX[25] = 0x00;	//??
		dataTX[26] = 0x00;	//??
		dataTX[27] = 0x00;	//??
		dataTX[28] = 0x00;	//??
		dataTX[29] = 0x00;	//??
		dataTX[30] = 0xFF;	//Checksum
		dataTX[30] = getChecksum(dataTX, sizeof(dataTX));

        sendData(dataTX, sizeof(dataTX));

    }


    void sendData(byte * message, byte size) {

        Serial.write(message, size);

        //auto raw = getHex(message, size);
        //ESP_LOGD("TCL", "Sended message: %s ", raw.c_str());

    }

    String getHex(byte *message, byte size) {

        String raw;

        for (int i = 0; i < size; i++) {
            raw += "\n" + String(message[i]);

        }
        raw.toUpperCase();

        return raw;
	}

    byte getChecksum(const byte * message, size_t size) {
        byte position = size - 1;
        byte crc = 0;

        for (int i = 0; i < position; i++)
            crc ^= message[i];
        return crc;

    }




};


#endif //TCL_ESP_TCL_H
