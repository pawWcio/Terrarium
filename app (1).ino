const int TX_FRAME_SIZE = 10;
const int RX_FRAME_SIZE = 10;
const int START_CODE = 0x40;
const int END_CODE = 0x80;

struct dataTX
{
	uint8_t start_code;
	uint16_t value_temperature;
	uint16_t value_humidity;
	uint8_t value_fan;
	uint8_t value_time;
	uint8_t value_bulb;
	uint8_t value_fogger;
	uint8_t end_code;
} __attribute__((__packed__));

union frameTX_u {
	uint8_t *bytes;
	struct dataTX *data;
} frameTX;

struct dataRX
{
	uint8_t start_code;
    uint16_t value_temperature;
	uint16_t value_humidity;
	uint8_t value_fan;
	uint8_t value_time;
	uint8_t value_bulb;
	uint8_t value_fogger;
	uint8_t end_code;
} __attribute__((__packed__));

union frameRX_u {
	uint8_t *bytes;
	struct dataTX *data;
} frameRX;

void setup()
{
	Serial.begin(9600);
	frameTX.bytes = new uint8_t[TX_FRAME_SIZE];
	frameRX.bytes = new uint8_t[RX_FRAME_SIZE];
	frameTX.data->value_temperature = sensor.readTemperature();
	frameTX.data->value_humidity = sensor.readHumidity();
	frameTX.data->value_fan = OCR2B;
	frameTX.data->value_time = tm.Hour;
	frameTX.data->value_bulb = bulb_status;
	frameTX.data->value_fogger = fogger_status;
	frameTX.data->start_code = START_CODE;
	frameTX.data->end_code = END_CODE;
}

void loop()
{
	Serial.write(frameTX.bytes, TX_FRAME_SIZE);

	if (Serial.available() >= RX_FRAME_SIZE)
	{
		for (byte i = 0; i < RX_FRAME_SIZE; i++)
		{
			frameRX.bytes[i] = Serial.read();
			frameTX.data->value_temperature = frameRX.data->value_temperature;
			frameTX.data->value_humidity = frameRX.data->value_humidity;
			frameTX.data->value_fan = frameRX.data->value_fan;
			frameTX.data->value_time = frameRX.data->value_time;
			frameTX.data->value_bulb = frameRX.data->value_bulb;
			frameTX.data->value_fogger = frameRX.data->value_fogger;
		}
	}
}
