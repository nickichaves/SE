#include <SimpleDHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// ========== DHT11 ==========
#define DHTPIN 4
SimpleDHT11 dht11(DHTPIN);

// ========== BMP085 ==========
Adafruit_BMP085 bmp;

// ========== USART ==========
#define BAUD_RATE 9600
#define BAUD_DIV ((F_CPU / 16 / BAUD_RATE) - 1)

volatile char usart_rx_buffer[64];
volatile uint8_t usart_rx_head = 0;
volatile uint8_t usart_rx_tail = 0;

void USART_init() {
  UBRR0 = BAUD_DIV;
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_transmit(char data) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

void USART_print(const char* s) {
  while (*s) USART_transmit(*s++);
}

void USART_println(const char* s) {
  USART_print(s);
  USART_transmit('\r');
  USART_transmit('\n');
}

void USART_printInt(long num) {
  char buf[16];
  ltoa(num, buf, 10);
  USART_print(buf);
}

bool USART_available() {
  return usart_rx_head != usart_rx_tail;
}

char USART_read() {
  if (usart_rx_head == usart_rx_tail) return 0;
  char c = usart_rx_buffer[usart_rx_tail];
  usart_rx_tail = (usart_rx_tail + 1) % sizeof(usart_rx_buffer);
  return c;
}

ISR(USART_RX_vect) {
  char c = UDR0;
  uint8_t next_head = (usart_rx_head + 1) % sizeof(usart_rx_buffer);
  if (next_head != usart_rx_tail) {
    usart_rx_buffer[usart_rx_head] = c;
    usart_rx_head = next_head;
  }
}

// ========== Variáveis Globais ==========
volatile int leituraADC[3] = {0, 0, 0};
volatile uint8_t canalAtual = 0;
volatile bool leiturasProntas = false;
unsigned long ultimoTempoAtivacao = 0;
const unsigned long intervaloAtivacao = 60000;
const byte frameStartByte = 0x7E;
const byte frameTypeTXrequest = 0x10;
const byte frameTypeRXpacket = 0x90;
const byte frameTypeATresponse = 0x88;
const long destAddressHigh = 0x13A200;
const long destAddressLow = 0x414F7F07;
char DBcommand[] = "DB";

byte ATcounter = 0;
byte rssi = 0;

const int LIMIAR_CHUVA = 400;

unsigned long servoAtivadoEm = 0;
bool servoAtivo = false;

// ===== ADC Setup =====
void adcSetup(void) {
  ADMUX = (1 << REFS0); 
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect) {
  leituraADC[canalAtual] = ADC;
  canalAtual++;
  if (canalAtual > 2) {
    canalAtual = 0;
    leiturasProntas = true;
  }
  ADMUX = (ADMUX & 0xF0) | canalAtual;
  ADCSRA |= (1 << ADSC);
}

// ===== Servo Setup =====
void servoSetup() {
  DDRB |= (1 << DDB1);
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 19999;
  OCR1A = 1100;  // posição inicial (fechado)
}

// ===== Funções API XBee e comunicação =====
char decodeAPIpacket(void) {
  char rxbyte = 0;
  byte frametype;

  // Espera start byte
  while (true) {
    if (!USART_available()) return rxbyte;
    char c = USART_read();
    if (c == frameStartByte) break;
  }

  if (!USART_available()) return rxbyte;
  char lenMSB = USART_read();
  if (!USART_available()) return rxbyte;
  char lenLSB = USART_read();

  int frameLength = (lenMSB << 8) | (uint8_t)lenLSB;

  if (!USART_available()) return rxbyte;
  frametype = USART_read();

  if (frametype == frameTypeRXpacket) {
    while (((usart_rx_head + 64 - usart_rx_tail) % 64) < frameLength) {
    }

    for (int i = 0; i < 11; i++) USART_read();
    rxbyte = USART_read();
    USART_read(); // checksum
    formatATcommandAPI("DB");
  } else if (frametype == frameTypeATresponse) {
    while (((usart_rx_head + 64 - usart_rx_tail) % 64) < 6) {}
    USART_read(); // Frame ID
    USART_read(); // AT cmd 1
    USART_read(); // AT cmd 2
    USART_read(); // Status
    rssi = USART_read();
    USART_read(); // checksum
  }
  return rxbyte;
}

void formatATcommandAPI(char* command) {
  long sum = 0;
  ATcounter++;
  USART_transmit(frameStartByte);
  USART_transmit(0x00);
  USART_transmit(0x04);
  USART_transmit(0x08);
  sum += 0x08;
  USART_transmit(ATcounter);
  sum += ATcounter;
  USART_transmit(command[0]);
  USART_transmit(command[1]);
  sum += command[0];
  sum += command[1];
  USART_transmit(0xFF - (sum & 0xFF));
  _delay_ms(10);
}

void sendXBeePayload(byte* payload, byte payloadLen) {
  long sum = 0;
  USART_transmit(frameStartByte);
  uint16_t length = 14 + payloadLen;
  USART_transmit((length >> 8) & 0xFF);
  USART_transmit(length & 0xFF);
  USART_transmit(frameTypeTXrequest); sum += frameTypeTXrequest;
  USART_transmit(0x00); sum += 0x00;

  USART_transmit((destAddressHigh >> 24) & 0xFF); sum += (destAddressHigh >> 24) & 0xFF;
  USART_transmit((destAddressHigh >> 16) & 0xFF); sum += (destAddressHigh >> 16) & 0xFF;
  USART_transmit((destAddressHigh >> 8) & 0xFF);  sum += (destAddressHigh >> 8) & 0xFF;
  USART_transmit(destAddressHigh & 0xFF);         sum += destAddressHigh & 0xFF;

  USART_transmit((destAddressLow >> 24) & 0xFF); sum += (destAddressLow >> 24) & 0xFF;
  USART_transmit((destAddressLow >> 16) & 0xFF); sum += (destAddressLow >> 16) & 0xFF;
  USART_transmit((destAddressLow >> 8) & 0xFF);  sum += (destAddressLow >> 8) & 0xFF;
  USART_transmit(destAddressLow & 0xFF);         sum += destAddressLow & 0xFF;

  USART_transmit(0xFF); USART_transmit(0xFE); sum += 0xFF + 0xFE;
  USART_transmit(0x00); sum += 0x00;
  USART_transmit(0x20); sum += 0x20;  // 0x20 - Enable APS encryption (if EE=1)

  for (byte i = 0; i < payloadLen; i++) {
    USART_transmit(payload[i]);
    sum += payload[i];
  }
  USART_transmit(0xFF - (sum & 0xFF));
  _delay_ms(10);
}


void sendSensorData(int vel, int dir, int agua, float temp, float hum, int32_t pressao, byte rssiVal, byte flagChuva) {
  byte payload[16];

  payload[0] = (vel >> 8) & 0xFF;
  payload[1] = vel & 0xFF;
  payload[2] = (dir >> 8) & 0xFF;
  payload[3] = dir & 0xFF;
  payload[4] = (agua >> 8) & 0xFF;
  payload[5] = agua & 0xFF;

  int16_t tempInt = (int16_t)(temp * 10);
  payload[6] = (tempInt >> 8) & 0xFF;
  payload[7] = tempInt & 0xFF;

  int16_t humInt = (int16_t)(hum * 10);
  payload[8] = (humInt >> 8) & 0xFF;
  payload[9] = humInt & 0xFF;

  payload[10] = (pressao >> 24) & 0xFF;
  payload[11] = (pressao >> 16) & 0xFF;
  payload[12] = (pressao >> 8) & 0xFF;
  payload[13] = pressao & 0xFF;

  payload[14] = rssiVal;
  payload[15] = flagChuva;

  sendXBeePayload(payload, sizeof(payload));
}

void setup() {
  USART_init();
  adcSetup();
  servoSetup();
  bmp.begin();
  sei();
}

void loop() {
  if (leiturasProntas) {
    leiturasProntas = false;

    int leituraVel = leituraADC[0];
    int leituraDir = leituraADC[1];
    int leituraAgua = leituraADC[2];

    int velocidade = (leituraVel * 50 * 5) / 1023;
    int direcao = map(leituraDir, 0, 1023, -180, 180);

    byte temperature = 0;
    byte humidity = 0;
    int err = dht11.read(&temperature, &humidity, NULL);
    float tempDHT = (err == 0) ? (float)temperature : NAN;
    float hum = (err == 0) ? (float)humidity : NAN;

    int32_t pressao = bmp.readPressure();

    byte flagChuva = (leituraAgua < LIMIAR_CHUVA) ? 1 : 0;

    // Ativação do servo por comando recebido via XBee
    if (((usart_rx_head + 64 - usart_rx_tail) % 64) >= 10) {
      char key = decodeAPIpacket();
      if (key == 'S') {
        OCR1A = 2000;
        servoAtivadoEm = millis();
        servoAtivo = true;
      }
    }

    // Ativação automática do servo
    bool ativarServo = false;

    if (millis() - ultimoTempoAtivacao >= intervaloAtivacao) {
      ativarServo = true;
      ultimoTempoAtivacao = millis();
    }

    if (flagChuva) {
      ativarServo = true;
    }

    if (ativarServo && !servoAtivo) {
      OCR1A = 2000;
      servoAtivadoEm = millis();
      servoAtivo = true;
    }

    // Retorno à posição inicial após 1 segundo
   if (servoAtivo && (millis() - servoAtivadoEm >= 1000) && leituraAgua >= LIMIAR_CHUVA) {
   OCR1A = 1100;
   servoAtivo = false;
  }

    sendSensorData(velocidade, direcao, leituraAgua, tempDHT, hum, pressao, rssi, flagChuva);
// ===== Verificações de Debug de valores =====
    USART_println("==== Leituras ====");
    USART_print("Velocidade: "); USART_printInt(velocidade); USART_println(" km/h");
    USART_print("Direcao: "); USART_printInt(direcao); USART_println(" graus");
    USART_print("Condutividade: "); USART_printInt(leituraAgua); USART_println("");
    USART_print("Temperatura: "); USART_printInt((int)tempDHT); USART_println(" °C");
    USART_print("Humidade: "); USART_printInt((int)hum); USART_println(" %");
    USART_print("Pressao: "); USART_printInt(pressao); USART_println(" Pa");
    USART_print("RSSI: -"); USART_printInt(rssi); USART_println(" dBm");
    USART_print("Chuva: "); USART_println(flagChuva ? "Sim" : "Nao");
    USART_println("==================");

    _delay_ms(2000);
  }
}

