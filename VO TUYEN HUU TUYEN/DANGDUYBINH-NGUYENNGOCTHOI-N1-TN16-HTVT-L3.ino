
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define MIC_PIN   0
#define BTN_PIN   2
#define LED_PIN   7
#define TIME_OUT  10000

typedef enum
{
  READY,
  WAITING,
  RINGING,
  RUNNING
}
mode_t;

void          adc_init                      (void);
void          pwm_init                      (void);
uint8_t       adc_read                      (uint8_t channel);
void          pwm_write                     (uint8_t value);
inline void   waiting_state_work            (void);
inline void   ringing_state_work            (void);
inline int    running_state_work            (void);
inline void   send_command                  (String str);
inline bool   recv_command                  (String str);
void          pin_ISR                       (void);

volatile byte adc;
volatile mode_t state = READY, previous_state = RUNNING;
volatile int16_t count = TIME_OUT, led = 0;

void setup()
{
  pinMode(BTN_PIN,INPUT);
  pinMode(LED_PIN,OUTPUT);
  Serial.begin(115200);
  adc_init();      
  pwm_init();       
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), pin_ISR, LOW);
  delay(5000);
}

void loop()
{
  switch (state)
  {
    case READY:
      if (recv_command("BT+CAL"))
        state = RINGING;
      digitalWrite(LED_PIN, LOW);
      break;
    case WAITING:
      if (recv_command("BT+ACC"))
        state = RUNNING;
      digitalWrite(LED_PIN, HIGH);
      break;
    case RINGING:
      if (recv_command("BT+END"))
        state = READY;
      digitalWrite(LED_PIN, HIGH);
      ringing_state_work();
      break;
    case RUNNING:
        if (running_state_work() == 0)
          state = READY;
        led++;
        if (led == 1000)
        {
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          led = 0;
        }
      break;
  }
  switch (previous_state)
  {
    case READY:
      send_command("BT+CAL");
      previous_state = RUNNING;
      break;
    case WAITING:
      send_command("BT+END");
      previous_state = RUNNING;
      break;
    case RINGING:
      send_command("BT+ACC");
      previous_state = RUNNING;
      break;
    case RUNNING:
      break;
  }


}

void adc_init(void)
{
  // Select Vref=AVcc
  ADMUX |= _BV(REFS0);
  //set prescaller to 128 and enable ADC
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADEN); 
}

uint8_t adc_read(uint8_t channel) 
{
  //Clear the older channel that was read
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); 
  //Defines the new ADC channel to be read      
  ADMUX |= channel;    
  //Starts a new conversion            
  ADCSRA |= _BV(ADSC);          
  //Wait until the conversion is done    
  while (ADCSRA & _BV(ADSC));      
  //Returns the ADC value of the chosen channel   
  return ADC >> 2;                    
}

void pwm_init(void) 
{
  // Set PB1 as outputs.
  DDRB |= _BV(DDB1);
  TCCR1A =
      _BV(COM1A1) | _BV(COM1B1) |
      // Fast PWM mode.
      _BV(WGM11);
  TCCR1B =
      // Fast PWM mode.
      _BV(WGM12) | _BV(WGM13) |
      // No clock prescaling (fastest possible
      // freq).
      _BV(CS10);
  OCR1A = 0;
  // Set the counter value that corresponds to
  // full duty cycle. For 15-bit PWM use
  // 0x7fff, etc. A lower value for ICR1 will
  // allow a faster PWM frequency.
  ICR1 = 0x00ff;
}

void pwm_write(uint8_t value)
{
  // Use OCR1A to control the PWM
  // duty cycle. Duty cycle = OCR1A / ICR1.
  // OCR1A controls PWM on pin 9 (PORTB1).
  OCR1A = value; 
}

inline void waiting_state_work()
{

}

inline void ringing_state_work()
{
   pwm_write(0xFF);
    delay(1);
    pwm_write(0x00);
    delay(1); 
}

inline int running_state_work()
{
  Serial.write(adc_read(MIC_PIN));
  if (Serial.available())
  {
    pwm_write(Serial.read());
    count = TIME_OUT;
  }
  else
    count--;
  if (count == 0)
  {
    count = TIME_OUT;
    return 0;
  }
  else
    return 1;
}

inline void send_command(String str)
{
  for (int i = 0; i< 20; i++)
    Serial.print(str);
  delay(1);
}

inline bool  recv_command(String str)
{
  char recvc;
  String str2 = "";
  if (Serial.available())
  {
    for (int i = 0; i < 6; i++)
    {
      recvc = (char)Serial.read();
      str2+=recvc;
      delay(10);
    }
  }
  //Serial.flush();
  if (strncmp(str.c_str(),str2.c_str(),6) == 0)
    return true;
  else 
    return false;
}

void pin_ISR() 
{
  //digitalWrite(LED_PIN,!digitalRead(LED_PIN));
     switch (state)
  {
    case READY:
      previous_state = READY;
      state = WAITING;
      break;
    case WAITING:
      previous_state = WAITING;
      state = READY;
      break;
    case RINGING:
      previous_state = RINGING;
      state = RUNNING;
      break;
    case RUNNING:
      previous_state = RUNNING;
      state = READY;
      pwm_write(0x00);
      break;
  }
  while(!digitalRead(BTN_PIN));
}
