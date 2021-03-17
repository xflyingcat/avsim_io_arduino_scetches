
#define BUS_ADDRESS 2

#define TXDEN  20

enum {
  STEPPER_POWER_UP,
  STEPPER_IDLE,
  STEPPER_GO_HOME,
  STEPPER_FOLLOW_REF
};

typedef int (*APPROX_FUNC)(int x);

typedef struct {
   char dir_pin;
   char clk_pin;
   char ena_pin;
   unsigned long ref_time;
   char state;
   int pos;
   int ref;
   int offset;
   int max;
   int input_pos;
   APPROX_FUNC approx_func;
} STEPPER_DRIVER_ENV;

/*****************************************************/
/*    Put appriximation functions below this header  */
/*****************************************************/

#define MOTORS_MAX   2
STEPPER_DRIVER_ENV stepper_drv[MOTORS_MAX] = 
{
   /*dir, clk, ena, --,  initial state,   --,  --, offset, max, --, func*/  
   { 4,   3,    2,   0,  STEPPER_POWER_UP, 0,   0,  0,    2000, 0,   NULL },
   { 5,   6,    7,   0,  STEPPER_POWER_UP, 0,   0,  0,    2000, 0,   NULL },
};

void setup(void);
void loop(void);
void serialEvent();
int parser(char in);
void send_response(unsigned char response_code);
void max7219_setup(void);
void refresh_all(void);

char seven_segments_leds[6][8];

int stepper_driver_periodic(STEPPER_DRIVER_ENV *env, int new_pos);
void stepper_driver_disable(STEPPER_DRIVER_ENV *env);
void stepper_driver_enable(STEPPER_DRIVER_ENV *env);
void one_step(STEPPER_DRIVER_ENV *env, char dir);

ISR (USART1_TX_vect)
{
   digitalWrite(TXDEN, LOW);
}

void setup() 
{
    /* https://horiokanta.com/en/arduino-and-rs485/ */
    UCSR1B |= (1<<TXCIE1);     // TX Complete Interrupt Enable 1
    pinMode(TXDEN, OUTPUT);
    pinMode(A0, OUTPUT);
    Serial.begin(115200);
}  


unsigned char input_data[16];

void loop() 
{
  int pos;
  int i;
  APPROX_FUNC f;
  for(i=0;i<MOTORS_MAX;i++)
  {
     f = stepper_drv[i].approx_func;
     pos = stepper_drv[i].input_pos + stepper_drv[i].offset;
     if(f)
       pos = f(pos);
     stepper_driver_periodic(&stepper_drv[i], pos);   
  }
}

void serialEvent() 
{
    while (Serial.available()) 
    {
       parser((unsigned char)Serial.read());
    }

  
}


#define STX 0x02
#define ETX 0x03
#define DLE 0x10
#define DLE_STUFFING_BYTE 0x40
#define PARSER_BUF_LEN 100

enum
{
    DI_REQUEST = 0x40,
    DO_REQUEST = 0x41,
    NO_REQUEST = 0x42
};


int parser(char in)
{
    static char dle_flag = 0;
    static char ndx;
    static char buf[PARSER_BUF_LEN];
    static unsigned char lrc;
    int j;
    switch(in)
    {
    case STX:
        dle_flag = 0;
        ndx = 0;
        lrc = 0;
               
        break;
    case ETX:

       if(lrc == 0)
        {
            if(buf[0] == BUS_ADDRESS && buf[1] == NO_REQUEST)
            {
                digitalWrite(A0, 1);
                send_response(NO_REQUEST);
                memcpy(seven_segments_leds,&buf[3],sizeof(seven_segments_leds));
                digitalWrite(A0, 0);
                
               /*  0,0  0,5  1,2 1,7  2,4   3,1  3,6  4,3  5,0
                   0       1       2       3       4       5          
                   012345670123456701234567012345670123456701234567 
                   1234=1234=1234=1234=1234=1234=1234=1234=1234=
               */ 
                char *p = (char*)seven_segments_leds;
                for(j=0;j<MOTORS_MAX;j++)
                {
                   p[4] = 0;
                   stepper_drv[j].input_pos = atoi(p);
                   p += 5;
                }                
            }
        }
        break;
    case DLE:
        dle_flag = 1;
        break;
    default:
        if(dle_flag)
        {
            in = in ^ DLE_STUFFING_BYTE;
            dle_flag = 0;
        }
        lrc += in;
        if(ndx < PARSER_BUF_LEN)
            buf[ndx++] = in;
    }

    return 0;
}

int add_byte(unsigned char *p, unsigned char *lrc, unsigned char data)
{
    if(lrc)
        *lrc += data;

    if((data == DLE) || (data == STX) || (data == ETX))
    {
        *p++ = DLE;
        *p++ = (data ^ DLE_STUFFING_BYTE);
        return 2;
    }
    *p++ = data;
    return 1;
}

static unsigned char tx_buffer[48];

void send_response(unsigned char response_code)
{
    unsigned char *txp = tx_buffer;
    unsigned char lrc;
    char i;
    lrc = 0;
    *txp++ = STX;
    
    txp += add_byte(txp, &lrc, BUS_ADDRESS);
    txp += add_byte(txp, &lrc, response_code);
    txp += add_byte(txp, &lrc, 0);

    txp += add_byte(txp, NULL, (lrc ^ 0xFF) + 1);

    *txp++ = ETX;
    
    digitalWrite(TXDEN, HIGH);
    int len = txp-tx_buffer;
    Serial.write(tx_buffer,len);
}

void stepper_driver_enable(STEPPER_DRIVER_ENV *env)
{
  digitalWrite(env->ena_pin, LOW);
}

void stepper_driver_disable(STEPPER_DRIVER_ENV *env)
{
  digitalWrite(env->ena_pin, HIGH);
}

void one_step(STEPPER_DRIVER_ENV *env, char dir)
{
  digitalWrite(env->dir_pin, dir);
  digitalWrite(env->clk_pin, HIGH);
  digitalWrite(env->clk_pin, HIGH);
  digitalWrite(env->clk_pin, HIGH);
  digitalWrite(env->clk_pin, LOW);
}


int stepper_driver_periodic(STEPPER_DRIVER_ENV *env, int new_pos)
{
  int dir;
  if ( (millis() - env->ref_time) > 1)
  {
    switch (env->state)
    {
      case STEPPER_POWER_UP:

        pinMode(env->clk_pin, OUTPUT);
        pinMode(env->dir_pin, OUTPUT);
        pinMode(env->ena_pin, OUTPUT);
        digitalWrite(env->dir_pin, HIGH);
        digitalWrite(env->clk_pin, LOW);
        digitalWrite(env->ena_pin, LOW);

      #if 0
        env->ref = new_pos;
        env->pos = new_pos;
        env->state = STEPPER_FOLLOW_REF;
      #else
        env->ref = 0;
        env->pos = env->max;
        env->state = STEPPER_GO_HOME;
      #endif
        
        break;
      case STEPPER_IDLE:
        env->ref = new_pos;
        env->state = STEPPER_FOLLOW_REF;
        break;

      case STEPPER_GO_HOME:
        if (env->pos < env->ref)
        {
          env->pos++;
          one_step(env,0);
        }
        else if (env->pos > env->ref)
        {
          env->pos--;
          one_step(env,0);          
        }
        else
        {
          env->pos = 0;
          env->state = STEPPER_IDLE;
        }

        break;

      case STEPPER_FOLLOW_REF:
        env->ref = new_pos;
        if (env->ref > env->max)
          env->ref = env->max;
        if (env->ref < 0)
          env->ref = 0;
        if (env->pos < env->ref)
        {
          env->pos++;
          one_step(env,1);
        }
        else if (env->pos > env->ref)
        {
          env->pos--;
          one_step(env, 0);
        }
        else
        {
          env->state = STEPPER_IDLE;
        }


        break;

    }
    env->ref_time = millis();
  }

  return env->state;
}  





