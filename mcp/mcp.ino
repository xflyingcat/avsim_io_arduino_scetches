///////////////////////////////////////////////////
//                                               //
//          Arduino Mega2560 based MCP           //  
//                                               //
///////////////////////////////////////////////////

/*--------------------------------------------------------
  "THE BEER-WARE LICENSE" (Revision 42):
  Alex Kostyuk wrote this code. As long as you retain this 
  notice you can do whatever you want with this stuff. 
  If we meet some day, and you think this stuff is worth it, 
  you can buy me a beer in return.
----------------------------------------------------------*/
// Revision 0.7.0
/*
 In generally, MCP is just an application.
 Many details can be ajusted in the configuration file for 
 another purposes.
 
 More common firmware specification:
 ===================================
 Push buttons in matrix ...... up to 64
 Encoders ........................... 7
 Lamps (LEDs) .......................22
 Inputs for toggle switches ..........7
 7segm indicators 8 digits ...........4 (16)  

*/
/*-------------------------------------------------------------
W A R N I N G !
Put into the file
Arduino/hardware/arduino/avr/cores/arduino/HardwareSerial.h
after lines:

#ifndef HardwareSerial_h
#define HardwareSerial_h

these lines to increase the serial buffer size
     #define SERIAL_TX_BUFFER_SIZE 512
     #define SERIAL_RX_BUFFER_SIZE 1024
---------------------------------------------------------------*/

#define DEVICE_ID  0

#if SERIAL_TX_BUFFER_SIZE != 512
 #error SERIAL_TX_BUFFER_SIZE still is not 512 bytes
#endif

#if SERIAL_RX_BUFFER_SIZE != 1024
 #error SERIAL_RX_BUFFER_SIZE still is not 1024 bytes
#endif

#define BAUD_RATE 115200

#define TIMING_TEST   0
#define VSI_LED_INDEX 6

/****************** System ******************/
void setup(void);
void loop(void);
void serialEvent();

/****************** MAX7219 ******************/
char seven_segments_leds[16][8] = {
  /* indicators lines from simulator */
  {'1','5','1','5','1','5','1','5'},
  {'1','4','1','4','1','4','1','4'},
  {'1','3','1','3','1','3','1','3'},
  {'1','2','1','2','1','2','1','2'},
  {'1','1',' ','1','1',' ','1','1'},
  {'1','0','1','0','1','0','1','0'},
  {'9','9','9','9','9','9','9','9'},
  {'8','8','8','8','8','8','8','8'},
  {'7','7','7','7','7','7','7','7'},
  {'6','6','6','6','6','6','6','6'},
  {'5','5','5','5','5','5','5','5'},
  {'4','4','4','4','4','4','4','4'},
  {'3','3','3','3','3','3','3','3'},
  {'2','2','2','2','2','2','2','2'},
  {'1','1','1','1','1','1','1','1'},
  {'0','0','0','0','0','0','0','0'}
};
void max7219_setup(void);
void refresh_all_digits_n_lamps(void);
int crs_shift = 0;

/****************** LEDs ******************/
#define LEDS_MAX       22
#define NEW_BOARD

#ifdef NEW_BOARD
const char leds_pins[] = {
/* 0   1    2    3    4    5    6    7   8    9 */  
  18,  21,  23,  22,  24,  25,  A4,  7,  6,   11,   
  5,   13,  8,   12,  10,  40,  41, 45,  39,  44,  
  42,  43
};
#else
/* 0   1    2    3    4    5    6    7   8    9 */  
  18,  21,  23,  22,  24,  25,  A4,  5,  7,   6,   
  8,   12,  13,  11,  10,  39,  40, 41,  42,  43,  
  44,  45
};
#endif

  /* leds state from simulator (all of 64)*/
char single_leds[64] = {
  '0','0','0','0','0','0','0','0',
  '0','0','0','0','0','0','0','0',
  '0','0','0','0','0','0','0','0',
  '0','0','0','0','0','0','0','0',
  '0','0','0','0','0','0','0','0',
  '0','0','0','0','0','0','0','0',
  '0','0','0','0','0','0','0','0',
  '0','0','0','0','0','0','0','0'
};
void leds_init(void);
void led_control(int led, char state);

/******** Main discrete input task ********/
#define BITS_MAXIMUM   85 /*including the keys matrix nodes*/
void bits_init(void);
void discrete_task( void);

/******** Keys matrix ********/
void keys_matrix_init(void);
int  keys_scanner(void);
const char scan_table_x[] = {28,29,4,26,36,A7,35,47};
const char scan_table_y[] = {A8,A9,A10,A11,A12,A13,A14,A15};

/****************** Utilities ******************/
const char hex_table[] = "0123456789ABCDEF";

/******** Serial communication ********/
#define STX           '{'
#define ETX           '}'
#define RBUF_LEN       162 
char rbuf[RBUF_LEN];
int  rcnt;


void setup() 
{
    max7219_setup();      
    digitalWrite(13, HIGH);  
    #if TIMING_TEST 
        pinMode(A0, OUTPUT);
    #endif    
    keys_matrix_init();
    bits_init();
    leds_init();
    Serial.begin(BAUD_RATE);
    refresh_all_digits_n_lamps();
       
}  
  
void loop() 
{
    refresh_all_digits_n_lamps();
}

void serialEvent() 
{
    int ndx,itmp;  
    while (Serial.available()) 
    {
        char ch = (char)Serial.read();
        switch(ch)
        {
            case STX:
                rcnt = 0;
            break;

            case ETX:
                rbuf[rcnt] = 0x00;
        
                if(rbuf[0] == '7')
                {
                    memcpy(seven_segments_leds,rbuf+3,128);
                    #ifdef NEW_BOARD
                        seven_segments_leds[15][0] = 0x31 + crs_shift;
                        if(crs_shift)
                        {
                           seven_segments_leds[15][2] = seven_segments_leds[crs_shift][0];
                           seven_segments_leds[15][3] = seven_segments_leds[crs_shift][1];
                           seven_segments_leds[15][4] = seven_segments_leds[crs_shift][2];
                        }  
                        
                    #endif
                    if(seven_segments_leds[14][1] == ' ')
                    {
                        seven_segments_leds[14][0] = seven_segments_leds[0][0] + 128;
                        seven_segments_leds[14][1] = seven_segments_leds[0][1];
                        seven_segments_leds[14][2] = seven_segments_leds[0][2];
                    }

                    if(!(single_leds[VSI_LED_INDEX] & 1))
                    {
                        seven_segments_leds[13][3] = ' ';
                        seven_segments_leds[13][4] = ' ';
                        seven_segments_leds[13][5] = ' ';
                        seven_segments_leds[13][6] = ' ';
                        seven_segments_leds[13][7] = ' ';
                    }
                } 
                    
                if(rbuf[0] == 'L')   
                    memcpy(single_leds,rbuf+3,64);
                
                if(rbuf[0] == 'D')
                {
                   Serial.write('!');
                   Serial.write(STX);
                   Serial.write('F');
                   itmp = DEVICE_ID;
                   Serial.write(hex_table[itmp >> 4]);
                   Serial.write(hex_table[itmp & 0x0F]);
                   Serial.write(ETX);
                } 
                else
                    Serial.write('!');

            break;

            default:
                if(rcnt<RBUF_LEN)
                    rbuf[rcnt++] = ch;
        }
    }
}  


/*****************************************
           MAX7219 routines
*****************************************/
#define MAX7219_REG_DECODE_MODE  0x09
#define MAX7219_REG_INTENSITY    0x0A
#define MAX7219_REG_SCANLIMIT    0x0B
#define MAX7219_REG_SHUTDOWN     0x0C
#define MAX7219_REG_TEST         0x0F
#define MAX7219_DATAIN           34
#define MAX7219_CLOCK            33
#define MAX7219_LOAD             32
#define MAX7219_MAX              16

void max7219_write_byte(byte bt) 
{
byte i;
    for(i=0;i<8;i++)
    {
        digitalWrite(MAX7219_CLOCK, LOW); 
        if (bt & (0x80>>i))
           digitalWrite(MAX7219_DATAIN, HIGH);
        else
           digitalWrite(MAX7219_DATAIN, LOW);
        digitalWrite(MAX7219_CLOCK, HIGH);   
    }       
}
     
void max7219_reg_init (byte reg, byte val) 
{
    int i;
    digitalWrite(MAX7219_LOAD, LOW);
    for (i=0; i < MAX7219_MAX; i++) 
    {
        max7219_write_byte(reg);
        max7219_write_byte(val);
    }
    digitalWrite(MAX7219_LOAD, LOW);
    digitalWrite(MAX7219_LOAD,HIGH);
}


void max7219_set_intensity (void) 
{
    const unsigned char intensity_table[16] = 
    {0,0,0,0,0,0,0,0,
     0,0,0,0,1,0,1,1}; 
    int i;
    digitalWrite(MAX7219_LOAD, LOW);
    for (i=0; i < MAX7219_MAX; i++) 
    {
        max7219_write_byte(MAX7219_REG_INTENSITY);
        max7219_write_byte(intensity_table[i]);
    }
    digitalWrite(MAX7219_LOAD, LOW);
    digitalWrite(MAX7219_LOAD,HIGH);
}


/*
If we have only 4 max7219 and we need to save processor time, we
can assign MAX_7219_ACTUAL_NUMBER for our needs as 4 for the
current application. 
*/
#define MAX_7219_ACTUAL_NUMBER 4

void refresh_all_digits_n_lamps(void) 
{    
    int i,j;
    unsigned char bt;
    char *p;
    for(i=0;i<8;i++)
    {
        digitalWrite(MAX7219_LOAD, LOW);    
        for ( j = (MAX7219_MAX - MAX_7219_ACTUAL_NUMBER); j < MAX7219_MAX; j++) 
        {
           max7219_write_byte(i+1);  
           discrete_task();
           bt = seven_segments_leds[j][7-i];
           
           if(bt == ' ')
               bt = 0x0F;
           if(bt == '-')
               bt = 0x0A;
              
           max7219_write_byte(bt);
           discrete_task();
        }
        digitalWrite(MAX7219_LOAD, LOW);
        digitalWrite(MAX7219_LOAD,HIGH);
    }
     
    for(p = single_leds,i=0; i<LEDS_MAX; i++,p++)
    {
      led_control(i,*p);
    }

}

void max7219_setup(void)
{
int i; 
    pinMode(MAX7219_DATAIN, OUTPUT);
    pinMode(MAX7219_CLOCK,  OUTPUT);
    pinMode(MAX7219_LOAD,   OUTPUT);

    max7219_reg_init(MAX7219_REG_SCANLIMIT,   0x07);
    max7219_reg_init(MAX7219_REG_DECODE_MODE, 0xFF);
    max7219_reg_init(MAX7219_REG_SHUTDOWN,    0x01);
    max7219_reg_init(MAX7219_REG_TEST,        0x00);
    for(i=0; i<8; i++) 
    {    
        max7219_reg_init(i+1,0);
    }
    //max7219_reg_init(MAX7219_REG_INTENSITY, 0x00);
    max7219_set_intensity();
} 

/*****************************************
           Discrete outputs (LEDs)
*****************************************/
void leds_init(void)
{
int i;
    for(i=0;i<LEDS_MAX;i++)
    {
        if(leds_pins[i])
        {
            pinMode(leds_pins[i], OUTPUT); 
            digitalWrite(leds_pins[i], LOW);
        }
    }
} 

void led_control(int led, char state)
{
   if(led>=0 && led < LEDS_MAX)
   {
        if(leds_pins[led])
        {
            if(state & 1)
                digitalWrite(leds_pins[led], HIGH);
            else
                digitalWrite(leds_pins[led], LOW);
        }
   }
}



/*****************************************
           Discrete inputs
*****************************************/
#define FLAG_INVERTED   (1<<0)
#define FLAG_ENABLED    (1<<1)

enum {
   TOGGLE_SWITCH_0 = 0,
   TOGGLE_SWITCH_1,
   ITEMS_MAX
};

enum {
  PUSH_BUTTON = 0,
  PUSH_BUTTON_X,
  TOGGLE_SWITCH,
  ROTARY_SWITCH,
  ROTARY_SWITCH_X,
  ENCODER_A,
  ENCODER_B,
  SHIFT
};


typedef struct {
char counter;
char signal;
char  max;
short critical_time;
short critical_timer;
char evcode;
char flag;
char mode;
char pin;
char id;
} BITS_DESCRIPTION;

BITS_DESCRIPTION  bits[BITS_MAXIMUM];


/*********************************************
          Keys matrix routines
**********************************************/

/* keys_scanner() populates this table */
/* discrete_task() uses this table as a source of signals 
when bits[x].pin == 0 (zero means keys matrix node) */
unsigned char keys_state[64]; 

void keys_matrix_init(void)
{
int i;
    for(i=0;i<8;i++)
    {
       pinMode(scan_table_x[i], OUTPUT); 
       digitalWrite(scan_table_x[i], HIGH);
       pinMode(scan_table_y[i],INPUT_PULLUP);
    }  
    memset(keys_state,0,64);
    
}
static char x = 0;
int keys_scanner(void)
{
char i;        
   for(i=0;i<8;i++)
   {
     keys_state[x*8 + i] = digitalRead(scan_table_y[i]);
   }
   digitalWrite(scan_table_x[x], HIGH);
   x++;
   if(x>7)
      x = 0;
   digitalWrite(scan_table_x[x], LOW);
   if(x==0)
        return 1; /* scanning cycle completed */
return 0;        
}

/*********************************************
          Main discrete task
**********************************************/
/* takes 448 uS  */
char prev_pos = 0x19;
void discrete_task( void)
{
    int bit,flag;
    int bit_state;
    int i,itmp;
    #if TIMING_TEST
        digitalWrite(A0, HIGH);  
    #endif  
    keys_scanner();
    for(i=0;i<BITS_MAXIMUM;i++)
    {
       if(bits[i].flag & FLAG_ENABLED)
       {
           if(bits[i].critical_timer)
           {
               if(--bits[i].critical_timer == 0)
               {
                   Serial.write(STX);
                   Serial.write('1');
                   itmp = bits[i].id + 128;
                   Serial.write(hex_table[itmp >> 4]);
                   Serial.write(hex_table[itmp & 0x0F]);
                   Serial.write(ETX);
               }
           }
           
           if(bits[i].pin == 0)
               bit_state = keys_state[i];
           else
               bit_state = digitalRead(bits[i].pin);
               
           flag = bits[i].flag;
           bit = (flag & FLAG_INVERTED) ^ bit_state;
           if(bit)
           {
               if(bits[i].counter < bits[i].max)
               {
                   if(++(bits[i].counter) >= bits[i].max)
                   {
                       if(bits[i].signal == 0)
                       {
                           #ifdef NEW_BOARD
                              if(bits[i].id == 1)
                              {
                                 if(++crs_shift >= 3)
                                 {  
                                    crs_shift = 0; 
                                 }
                              }   

                           #endif
                           
                           //front __0__/--1--
                           
                           if(bits[i].critical_time)
                              bits[i].critical_timer = bits[i].critical_time;  
                           
                           if(bits[i].mode == ENCODER_A)
                           {
                               Serial.write(STX);
                               Serial.write('2');
                               itmp = bits[i].id + bits[i+1].signal;
                            #ifdef NEW_BOARD   
                               if((itmp & 0xFE) == 72 && crs_shift)
                               {
                                  itmp += ((crs_shift<<1) + 0x80);
                               } 
                            #endif   
                               Serial.write(hex_table[itmp >> 4]);
                               Serial.write(hex_table[itmp & 0x0F]);
                               Serial.write(ETX);
                            }
                            else if(bits[i].mode == ROTARY_SWITCH)
                            {
                                Serial.write(STX);
                                Serial.write('1');
                                itmp = bits[i].id;
                                Serial.write(hex_table[itmp >> 4]);
                                Serial.write(hex_table[itmp & 0x0F]);
                                Serial.write(ETX);
                            }
                            else if(bits[i].mode == ROTARY_SWITCH_X)
                            {
                                Serial.write(STX);
                                Serial.write('1');
                                itmp = bits[i].id;
                                Serial.write(hex_table[itmp >> 4]);
                                Serial.write(hex_table[itmp & 0x0F]);
                                Serial.write(ETX);

                                Serial.write(STX);
                                Serial.write('2');
                                if(prev_pos > bits[i].id)
                                    itmp = 0xF0;
                                else
                                    itmp = 0xF1; 
                                Serial.write(hex_table[itmp >> 4]);
                                Serial.write(hex_table[itmp & 0x0F]);
                                Serial.write(ETX);
                                prev_pos = bits[i].id;
                                
                            }  
                            else if(bits[i].mode != ENCODER_B && bits[i].mode != PUSH_BUTTON_X)
                            {
                                Serial.write(STX);
                                Serial.write('8');
                                itmp = bits[i].id;
                                Serial.write(hex_table[itmp >> 4]);
                                Serial.write(hex_table[itmp & 0x0F]);
                                Serial.write(ETX);
                            }
                        }
                        bits[i].signal = 1;
                    }
                }
            } 
            else
            {
                if(bits[i].counter > 0)
                {
                    if(--(bits[i].counter) <= 0)
                    {
                        if(bits[i].signal)
                        {
                            // edge  --1--\___0___
                            if( bits[i].mode == TOGGLE_SWITCH || 
                                bits[i].mode == PUSH_BUTTON )
                            {
                                Serial.write(STX);
                                Serial.write('4');
                                itmp = bits[i].id;
                                Serial.write(hex_table[itmp >> 4]);
                                Serial.write(hex_table[itmp & 0x0F]);
                                Serial.write(ETX);
                            } 
                            else if(bits[i].mode == PUSH_BUTTON_X && bits[i].critical_timer)
                            {
                                Serial.write(STX);
                                Serial.write('1');
                                itmp = bits[i].id;
                                Serial.write(hex_table[itmp >> 4]);
                                Serial.write(hex_table[itmp & 0x0F]);
                                Serial.write(ETX);
                            }
                            bits[i].critical_timer = 0;
                        }   
                        bits[i].signal = 0;
                    }
                }
            }
        }
    }
    #if TIMING_TEST    
        digitalWrite(A0, LOW);  
    #endif
}



/*********************************************
      Regular inputs setup macro definitions
**********************************************/
    
#define input_init(x,y,m,i)   do {\
bits[x].counter = 0;\
bits[x].signal = 0;\
bits[x].max = 50;\
bits[x].flag = (FLAG_ENABLED | FLAG_INVERTED);\
bits[x].pin = y;\
pinMode(y,INPUT_PULLUP);\
bits[x].mode = m;\
bits[x].id = i;\
} while(0)


#define input_init_rotary_switch(x,y,m,i)   do {\
bits[x].counter = 0;\
bits[x].signal = 0;\
bits[x].max = 30;\
bits[x].flag = (FLAG_ENABLED | FLAG_INVERTED);\
bits[x].pin = y;\
pinMode(y,INPUT_PULLUP);\
bits[x].mode = m;\
bits[x].id = i;\
} while(0)

#define input_init_fast(x,y,m,i)   do {\
bits[x].counter = 0;\
bits[x].signal = 0;\
bits[x].max = 3;\
bits[x].flag = (FLAG_ENABLED | FLAG_INVERTED);\
bits[x].pin = y;\
pinMode(y,INPUT_PULLUP);\
bits[x].mode = m;\
bits[x].id = i;\
} while(0)
   
/*****************************************
           Regular inputs setup
*****************************************/
void bits_init(void)
{  
    int i;
    memset(bits,0,sizeof(bits));
 
    /* The first 64 bits are states of the key matrix nodes
       which have bits[x].pin = 0;
    */
    for(i=0;i<64;i++)
       input_init(i,0,PUSH_BUTTON,i+1);
#ifdef NEW_BOARD    
    /* Regular microporcessor inputs */
    input_init(64,9,TOGGLE_SWITCH,0x41);  /* A/P DISENGAGE*/
    input_init(65,27,TOGGLE_SWITCH,0x42); /* F/D ON*/
    input_init(66,17,TOGGLE_SWITCH,0x43); /* A/T ARM*/
    input_init(67,53,TOGGLE_SWITCH,0x45); /* ADF1*/
    input_init(68,52,TOGGLE_SWITCH,0x44); /* VOR1*/
    input_init(69,A6,TOGGLE_SWITCH,0x47); /* ADF2*/
    input_init(70,A5,TOGGLE_SWITCH,0x46); /* VOR2*/
  
    input_init_fast(71,31,ENCODER_A,0x48); /* CRS */
    input_init_fast(72,30,ENCODER_B,0x49);

    input_init_fast(73,19,ENCODER_A,0x4A); /* IAS */
    input_init_fast(74,20,ENCODER_B,0x4B);

    input_init_fast(75,37,ENCODER_A,0x4C); /* HDG */
    input_init_fast(76,38,ENCODER_B,0x4D);
    
    input_init_fast(77,15,ENCODER_A,0x4E); /* VS */
    input_init_fast(78,16,ENCODER_B,0x4F);

    input_init_fast(79,3,ENCODER_A,0x50); /* ALT */
    input_init_fast(80,14,ENCODER_B,0x51);

    input_init_fast(81,49,ENCODER_A,0x52); /* BARO */
    input_init_fast(82,48,ENCODER_B,0x53);

    input_init_fast(83,50,ENCODER_A,0x54); /* MINS */
    input_init_fast(84,51,ENCODER_B,0x55);

    bits[6].critical_time = 1500;
    bits[8].critical_time = 1500;
    bits[11].critical_time = 1500;
    
    bits[60].critical_time = 1500;
    bits[61].critical_time = 1500;
    bits[62].critical_time = 1500;
    bits[63].critical_time = 1500;


    bits[6].mode = PUSH_BUTTON_X;
    bits[8].mode = PUSH_BUTTON_X;
    bits[11].mode = PUSH_BUTTON_X;

    bits[60].mode = PUSH_BUTTON_X;
    bits[61].mode = PUSH_BUTTON_X;
    bits[62].mode = PUSH_BUTTON_X;
    bits[63].mode = PUSH_BUTTON_X;

    bits[24].mode = ROTARY_SWITCH_X;
    bits[25].mode = ROTARY_SWITCH_X;
    bits[26].mode = ROTARY_SWITCH_X;
    bits[27].mode = ROTARY_SWITCH_X;
    bits[28].mode = ROTARY_SWITCH_X;
    bits[29].mode = ROTARY_SWITCH_X;

    bits[40].mode = ROTARY_SWITCH;
    bits[41].mode = ROTARY_SWITCH;
    bits[42].mode = ROTARY_SWITCH;
    bits[43].mode = ROTARY_SWITCH;

    bits[48].mode = ROTARY_SWITCH;
    bits[49].mode = ROTARY_SWITCH;
    bits[50].mode = ROTARY_SWITCH;
    bits[51].mode = ROTARY_SWITCH;
    bits[52].mode = ROTARY_SWITCH;
    bits[53].mode = ROTARY_SWITCH;
    bits[54].mode = ROTARY_SWITCH;
    bits[55].mode = ROTARY_SWITCH;
#else
    /* Regular microporcessor inputs */
    input_init(64,9,TOGGLE_SWITCH,0x41);  /* A/P DISENGAGE*/
    input_init(65,27,TOGGLE_SWITCH,0x42); /* F/D ON*/
    input_init(66,17,TOGGLE_SWITCH,0x43); /* A/T ARM*/
    input_init(67,53,TOGGLE_SWITCH,0x44); /* ADF1*/
    input_init(68,52,TOGGLE_SWITCH,0x45); /* VOR1*/
    input_init(69,A6,TOGGLE_SWITCH,0x46); /* ADF2*/
    input_init(70,A5,TOGGLE_SWITCH,0x47); /* VOR2*/
  
    input_init_fast(71,30,ENCODER_A,0x48); /* CRS */
    input_init_fast(72,31,ENCODER_B,0x49);

    input_init_fast(73,20,ENCODER_A,0x4A); /* IAS */
    input_init_fast(74,19,ENCODER_B,0x4B);

    input_init_fast(75,37,ENCODER_A,0x4C); /* HDG */
    input_init_fast(76,38,ENCODER_B,0x4D);
    
    input_init_fast(77,15,ENCODER_A,0x4E); /* VS */
    input_init_fast(78,16,ENCODER_B,0x4F);

    input_init_fast(79,14,ENCODER_A,0x50); /* ALT */
    input_init_fast(80,3,ENCODER_B,0x51);

    input_init_fast(81,48,ENCODER_A,0x52); /* BARO */
    input_init_fast(82,49,ENCODER_B,0x53);

    input_init_fast(83,51,ENCODER_A,0x54); /* MINS */
    input_init_fast(84,50,ENCODER_B,0x55);
    
    bits[60].critical_time = 1500;
    bits[61].critical_time = 1500;
    bits[62].critical_time = 1500;
    bits[63].critical_time = 1500;

    bits[60].mode = PUSH_BUTTON_X;
    bits[61].mode = PUSH_BUTTON_X;
    bits[62].mode = PUSH_BUTTON_X;
    bits[63].mode = PUSH_BUTTON_X;

    bits[24].mode = ROTARY_SWITCH_X;
    bits[25].mode = ROTARY_SWITCH_X;
    bits[26].mode = ROTARY_SWITCH_X;
    bits[27].mode = ROTARY_SWITCH_X;
    bits[28].mode = ROTARY_SWITCH_X;
    bits[29].mode = ROTARY_SWITCH_X;

    bits[40].mode = ROTARY_SWITCH;
    bits[41].mode = ROTARY_SWITCH;
    bits[42].mode = ROTARY_SWITCH;
    bits[43].mode = ROTARY_SWITCH;

    bits[48].mode = ROTARY_SWITCH;
    bits[49].mode = ROTARY_SWITCH;
    bits[50].mode = ROTARY_SWITCH;
    bits[51].mode = ROTARY_SWITCH;
    bits[52].mode = ROTARY_SWITCH;
    bits[53].mode = ROTARY_SWITCH;
    bits[54].mode = ROTARY_SWITCH;
    bits[55].mode = ROTARY_SWITCH;
#endif
    
} 



