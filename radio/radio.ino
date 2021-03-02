///////////////////////////////////////////////////
//                                               //
//  Arduino Mega2560 based Radiostack panels     //  
//                                               //
///////////////////////////////////////////////////
/*--------------------------------------------------------
  "THE BEER-WARE LICENSE" (Revision 42):
  Alex Kostyuk wrote this code. As long as you retain this 
  notice you can do whatever you want with this stuff. 
  If we meet some day, and you think this stuff is worth it, 
  you can buy me a beer in return.
----------------------------------------------------------*/
// Revision 0.3.0 2020/04/08

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

#define DEVICE_ID 3

#if SERIAL_TX_BUFFER_SIZE != 512
 #error SERIAL_TX_BUFFER_SIZE still is not 512 bytes
#endif

#if SERIAL_RX_BUFFER_SIZE != 1024
 #error SERIAL_RX_BUFFER_SIZE still is not 1024 bytes
#endif

#define BAUD_RATE 115200

#define TIMING_TEST     1
#define TIMING_TEST_PIN 4

#define UNITS_NUMBER    1

/****************** System ******************/
void setup(void);
void loop(void);
void serialEvent();

/************ Indication buffer ***************/
#define DEBUG_MODE 0
char seven_segments_leds[16][8] = {
#if DEBUG_MODE == 1
  /* indicators lines from simulator 
  (it is thinking that sends to the 7 segments indicator lines)
  */
  {' ',' ',' ',' ',' ',' ',' ',' '},
  {' ',' ',' ',' ',' ',' ',' ',' '},
  {' ',' ',' ',' ',' ',' ',' ',' '},
  {'F','F','F','F','H','H','H','H'}, /* 03 PRESS(4)    HPA(4)  12 */
  {'D','D','D','D','E','E','E','E'}, /* 04 DME1(4)     DME2(4) 11 */
  {'C','C','C','C','X','X','X','X'}, /* 05 DME_DIST(4) XPND(4) 10 */
  {'A','A','A','A','A','M','M','M'}, /* 06 DME_FREQ(5) MM_HG(3)09 */
  {'9','9','9','9','B','B','B','B'}, /* 07 ADF1(4)     ADF2(4) 08 */ 
  {'8','8','8','8','8',' ',' ',' '}, /* 08 NAV2 STB(5)         07 */
  {'7','7','7','7','7','C','R','2'}, /* 09 NAV2(5)     CRS2(3) 06 */
  {'6','6','6','6','6',' ',' ',' '}, /* 10 NAV1_STB(5)         05 */
  {'5','5','5','5','5','C','R','1'}, /* 11 NAV1(5)     CRS1(3) 04 */
  {'4','4','4','4','4',' ',' ',' '}, /* 12 COM2_STB(5)         03 */
  {'3','3','3','3','3',' ',' ',' '}, /* 13 COM2(5)             02 */
  {'2','2','2','2','2',' ',' ',' '}, /* 14 COM1_STB(5)         01 */
  {'1','1','1','1','1',' ',' ',' '}  /* 15 COM1(5)             00 */
#else
  {' ',' ',' ',' ',' ',' ',' ',' '},
  {' ',' ',' ',' ',' ',' ',' ',' '},
  {' ',' ',' ',' ',' ',' ',' ',' '},
  {'0','0','0','0','0','0','0','0'}, /* 03 PRESS(4)    HPA(4)  12 */
  {'0','0','0','0','0','0','0','0'}, /* 04 DME1(4)     DME2(4) 11 */
  {'0','0','0','0','0','0','0','0'}, /* 05 DME_DIST(4) XPND(4) 10 */
  {'0','0','0','0','0','0','0','0'}, /* 06 DME_FREQ(5) MM_HG(3)09 */
  {'0','0','0','0','0','0','0','0'}, /* 07 ADF1(4)     ADF2(4) 08 */ 
  {'0','0','0','0','0',' ',' ',' '}, /* 08 NAV2 STB(5)         07 */
  {'0','0','0','0','0','0','0','0'}, /* 09 NAV2(5)     CRS2(3) 06 */
  {'0','0','0','0','0',' ',' ',' '}, /* 10 NAV1_STB(5)         05 */
  {'0','0','0','0','0','0','0','0'}, /* 11 NAV1(5)     CRS1(3) 04 */
  {'0','0','0','0','0',' ',' ',' '}, /* 12 COM2_STB(5)         03 */
  {'0','0','0','0','0',' ',' ',' '}, /* 13 COM2(5)             02 */
  {'0','0','0','0','0',' ',' ',' '}, /* 14 COM1_STB(5)         01 */
  {'0','0','0','0','0',' ',' ',' '}  /* 15 COM1(5)             00 */
#endif  
};

/**************** LCD *********************/
#include <LiquidCrystalFast.h>

#define RS0   22
#define RW0   23
#define EN0   25

#define RS1   27
#define RW1   29
#define EN1   31

#define DATA0 24 
#define DATA1 26 
#define DATA2 28 
#define DATA3 30

LiquidCrystalFast lcd1(RS0,RW0, EN0, DATA0, DATA1, DATA2, DATA3);
#if UNITS_NUMBER == 2
LiquidCrystalFast lcd2(RS1,RW1, EN1, DATA0, DATA1, DATA2, DATA3);
#endif
void lcd_init(void);
void line1_modification(char *line, char mode, char tune_step, char stb_act);
void line2_modification(char *line, char mode, char tune_step, char stb_act);

/****************** LEDs ******************/
#if UNITS_NUMBER == 1
#define LEDS_MAX   1
const char leds[] = 
{
    13
};
#endif

#if UNITS_NUMBER == 2
#define LEDS_MAX   2
const char leds[] = 
{
    13,
    51
};
#endif



  /* leds state from simulator (all of 64)*/
char single_leds[64] = 
{
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
#if UNITS_NUMBER == 2
#define BITS_MAXIMUM   42
#endif

#if UNITS_NUMBER == 1
#define BITS_MAXIMUM   46
#endif

typedef struct {
    int counter;
    char signal;
    int  max;
    char evcode;
    char flag;
    char mode;
    int pin;
    int id;
} BITS_DESCRIPTION;

BITS_DESCRIPTION  bits[BITS_MAXIMUM];
void bits_init(void);
void discrete_task( void);

/****************** Utilities ******************/
const char hex_table[] = "0123456789ABCDEF";

/******** Serial communication ********/
#define STX           '{'
#define ETX           '}'
#define RBUF_LEN       200 
char rbuf[RBUF_LEN];
int  rcnt;


/****************** App definitions ******************/

#if UNITS_NUMBER == 1
#define SIGNAL_UNIT_1_TUNE_STEP      2
#define SIGNAL_UNIT_1_STDBY_ACT_FREQ 3
#endif

#if UNITS_NUMBER == 2
#define SIGNAL_UNIT_1_TUNE_STEP      2
#define SIGNAL_UNIT_1_STDBY_ACT_FREQ 6

#define SIGNAL_UNIT_2_STDBY_ACT_FREQ 7
#define SIGNAL_UNIT_2_TUNE_STEP      5
#endif
const char step_of_mode_table[] = 
    {
    /* MODE_COM1 */      2,
    /* MODE_COM2 */      2,
    /* MODE_NAV1 */      2,
    /* MODE_NAV2 */      2,
    /* MODE_ADF */       3,
    /* MODE_DME */       2,
    /* MODE TXPND */     4,
    /* MODE_NAV_CRS */   2
    };

const char mask_of_mode_table[] = 
    {
    /* MODE_COM1 */      0x0C,
    /* MODE_COM2 */      0x0C,
    /* MODE_NAV1 */      0x0C,
    /* MODE_NAV2 */      0x0C,
    /* MODE_ADF */       0x0E,
    /* MODE_DME */       0x0E,
    /* MODE TXPND */     0x0C,
    /* MODE_NAV_CRS */   0x0E
    };

enum {
    MODE_COM1 = 0,
    MODE_COM2,
    MODE_NAV1,
    MODE_NAV2,
    MODE_ADF,
    MODE_DME,
    MODE_TXPND,
    MODE_NAV_CRS
 };

char unit1_mode = 0;
char unit2_mode = 0;
char unit1_step = 0;
char unit2_step = 0;
char unit1_stb_act = 0;
char unit2_stb_act = 0;

/********************************************************/


void setup(void)
{
    pinMode(13,OUTPUT);

    #if TIMING_TEST 
        pinMode(TIMING_TEST_PIN, OUTPUT);
    #endif 

    lcd_init(); 
    bits_init();
    leds_init();
    Serial.begin(115200);

}

void loop(void)
{
    int cur_time;
    short tmp;
    char line[17] = "                ",*p;
    static char ndx = 0;
    char i;

#if UNITS_NUMBER == 1
    #define STB_ACT_UNIT1_SIGNAL_INDEX  19
#endif

#if UNITS_NUMBER == 2
    #define STB_ACT_UNIT1_SIGNAL_INDEX  31
    #define STB_ACT_UNIT2_SIGNAL_INDEX  40
#endif



    if((single_leds[0] & 1) && bits[STB_ACT_UNIT1_SIGNAL_INDEX].signal)
        led_control(0,1);
    else
        led_control(0,0);
        
    line1_modification(line, unit1_mode, unit1_step, unit1_stb_act);

    lcd1.setCursor(0, 0);
    for(i=0,ndx=0;i<8;i++,ndx+=2)
    {
        lcd1.write(&line[ndx],2); 
        discrete_task();
    }

    line2_modification(line, unit1_mode, unit1_step, unit1_stb_act);

    lcd1.setCursor(0, 1);
    for(i=0,ndx=0;i<8;i++,ndx+=2)
    {
        lcd1.write(&line[ndx],2); 
        discrete_task();
    }

#if UNITS_NUMBER == 2

    if((single_leds[1] & 1) && bits[STB_ACT_UNIT2_SIGNAL_INDEX].signal)
        led_control(1,1);
    else
        led_control(1,0);

    line1_modification(line, unit2_mode, unit2_step, unit2_stb_act);

    lcd2.setCursor(0, 0);
    for(i=0,ndx=0;i<8;i++,ndx+=2)
    {
        lcd2.write(&line[ndx],2); 
        discrete_task();
    }

    line2_modification(line, unit2_mode, unit2_step, unit2_stb_act);

    lcd2.setCursor(0, 1);
    for(i=0,ndx=0;i<8;i++,ndx+=2)
    {
        lcd2.write(&line[ndx],2); 
        discrete_task();
    }
#endif
   
}


void serialEvent() 
{
    int ndx,i,itmp;  
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
                    memcpy(seven_segments_leds,rbuf+3,128);
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
           LCD routines
*****************************************/
void lcd_init(void)
{
  lcd1.begin(16, 2);
#if UNITS_NUMBER == 2   
  lcd2.begin(16, 2); 
#endif  
}

void line1_modification(char *line, char mode, char tune_step, char stb_act)
{
  switch(mode)
  {
    
    case MODE_COM1:
       strcpy(line,"COM1    000.000 ");
       memcpy(&line[8],&seven_segments_leds[15][0],3);
       memcpy(&line[12],&seven_segments_leds[15][3],2);
       if(line[13] == '2' || line[13] == '7')
           line[14] = '5';
    break;

    case MODE_COM2:
       strcpy(line,"COM2    000.000 "); 
       memcpy(&line[8],&seven_segments_leds[13][0],3);
       memcpy(&line[12],&seven_segments_leds[13][3],2);
       if(line[13] == '2' || line[13] == '7')
           line[14] = '5';
    break;

    case MODE_NAV1:
       strcpy(line,"NAV1    000.000 "); 
       memcpy(&line[8],&seven_segments_leds[11][0],3);
       memcpy(&line[12],&seven_segments_leds[11][3],2);
       if(line[13] == '2' || line[13] == '7')
           line[14] = '5';    
    break;

    case MODE_NAV2:
       strcpy(line,"NAV2    000.000 "); 
       memcpy(&line[8],&seven_segments_leds[9][0],3);
       memcpy(&line[12],&seven_segments_leds[9][3],2);
       if(line[13] == '2' || line[13] == '7')
           line[14] = '5';    
    break;    

    case MODE_ADF:
          if(stb_act)
          { 
              strcpy(line,"ADF2      000   "); 
              memcpy(&line[9],&seven_segments_leds[7][4],4);
          }    
          else
          {    
              strcpy(line,"ADF1      000   "); 
              memcpy(&line[9],&seven_segments_leds[7][0],4);
          }    
    break;

    case MODE_DME:
          if(stb_act)
          {
              strcpy(line,"DME1    000.0 nm"); 
              memcpy(&line[8],&seven_segments_leds[4][0],3); 
              line[12] = seven_segments_leds[4][3];

          }    
          else
          {
              strcpy(line,"DME     000.000 ");
              memcpy(&line[8],&seven_segments_leds[6][0],3);
              memcpy(&line[12],&seven_segments_leds[6][3],2);
              if(line[13] == '2' || line[13] == '7')
                  line[14] = '5';
          }    
              
    break;

    case MODE_TXPND:
             strcpy(line,"XPDR     0000   ");
             memcpy(&line[9], &seven_segments_leds[5][4],4);
    break;

    case MODE_NAV_CRS:
          if(stb_act)
          {
            strcpy(line,"NAV2 CRS   000  "); 
            memcpy(&line[11], &seven_segments_leds[9][5],3);
          }  
          else
          {  
            strcpy(line,"NAV1 CRS   000  ");
            memcpy(&line[11], &seven_segments_leds[11][5],3);
          }  
          if(tune_step)
          {
             line[10] = '<';
             line[14] = '>';
          }
    break;
  }
}

void line2_modification(char *line, char mode, char tune_step, char stb_act)
{
  switch(mode)
  {
  
    case MODE_COM1:
       if(tune_step)
         strcpy(line,"STBY   <000>000 "); 
       else  
         strcpy(line,"STBY    000<000>"); 

       memcpy(&line[8],&seven_segments_leds[14][0],3);
       memcpy(&line[12],&seven_segments_leds[14][3],2);
       if(line[13] == '2' || line[13] == '7')
           line[14] = '5';  
    break;


    case MODE_COM2:
       if(tune_step)
         strcpy(line,"STBY   <000>000 "); 
       else  
         strcpy(line,"STBY    000<000>"); 
       memcpy(&line[8],&seven_segments_leds[12][0],3);
       memcpy(&line[12],&seven_segments_leds[12][3],2);
       if(line[13] == '2' || line[13] == '7')
           line[14] = '5';    

    break;

    case MODE_NAV1:
       if(tune_step)
         strcpy(line,"STBY   <000>000 "); 
       else  
         strcpy(line,"STBY    000<000>"); 

       memcpy(&line[8],&seven_segments_leds[10][0],3);
       memcpy(&line[12],&seven_segments_leds[10][3],2);
       if(line[13] == '2' || line[13] == '7')
           line[14] = '5';
    
    
    break;


    case MODE_NAV2:
       if(tune_step)
         strcpy(line,"STBY   <000>000 "); 
       else  
         strcpy(line,"STBY    000<000>"); 
       memcpy(&line[8],&seven_segments_leds[8][0],3);
       memcpy(&line[12],&seven_segments_leds[8][3],2);
       if(line[13] == '2' || line[13] == '7')
           line[14] = '5';

    break;


    case MODE_ADF:
         strcpy(line,"                ");
         line[10 + tune_step] = '^';
    break;

    case MODE_DME:
          if(stb_act)
          {
              strcpy(line,"DME2    000.0 nm"); 
              memcpy(&line[8],&seven_segments_leds[4][4],3); 
              line[12] = seven_segments_leds[4][7];
          }
          else
          {
              strcpy(line,"000.0 nm        "); 
              memcpy(&line[0],&seven_segments_leds[5][0],3); 
              line[4] = seven_segments_leds[5][3];
              
              if(tune_step)
                  line[10] = '^';
              else    
                  line[14] = '^';
          }    
    break;

    case MODE_TXPND:
         strcpy(line,"                ");
         line[9 + tune_step] = '^';
    break;

    case MODE_NAV_CRS:
       if(!tune_step)
         strcpy(line,"<00.00 0000 000>"); 
       else  
         strcpy(line," 00.00 0000 000 "); 

       memcpy(&line[1],&seven_segments_leds[3][0],2);
       memcpy(&line[4],&seven_segments_leds[3][2],2);

       memcpy(&line[7],&seven_segments_leds[3][4],4);
       memcpy(&line[12],&seven_segments_leds[6][5],3);
       
         
    break;
  }
}  

/*****************************************
           Discrete outputs (LEDs)
*****************************************/


void leds_init(void)
{
int i;
    for(i=0;i<LEDS_MAX;i++)
    {
      pinMode(leds[i], OUTPUT); 
      digitalWrite(leds[i], LOW);
    }

}


void led_control(int led, char state)
{
	   if(led>=0 && led < LEDS_MAX)
	   {
	      if(state & 1)
	        digitalWrite(leds[led], HIGH);
	      else
	        digitalWrite(leds[led], LOW);
	   }
}



/*****************************************
           Discrete inputs
*****************************************/

#define FLAG_INVERTED  (1<<0)
#define FLAG_ENABLED   (1<<1)

enum {
  PUSH_BUTTON = 0,
  TOGGLE_SWITCH,
  ROTARY_SWITCH,
  ENCODER_A,
  ENCODER_B
};


/*********************************************
      Inputs setup macro definitions
**********************************************/
#define input_init(x,y,m,i)   do {\
bits[x].counter = 0;\
bits[x].signal = 0;\
bits[x].max = 10;\
bits[x].flag = (FLAG_ENABLED | FLAG_INVERTED);\
bits[x].pin = y;\
pinMode(y,INPUT_PULLUP);\
bits[x].mode = m;\
bits[x].id = i;\
} while(0)


#define input_init_fast(x,y,m,i)   do {\
bits[x].counter = 0;\
bits[x].signal = 0;\
bits[x].max = 4;\
bits[x].flag = (FLAG_ENABLED | FLAG_INVERTED);\
bits[x].pin = y;\
pinMode(y,INPUT_PULLUP);\
bits[x].mode = m;\
bits[x].id = i;\
} while(0)    
   
/*****************************************
           Inputs setup
*****************************************/
void bits_init(void)
{
  int i;
  memset(bits,0,sizeof(bits));
#if UNITS_NUMBER == 2
  input_init_fast(0,40,ENCODER_A,0x00); 
  input_init_fast(1,41,ENCODER_B,0x00);
  input_init(2,42,PUSH_BUTTON,0x80);     /* TUNE STEP BUTTON UNIT 1     */
 
  input_init_fast(3,46,ENCODER_A,0x00);
  input_init_fast(4,47,ENCODER_B,0x00);
  input_init(5,48,PUSH_BUTTON,0x81);     /* TUNE STEP BUTTON UNIT 2     */

  input_init(6,39,PUSH_BUTTON,0x82);     /* STDBY/ACT FREQ BUTTON UNIT 1*/ 
  input_init(7,5,PUSH_BUTTON,0x83);      /* STDBY/ACT FREQ BUTTON UNIT 2*/ 

  input_init(8,A0,ROTARY_SWITCH,0x88);  
  input_init(9,A1,ROTARY_SWITCH,0x89); 
  input_init(10,A2,ROTARY_SWITCH,0x8A);  
  input_init(11,A3,ROTARY_SWITCH,0x8B);  
  input_init(12,A4,ROTARY_SWITCH,0x8C); 
  input_init(13,A5,ROTARY_SWITCH,0x8D);  
  input_init(14,A6,ROTARY_SWITCH,0x8E); 
  input_init(15,A7,ROTARY_SWITCH,0x8F);  

  input_init(16,A8,ROTARY_SWITCH,0x90);  
  input_init(17,A9,ROTARY_SWITCH,0x91); 
  input_init(18,A10,ROTARY_SWITCH,0x92);  
  input_init(19,A11,ROTARY_SWITCH,0x93);  
  input_init(20,A12,ROTARY_SWITCH,0x94); 
  input_init(21,A13,ROTARY_SWITCH,0x95);  
  input_init(22,A14,ROTARY_SWITCH,0x96); 
  input_init(23,A15,ROTARY_SWITCH,0x97);  

  input_init(24,32,TOGGLE_SWITCH,0x98); /*   ADF1 SOUND TOGGLE SWITCH  UNIT 1    */
  input_init(25,33,TOGGLE_SWITCH,0x99); /*   ADF2 SOUND TOGGLE SWITCH  UNIT 1    */ 
  input_init(26,34,TOGGLE_SWITCH,0x9A); /*   NAV1 SOUND TOGGLE SWITCH  UNIT 1    */ 
  input_init(27,35,TOGGLE_SWITCH,0x9B); /*   NAV2 SOUND TOGGLE SWITCH  UNIT 1    */ 
  input_init(28,36,TOGGLE_SWITCH,0x9C); /*   MARKER SOUND TOGGLE SWITCH UNIT 1   */ 
  input_init(29,37,TOGGLE_SWITCH,0x9D); /*   DME SOUND TOGGLE SWITCH  UNIT 1     */
  input_init(30,38,TOGGLE_SWITCH,0x9E); /*   COM1_2_ACTIVE_TOGGLE SWITCH UNIT 1  */
  input_init(31,43,TOGGLE_SWITCH,0x9F); /*   XPNDR STBY/ACT TOGGLE SWITCH UNIT 1 */
  input_init(32,44,TOGGLE_SWITCH,0xA0); /*   IDENT BUTTON UNIT 1*/ 
                                            
  input_init(33,7,TOGGLE_SWITCH,0xA1);  /*   ADF1 SOUND TOGGLE SWITCH  UNIT 2    */
  input_init(34,9,TOGGLE_SWITCH,0xA2);  /*   ADF2 SOUND TOGGLE SWITCH  UNIT 2    */ 
  input_init(35,8,TOGGLE_SWITCH,0xA3);  /*   NAV1 SOUND TOGGLE SWITCH  UNIT 2    */ 
  input_init(36,11,TOGGLE_SWITCH,0xA4); /*   NAV2 SOUND TOGGLE SWITCH  UNIT 2    */ 
  input_init(37,12,TOGGLE_SWITCH,0xA5); /*   MARKER SOUND TOGGLE SWITCH UNIT 2   */ 
  input_init(38,50,TOGGLE_SWITCH,0xA6); /*   DME SOUND TOGGLE SWITCH  UNIT 2     */
  input_init(39,49,TOGGLE_SWITCH,0xA7); /*   COM1_2_ACTIVE_TOGGLE SWITCH UNIT 2  */
  input_init(40,10,TOGGLE_SWITCH,0xA8); /*   XPNDR STBY/ACT TOGGLE SWITCH UNIT 2 */
  input_init(41,6,TOGGLE_SWITCH,0xA9);  /*   IDENT BUTTON UNIT 2                 */ 
#endif

#if UNITS_NUMBER == 1
  input_init_fast(0,40,ENCODER_A,0x00); 
  input_init_fast(1,41,ENCODER_B,0x00);
  input_init(2,42,PUSH_BUTTON,0x80);     /* TUNE STEP BUTTON UNIT 1     */

  input_init(3,39,PUSH_BUTTON,0x82);     /* STDBY/ACT FREQ BUTTON UNIT 1*/ 

  input_init(4,A0,ROTARY_SWITCH,0x88);  
  input_init(5,A1,ROTARY_SWITCH,0x89); 
  input_init(6,A2,ROTARY_SWITCH,0x8A);  
  input_init(7,A3,ROTARY_SWITCH,0x8B);  
  input_init(8,A4,ROTARY_SWITCH,0x8C); 
  input_init(9,A5,ROTARY_SWITCH,0x8D);  
  input_init(10,A6,ROTARY_SWITCH,0x8E); 
  input_init(11,A7,ROTARY_SWITCH,0x8F);  

  input_init(12,32,TOGGLE_SWITCH,0x98); /*   ADF1 SOUND TOGGLE SWITCH  UNIT 1    */
  input_init(13,33,TOGGLE_SWITCH,0x99); /*   ADF2 SOUND TOGGLE SWITCH  UNIT 1    */ 
  input_init(14,34,TOGGLE_SWITCH,0x9A); /*   NAV1 SOUND TOGGLE SWITCH  UNIT 1    */ 
  input_init(15,35,TOGGLE_SWITCH,0x9B); /*   NAV2 SOUND TOGGLE SWITCH  UNIT 1    */ 
  input_init(16,36,TOGGLE_SWITCH,0x9C); /*   MARKER SOUND TOGGLE SWITCH UNIT 1   */ 
  input_init(17,37,TOGGLE_SWITCH,0x9D); /*   DME SOUND TOGGLE SWITCH  UNIT 1     */
  input_init(18,38,TOGGLE_SWITCH,0x9E); /*   COM1_2_ACTIVE_TOGGLE SWITCH UNIT 1  */
  input_init(19,43,TOGGLE_SWITCH,0x9F); /*   XPNDR STBY/ACT TOGGLE SWITCH UNIT 1 */
  input_init(20,44,TOGGLE_SWITCH,0xA0); /*   IDENT BUTTON UNIT 1*/ 

  input_init(21,46,TOGGLE_SWITCH,0xC0);
  input_init(22,47,TOGGLE_SWITCH,0xC1);
  input_init(23,48,TOGGLE_SWITCH,0xC2);     
  input_init(24,5,TOGGLE_SWITCH,0xC3);      
  input_init(25,A8,TOGGLE_SWITCH,0xC4);  
  input_init(26,A9,TOGGLE_SWITCH,0xC5); 
  input_init(27,A10,TOGGLE_SWITCH,0xC6);  
  input_init(28,A11,TOGGLE_SWITCH,0xC7);  
  input_init(29,A12,TOGGLE_SWITCH,0xC8); 
  input_init(30,A13,TOGGLE_SWITCH,0xC9);  
  input_init(31,A14,TOGGLE_SWITCH,0xCA); 
  input_init(32,A15,TOGGLE_SWITCH,0xCB);
  input_init(33,7,TOGGLE_SWITCH,0xCC);
  input_init(34,9,TOGGLE_SWITCH,0xCD);
  input_init(35,8,TOGGLE_SWITCH,0xCE);
  input_init(36,11,TOGGLE_SWITCH,0xCF);
  input_init(37,12,TOGGLE_SWITCH,0xD0);
  input_init(38,50,TOGGLE_SWITCH,0xD1);
  input_init(39,49,TOGGLE_SWITCH,0xD2);
  input_init(40,10,TOGGLE_SWITCH,0xD3);
  input_init(41,6,TOGGLE_SWITCH,0xD4);
  input_init(42,27,TOGGLE_SWITCH,0xD5);
  input_init(43,29,TOGGLE_SWITCH,0xD6);
  input_init(44,31,TOGGLE_SWITCH,0xD7);
  input_init(45,51,TOGGLE_SWITCH,0xD8);

#endif


  
}


/*********************************************
          Main discrete task
**********************************************/
void discrete_task( void)
{
    char unit1_mode_tmp = 0;
    char unit2_mode_tmp = 0;
    int bit,flag;
    int bit_state;
    int i,itmp;
    
    #if TIMING_TEST
        digitalWrite(TIMING_TEST_PIN, HIGH);  
    #endif  

    for(i=0;i<BITS_MAXIMUM;i++)
    {
       if(bits[i].flag & FLAG_ENABLED)
       {
    #if UNITS_NUMBER == 2    
           if(i>=8 && i <=15)
           {
              if(bits[i].signal)
                          unit1_mode_tmp = i-8;
           }

           if(i>=16 && i <=23)
           {
              if(bits[i].signal)
                          unit2_mode_tmp = i-16;
           }
    #endif 

    #if UNITS_NUMBER == 1    
           if(i>=4 && i <=11)
           {
              if(bits[i].signal)
                          unit1_mode_tmp = i-4;
           }

    #endif 

           
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
                           //front __0__/--1--
                           if(bits[i].mode == ENCODER_A)
                           {
                               Serial.write(STX);
                               Serial.write('2');
                               if(i==0)
                               {
                                   itmp = (unit1_mode << 4) + 
                                          (((unit1_step << 2) +
                                          (unit1_stb_act << 1)) & mask_of_mode_table[unit1_mode]) +
                                          bits[i+1].signal;
                               }  
                       #if UNITS_NUMBER == 2                
                               else
                               {
                                   itmp = (unit2_mode << 4) + 
                                          (((unit2_step << 2) +
                                          (unit2_stb_act << 1)) & mask_of_mode_table[unit2_mode]) +
                                          bits[i+1].signal;
                               }
                       #endif          
                               Serial.write(hex_table[itmp >> 4]);
                               Serial.write(hex_table[itmp & 0x0F]);
                               Serial.write(ETX);
                            }
                            else if(bits[i].mode == ROTARY_SWITCH)
                            {

                       #if UNITS_NUMBER == 1  
                                if(i>=4 && i <=11)
                                {
                                    unit1_step = 0;
                                    unit1_stb_act = 0;
                                }
                       #endif 

                       
                       
                       #if UNITS_NUMBER == 2  
                                if(i>=8 && i <=15)
                                {
                                    unit1_step = 0;
                                    unit1_stb_act = 0;
                                }

                                if(i>=16 && i <=23)
                                {
                                    unit2_step = 0;
                                    unit2_stb_act = 0;
                                }
                       #endif 
                                //Serial.write(STX);
                                //Serial.write('1');
                                //itmp = bits[i].id;
                                //Serial.write(hex_table[itmp >> 4]);
                                //Serial.write(hex_table[itmp & 0x0F]);
                                //Serial.write(ETX);
                            }
                            else if(bits[i].mode != ENCODER_B)
                            {
                                switch(i)
                                {
                                    case SIGNAL_UNIT_1_TUNE_STEP:
                                        unit1_step++;
                                        if(unit1_step >= step_of_mode_table[unit1_mode])
                                            unit1_step  = 0;
                                        itmp = bits[i].id;
                                    break;
                       #if UNITS_NUMBER == 2             
                                    case SIGNAL_UNIT_2_TUNE_STEP:
                                        unit2_step++;
                                        if(unit2_step >= step_of_mode_table[unit2_mode])
                                            unit2_step  = 0;                                       
                                        itmp = bits[i].id;
                                    break;
                       #endif            
                                    case SIGNAL_UNIT_1_STDBY_ACT_FREQ:
                                        unit1_stb_act ^= 1;                                          
                                        itmp = 0xB0 + unit1_mode;
                                        Serial.write(STX);
                                        Serial.write('8');
                                        Serial.write(hex_table[itmp >> 4]);
                                        Serial.write(hex_table[itmp & 0x0F]);
                                        Serial.write(ETX);                                        
                                    break;
                       #if UNITS_NUMBER == 2                                     
                                    case SIGNAL_UNIT_2_STDBY_ACT_FREQ:
                                        unit2_stb_act ^= 1;                                          
                                        itmp = 0xB0 + unit2_mode;
                                        Serial.write(STX);
                                        Serial.write('8');
                                        Serial.write(hex_table[itmp >> 4]);
                                        Serial.write(hex_table[itmp & 0x0F]);
                                        Serial.write(ETX);                                        
                                    break;
                       #endif             
                                    default:
                                        itmp = bits[i].id;
                                        Serial.write(STX);
                                        Serial.write('8');
                                        Serial.write(hex_table[itmp >> 4]);
                                        Serial.write(hex_table[itmp & 0x0F]);
                                        Serial.write(ETX);                                        
                                    
                                }
                                

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
                                switch(i)
                                {

                                    case SIGNAL_UNIT_1_TUNE_STEP:
                                        //unit1_step++;
                                        //if(unit1_step >= step_of_mode_table[unit1_mode])
                                        //    unit1_step  = 0;
                                        //itmp = bits[i].id;
                                    break;
                       #if UNITS_NUMBER == 2             
                                    case SIGNAL_UNIT_2_TUNE_STEP:
                                        //unit2_step++;
                                        //if(unit2_step >= step_of_mode_table[unit2_mode])
                                        //    unit2_step  = 0;                                       
                                        //itmp = bits[i].id;
                                    break;
                       #endif 
                                  
                                    case SIGNAL_UNIT_1_STDBY_ACT_FREQ:
                                        unit1_stb_act ^= 1;                                          
                                        itmp = 0xB0 + unit1_mode;
                                        Serial.write(STX);
                                        Serial.write('4');
                                        Serial.write(hex_table[itmp >> 4]);
                                        Serial.write(hex_table[itmp & 0x0F]);
                                        Serial.write(ETX);                                        
                                    break;
                       #if UNITS_NUMBER == 2                                     
                                    case SIGNAL_UNIT_2_STDBY_ACT_FREQ:
                                        unit2_stb_act ^= 1;                                          
                                        itmp = 0xB0 + unit2_mode;
                                        Serial.write(STX);
                                        Serial.write('4');
                                        Serial.write(hex_table[itmp >> 4]);
                                        Serial.write(hex_table[itmp & 0x0F]);
                                        Serial.write(ETX);                                        
                                    break;
                       #endif             
                                    default:
                                        itmp = bits[i].id;
                                        Serial.write(STX);
                                        Serial.write('4');
                                        Serial.write(hex_table[itmp >> 4]);
                                        Serial.write(hex_table[itmp & 0x0F]);
                                        Serial.write(ETX);                                        
                                    
                                }                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                //Serial.write(STX);
                                //Serial.write('4');
                                //itmp = bits[i].id;
                                //Serial.write(hex_table[itmp >> 4]);
                                //Serial.write(hex_table[itmp & 0x0F]);
                                //Serial.write(ETX);
                            }
                        }
                        bits[i].signal = 0;
                    }
                }
            }
        }
    }
    #if TIMING_TEST    
        digitalWrite(TIMING_TEST_PIN, LOW);  
    #endif
    unit1_mode = unit1_mode_tmp;
    unit2_mode = unit2_mode_tmp;
    
}







