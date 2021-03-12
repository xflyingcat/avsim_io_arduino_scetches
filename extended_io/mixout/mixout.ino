//----------------------------------------------------------
//     MixOut:  Discrete output and symbolic/numeric output
//----------------------------------------------------------
#define BUS_ADDRESS 3

#define INTENSITY_LEVEL  2


#define MAX7219_MAX              6

const unsigned char intensity_table[MAX7219_MAX] = 
{
 2,   2, 2, 2, /* 7SEGM DIGITS*/
 15, 15        /* LEDS */
};

const unsigned char decimal_points[6][8] = {
/* put 0x80 instead of 0x00 to show the decimal point at position you want */
  /* 0 */ { 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 },
  /* 1 */ { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  /* 2 */ { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  /* 3 */ { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  /* 4 */ { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  /* 5 */ { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

const unsigned char seven_segm_decoder[] = {
  0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
  0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
/*       !     "     #     $     %     &     '     (     )     *     +     ,     -     .     /    */
  0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x08, 0x01, 0x08, 0x08,
/*  0     1     2     3     4     5     6     7     8     9     :     ;     <     =     >     ?   */
  0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
/*  @     A     B     C     D     E     F     G     H     I     J     K     L     M     N     O   */
  0x08, 0x77, 0x1F, 0x4E, 0x3D, 0x4F, 0x47, 0x7B, 0x37, 0x06, 0x3C, 0x37, 0x0E, 0x54, 0x15, 0x7E,
/* P     Q     R     S     T     U     V     W     X     Y     Z     [     \     ]     ^     _    */
  0x67, 0x73, 0x05, 0x5B, 0x0F, 0x3E, 0x1C, 0x2A, 0x37, 0x3B, 0x6D, 0x08, 0x08, 0x08, 0x08, 0x08,
/* `     a     b     c     d     e     f     g     h     i     j     k     l     m     n     o    */
  0x08, 0x77, 0x1F, 0x4E, 0x3D, 0x4F, 0x47, 0x7B, 0x37, 0x06, 0x3C, 0x37, 0x0E, 0x54, 0x15, 0x1d,
/* p     q     r     s     t     u     v     w     x     y     z     {     |     }     ~         */
  0x67, 0x73, 0x05, 0x5B, 0x0F, 0x3E, 0x1C, 0x2A, 0x37, 0x3B, 0x6D, 0x08, 0x08, 0x08, 0x08, 0x08,
};

// Stop! Keep away! Do not edit below it at all!

#define USING_SPI

#ifdef USING_SPI
   #include <SPI.h>
#endif

#define TXDEN  2

void setup(void);
void loop(void);
void serialEvent();
int parser(char in);
void send_response(unsigned char response_code);
void max7219_setup(void);
void refresh_all(void);

extern char seven_segments_leds[6][8];
extern char do_bytes[2][8];

ISR (USART_TX_vect)
{
   digitalWrite(TXDEN, LOW);
}

void setup() 
{

    /* https://horiokanta.com/en/arduino-and-rs485/ */
    UCSR0B |= (1<<TXCIE0);     // TX Complete Interrupt Enable 0
    pinMode(TXDEN, OUTPUT);
    pinMode(A0, OUTPUT);
#ifdef USING_SPI
    SPI.setBitOrder(MSBFIRST);
    SPI.begin();
#endif
    
    max7219_setup();
   
    Serial.begin(115200);
   
}  


unsigned char input_data[16];

void loop() 
{
  refresh_all();
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

            }

            if(buf[0] == BUS_ADDRESS && buf[1] == DO_REQUEST)
            {
                digitalWrite(A0, 1);
                send_response(DO_REQUEST);
                memcpy(do_bytes,&buf[3],sizeof(do_bytes));
                digitalWrite(A0, 0);

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


/*****************************************
           MAX7219 routines
*****************************************/
#define MAX7219_REG_DECODE_MODE  0x09
#define MAX7219_REG_INTENSITY    0x0A
#define MAX7219_REG_SCANLIMIT    0x0B
#define MAX7219_REG_SHUTDOWN     0x0C
#define MAX7219_REG_TEST         0x0F
#define MAX7219_DATAIN           11
#define MAX7219_CLOCK            13
#define MAX7219_LOAD             10



char seven_segments_leds[6][8] = {
  /* indicators lines from simulator */
  {'0','0','0','0','0','0','0','0'},
  {'1','1','1','1','1','1','1','1'},
  {'2','2','2','2','2','2','2','2'},
  {'3','3','3','3','3','3','3','3'},
  {'4','4','4','4','4','4','4','4'},
  {'5','5','5','5','5','5','5','5'}
};


char do_bytes[2][8] = {
  { 1,  2,  3,  4, 5,  6,  7,  8 },
  { 9, 10,  11, 12,13, 14, 15, 16}
};


void max7219_write_byte(byte bt) 
{
#ifdef USING_SPI
    SPI.transfer(bt);
#else
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
#endif    
}

void max7219_reg_init (byte reg, byte val, int rstart, int rend) 
{
    int i;
    digitalWrite(MAX7219_LOAD, LOW);
    for (i=rstart; i < rend; i++) 
    {
        max7219_write_byte(reg);
        max7219_write_byte(val);
    }
    digitalWrite(MAX7219_LOAD, LOW);
    digitalWrite(MAX7219_LOAD,HIGH);
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

void refresh_all(void) 
{    
    int i,j;
    unsigned char bt, dp;
    char *p;
    for(i=0;i<8;i++)
    {
        digitalWrite(MAX7219_LOAD, LOW);    
        for ( j = 0; j < 4; j++) 
        {
           max7219_write_byte(i+1);  
           bt = seven_segments_leds[j][i];
           dp = decimal_points[j][i];   
           bt = seven_segm_decoder[bt & 0x7F];
           max7219_write_byte(bt | dp);
        }
        for (j=4 ; j < MAX7219_MAX; j++) 
        {
           max7219_write_byte(i+1);
           max7219_write_byte(do_bytes[j-4][i]);   
        }
        digitalWrite(MAX7219_LOAD, LOW);
        digitalWrite(MAX7219_LOAD,HIGH);
    }
     

}

void max7219_setup(void)
{
int i; 

#ifdef USING_SPI
#else
    pinMode(MAX7219_DATAIN, OUTPUT);
    pinMode(MAX7219_CLOCK,  OUTPUT);
#endif
    pinMode(MAX7219_LOAD,   OUTPUT);

    max7219_reg_init(MAX7219_REG_SCANLIMIT,   0x07);
    max7219_reg_init(MAX7219_REG_DECODE_MODE, 0x00);
    max7219_reg_init(MAX7219_REG_SHUTDOWN,    0x01);
    max7219_reg_init(MAX7219_REG_TEST,        0x00);
    for(i=0; i<8; i++) 
    {    
        max7219_reg_init(i+1,0);
    }

    max7219_set_intensity();
} 

