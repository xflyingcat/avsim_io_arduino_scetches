//----------------------------------------------
//         Discrete input
//----------------------------------------------
#define BUS_ADDRESS 1

#define BITS_MAXIMUM 128
#define KEY_PRESSING_CRITICAL_TIME 1000
#define TXDEN  2
#define SIMPLE_FIFO_LENGTH 20
typedef struct
{
  int   push_indx;
  int   pop_indx;
  unsigned short buff[SIMPLE_FIFO_LENGTH];
} SIMPLE_FIFO;

SIMPLE_FIFO ev_fifo;

void setup(void);
void loop(void);
void serialEvent();
unsigned short upd_crc_16(unsigned char cp, unsigned short crc);
int parser(char in);
void send_data(void);
void bits_init(void);
void discrete_task( void);
void send_event(unsigned short event_type, unsigned short event_id);
int recv_event(unsigned short *ev);


ISR (USART_TX_vect)
{
   digitalWrite(TXDEN, LOW);
}

void setup() 
{
    /* https://horiokanta.com/en/arduino-and-rs485/ */
    UCSR0B |= (1<<TXCIE0);     // TX Complete Interrupt Enable 0
    pinMode(TXDEN, OUTPUT);
    
    pinMode(5,INPUT_PULLUP);
    pinMode(6,INPUT_PULLUP);
    pinMode(7,INPUT_PULLUP);
    pinMode(8,INPUT_PULLUP);
    pinMode(9,INPUT_PULLUP);
    pinMode(10,INPUT_PULLUP);
    pinMode(11,INPUT_PULLUP);
    pinMode(12,INPUT_PULLUP);

    pinMode(A0, OUTPUT); 
    pinMode(A1, OUTPUT); 
    pinMode(A2, OUTPUT); 
    pinMode(A3, OUTPUT); 
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);
    digitalWrite(A2, LOW);
    digitalWrite(A3, LOW);

    bits_init();
    Serial.begin(115200);
  
}  

unsigned char input_data[16];

unsigned char get_bit(unsigned char ndx)
{
  return ((input_data[ ndx & 0xF ]) & (1<<(ndx>>4))) && 1;
}

void loop() 
{
  static unsigned char addr = 0;
  unsigned char pb = PINB;
  unsigned char pd = PIND;
  static unsigned long ref = 0;
  pd = (pd >> 5) & 0x07;
  pb = (pb << 3) & 0xF8;
  pb = pb | pd;
  input_data[addr] = pb;

  addr++;
  addr &= 0x0F;
  unsigned char tmp =   1 * ((addr & 2)&&1) +
                        2 * ((addr & 1)&&1) +
                        4 * ((addr & 8)&&1) +
                        8 * ((addr & 4)&&1);

  pd = PINC;
  pd &= 0xF0;
  pd |= tmp;
  PORTC = pd;

  discrete_task();
  
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
#define PARSER_BUF_LEN 32

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
        ndx = 0;
        lrc = 0;
        break;
    case ETX:
        if((lrc == 0) && (ndx < PARSER_BUF_LEN))
        {
            if(buf[0] == BUS_ADDRESS && buf[1] == DI_REQUEST)
                send_data();
            
            return 1;
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

// stx(1) adr(<=2) tag(1) len(<=2)   data(SIMPLE_FIFO_LENGTH*4)   lrc(<=2) etx(1) + spare(8)

void send_data(void)
{
    static unsigned char tx_buffer[1+2+1+2+SIMPLE_FIFO_LENGTH*4+2+1+8];
    unsigned char *txp = tx_buffer;
    unsigned char lrc;
    unsigned char *pdata = &tx_buffer[sizeof(tx_buffer) - SIMPLE_FIFO_LENGTH*2-1];
    char i;
    unsigned short ev;
    for(i=0;i<SIMPLE_FIFO_LENGTH && recv_event(&ev);i++)
    {
      pdata[i*2] = ev >> 8;
      pdata[i*2+1] = ev & 0xFF;
    }
    int data_size = i*2;
    
    lrc = 0;
    *txp++ = STX;
    txp += add_byte(txp, &lrc, BUS_ADDRESS);
    txp += add_byte(txp, &lrc, DI_REQUEST);
    txp += add_byte(txp, &lrc, data_size);

    for(i=0;i<data_size;i++)
    {
      txp += add_byte(txp, &lrc, pdata[i]);
    }

    txp += add_byte(txp, NULL, ((lrc ^ 0xFF) + 1));
    *txp++ = ETX;
    
    digitalWrite(TXDEN, HIGH);
    int len = txp-tx_buffer;
    Serial.write(tx_buffer,len);
}


unsigned short fifo_push(SIMPLE_FIFO *fifo, unsigned short data)
{
  int indx;
  indx = fifo->push_indx + 1;
  indx = indx % SIMPLE_FIFO_LENGTH;

  if (indx == fifo->pop_indx)
    return 0; // fifo is full
  fifo->buff[indx] = data;
  fifo->push_indx = indx;
  return 1;
}

unsigned short fifo_pop(SIMPLE_FIFO *fifo, unsigned short *data)
{
  int indx;
  if (fifo->push_indx == fifo->pop_indx)
    return 0; // fifo is empty
  indx = fifo->pop_indx + 1;
  indx = indx % SIMPLE_FIFO_LENGTH;
  *data = fifo->buff[indx];
  fifo->pop_indx = indx;
  return 1;
}

void send_event(unsigned short event_type, unsigned short event_id)
{
    event_id |= event_type;
    noInterrupts();
    fifo_push(&ev_fifo, event_id);
    interrupts();
}

int recv_event(unsigned short *ev)
{
   int result;
   noInterrupts();
   result = fifo_pop(&ev_fifo, ev);
   interrupts();
   return result;
}



/*****************************************
           Discrete inputs
*****************************************/
#define FLAG_INVERTED         (1<<0)
#define FLAG_ENABLED          (1<<1)
#define FLAG_HALF_CYCLE_EC    (1<<2)
#define USING_LONG_PRESSING   (1<<3)

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
short critical_timer;
char flag;
char mode;
short id;
} BITS_DESCRIPTION;

BITS_DESCRIPTION  bits[BITS_MAXIMUM];




/*********************************************
          Main discrete task
**********************************************/
void discrete_task( void)
{
    int bit,flag;
    int bit_state;
    int i,itmp;

    for(i=0;i<BITS_MAXIMUM;i++)
    {

       if(bits[i].flag & FLAG_ENABLED)
       {

           if(bits[i].critical_timer)
           {
               if(--bits[i].critical_timer == 0)
               {
                   send_event(0x3000, bits[i].id + 128);
               }
           }

           bit_state = get_bit(i);
               
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
                           
                           if(flag & USING_LONG_PRESSING)
                              bits[i].critical_timer = KEY_PRESSING_CRITICAL_TIME;  
                           
                           if(bits[i].mode == ENCODER_A)
                           {
                               send_event(0x2000, bits[i].id + bits[i+1].signal);
                            }
                            else if(bits[i].mode == ROTARY_SWITCH)
                            {
                                send_event(0x3000, bits[i].id);
                            }
                            else if(bits[i].mode != ENCODER_B && bits[i].mode != PUSH_BUTTON_X)
                            {
                                //unsigned short qq = bits[i].id;
                                //               qq &= 0xFF;
                                //               qq |= 0x8000;
                                //Serial.print(qq,HEX);
                                //Serial.print(" "); 
                                send_event(0x8000, bits[i].id);
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

                           if(bits[i].flag & FLAG_HALF_CYCLE_EC)
                           {
                               if(bits[i].mode == ENCODER_B)
                               {
                                   send_event(0x2000, bits[i].id + bits[i-1].signal);
                               }
                           } 

                           //if( bits[i].mode == ROTARY_SWITCH)
                           //{
                           //    if(bits[i].id == 1)
                           //    {
                           //       send_event(0x1000, bits[i].id + 128);
                           //    }
                           //}
                           //else
                           if( bits[i].mode == TOGGLE_SWITCH || 
                                bits[i].mode == PUSH_BUTTON )
                            {
                                send_event(0x4000, bits[i].id);
                            } 
                            else if(bits[i].mode == PUSH_BUTTON_X && bits[i].critical_timer)
                            {
                                send_event(0x4000, bits[i].id);
                            }
                            bits[i].critical_timer = 0;
                        }   
                        bits[i].signal = 0;
                    }
                }
            }
        }
    }
}



/*********************************************
      Regular inputs setup macro definitions
**********************************************/
    
#define input_init(x,m)   do {\
bits[x-1].counter = 0;\
bits[x-1].signal = 0;\
bits[x-1].max = 30;\
bits[x-1].flag = (FLAG_ENABLED | FLAG_INVERTED);\
bits[x-1].mode = m;\
bits[x-1].id = x;\
} while(0)


#define input_init_rotary_switch(x,m)   do {\
bits[x-1].counter = 0;\
bits[x-1].signal = 0;\
bits[x-1].max = 30;\
bits[x-1].flag = (FLAG_ENABLED | FLAG_INVERTED);\
bits[x-1].mode = m;\
bits[x-1].id = x;\
} while(0)

#define input_init_fast(x,m)   do {\
bits[x-1].counter = 0;\
bits[x-1].signal = 0;\
bits[x-1].max = 1;\
bits[x-1].flag = (FLAG_ENABLED | FLAG_INVERTED );\
bits[x-1].mode = m;\
bits[x-1].id = x;\
} while(0)

#define input_init_fast_half_cycle_ec(x,m)   do {\
bits[x-1].counter = 0;\
bits[x-1].signal = 0;\
bits[x-1].max = 1;\
bits[x-1].flag = (FLAG_ENABLED | FLAG_INVERTED | FLAG_HALF_CYCLE_EC);\
bits[x-1].mode = m;\
bits[x-1].id = x;\
} while(0)

   
/*****************************************
           Regular inputs setup
*****************************************/
void bits_init(void)
{  
    int i = 0;
    memset(bits,0,sizeof(bits));

    /* Default settings as a SPST switch */
    for(i=1;i<=BITS_MAXIMUM; i++)
    {
       input_init(i,ROTARY_SWITCH);
    }   

    /* user settings */
       input_init_fast(21,ENCODER_A);
       input_init_fast(22,ENCODER_B);

    /* half cycle encoder */
      input_init_fast_half_cycle_ec(40,ENCODER_A); 
      input_init_fast_half_cycle_ec(41,ENCODER_A);

    /* toggle switch */   
      input_init(110,TOGGLE_SWITCH);


} 





