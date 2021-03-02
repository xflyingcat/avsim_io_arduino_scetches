//---------------------------------------------------------------------
//       Master for extended I/O downlink network
//---------------------------------------------------------------------
#define DEVICE_ID   2   /* Uplink device ID in the plugin network */
//---------------------------------------------------------------------
#define UPLINK_STX  '{'
#define UPLINK_ETX  '}'
#define UPLINK_ACK  '!'

#define STX 0x02
#define ETX 0x03
#define DLE 0x10
#define DLE_STUFFING_BYTE 0x40
#define PARSER_BUF_LEN 32

#define DISCRETE_OUTPUT_BOARD_MEMORY_SIZE  16
#define DISCRETE_INPUT_BOARD_MEMORY_SIZE   16
#define NUMERIC_OUTPUT_BOARD_MEMORY_SIZE   48

#define DISCRETE_INPUT   0
#define DISCRETE_OUTPUT  1
#define NUMERIC_OUTPUT   2

#define DISCRETE_INPUT_REQUEST      0x40
#define DISCRETE_OUTPUT_REQUEST     0x41
#define NUMERIC_OUTPUT_REQUEST      0x42

typedef struct
{
    unsigned char addr;
    unsigned char msize;
    unsigned char *mem;
    unsigned char type;
} SLAVE_STRUCT;

////////////////////////////////////////////////////////////////////////
///////////      User responsibility area      /////////////////////////
///////////    Downlink network configuration  /////////////////////////
////////////////////////////////////////////////////////////////////////
unsigned char mem1[DISCRETE_INPUT_BOARD_MEMORY_SIZE];
unsigned char mem2[DISCRETE_INPUT_BOARD_MEMORY_SIZE];
unsigned char mem3[NUMERIC_OUTPUT_BOARD_MEMORY_SIZE];
unsigned char mem4[NUMERIC_OUTPUT_BOARD_MEMORY_SIZE];
unsigned char mem5[DISCRETE_OUTPUT_BOARD_MEMORY_SIZE];

#define REQUEST_MAX   3

SLAVE_STRUCT request_table[REQUEST_MAX] = {
  { 1, DISCRETE_INPUT_BOARD_MEMORY_SIZE,  mem1,  DISCRETE_INPUT},
//  { 2, DISCRETE_INPUT_BOARD_MEMORY_SIZE,  mem2,  DISCRETE_INPUT},
  { 3, NUMERIC_OUTPUT_BOARD_MEMORY_SIZE,  mem3,  NUMERIC_OUTPUT},
//  { 4, NUMERIC_OUTPUT_BOARD_MEMORY_SIZE,  mem4,  NUMERIC_OUTPUT},
  { 3, DISCRETE_OUTPUT_BOARD_MEMORY_SIZE,  mem5,  DISCRETE_OUTPUT},

};   
/////////////////////////////////////////////////////////////////////////
///////////////  End of user responsibility area  ///////////////////////
/////////////////////////////////////////////////////////////////////////

const char hex_table[] = "0123456789ABCDEF";

#define TXDEN  20
#define SIMPLE_FIFO_LENGTH 10
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
int parser(char in);
void send_data(void);
void bits_init(void);
void send_event(unsigned short event_type, unsigned short event_id);
int recv_event(unsigned short *ev);


ISR (USART1_TX_vect)
{
   digitalWrite(TXDEN, LOW);
}

void setup() 
{
    /* https://horiokanta.com/en/arduino-and-rs485/ */
    UCSR1B |= (1<<TXCIE1);     // TX Complete Interrupt Enable 0
    pinMode(TXDEN, OUTPUT);
    Serial.begin(115200);
    Serial1.begin(115200);
      
}  

char response_flag = 0;
char slave_index = 0;

void hexdump(void *ptr, int size)
{
    int i;
    char buf[10];
    for(i=0;i<size;i++)
    {
      if(!(i % 16))
          Serial.println("");
      sprintf(buf, "%02X ", ((unsigned char*)ptr)[i]);
      Serial.print(buf);
    }
    if(size)
       Serial.println(""); 
}

void loop() 
{
    poll_slaves();
} 

#define RBUF_LEN (4*NUMERIC_OUTPUT_BOARD_MEMORY_SIZE+5)

char rbuf[RBUF_LEN];
int rcnt;

void serialEvent() 
{
    int ndx,itmp,i,j,k;
    char *p;  
    unsigned char bt;
    while (Serial.available()) 
    {
        char ch = (char)Serial.read();
        switch(ch)
        {
            case UPLINK_STX:
                rcnt = 0;
            break;

            case UPLINK_ETX:
                rbuf[rcnt] = 0x00;
        
                if(rbuf[0] == '7')
                {
                   p = &rbuf[3];
                   for(i = 0; i < REQUEST_MAX; i++)
                   {
                      if(request_table[i].type == NUMERIC_OUTPUT)
                      {
                        memcpy(request_table[i].mem,p,request_table[i].msize);
                        p += request_table[i].msize;
                      }
                   }
                } 
                    
                if(rbuf[0] == 'L')
                {   
                   p = &rbuf[3];
                   for(i = 0; i < REQUEST_MAX; i++)
                   {
                      if(request_table[i].type == DISCRETE_OUTPUT)
                      {
                         for(k=0;k<request_table[i].msize;k++)
                         {  
                          for(bt =0,j=0;j<8;j++)
                            bt |= ((*p++ & 1) << j);
                          request_table[i].mem[k] = bt;
                         } 
                         //p += request_table[i].msize;
                      }
                   }
                }
                
                if(rbuf[0] == 'D')
                {
                   Serial.write(UPLINK_ACK);
                   Serial.write(UPLINK_STX);
                   Serial.write('F');
                   itmp = DEVICE_ID;
                   Serial.write(hex_table[itmp >> 4]);
                   Serial.write(hex_table[itmp & 0x0F]);
                   Serial.write(UPLINK_ETX);
                } 
                else
                    Serial.write(UPLINK_ACK);

            break;

            default:
                if(rcnt<RBUF_LEN)
                    rbuf[rcnt++] = ch;
        }
    }
}  

void serialEvent1() 
{
    while (Serial1.available()) 
    {
       parser((unsigned char)Serial1.read());
    }
}



int parser(char in)
{
    static char dle_flag = 0;
    static char ndx;
    static char buf[PARSER_BUF_LEN];
    static unsigned char lrc;
    char len, addr;
    unsigned char *p,u8tmp;
    switch(in)
    {
    case STX:
        ndx = 0;
        lrc = 0;
        break;
    case ETX:
        //Serial.print("Got ETX current slave addr: ");
        //Serial.println(request_table[slave_index].addr,DEC);
        //hexdump(buf, ndx);
        if((lrc == 0) && (ndx < PARSER_BUF_LEN) && buf[0] == request_table[slave_index].addr)
        {
            switch(buf[1])
            {
               
                case DISCRETE_INPUT_REQUEST:
                     len = buf[2];
                     addr = buf[0];
                     p = &buf[3];
                     while(len > 0)
                     {
                        Serial.write('{'); 
                        u8tmp = *p++ >> 4;
                        if(u8tmp == 3)
                             u8tmp = 1;
                        Serial.write(hex_table[u8tmp]); 
                        Serial.write(hex_table[addr]); 
                        Serial.write(hex_table[(*p >> 4)]); 
                        Serial.write(hex_table[(*p++ & 0x0F)]); 
                        Serial.write('}'); 
                        len -= 2;
                     }
                     response_flag = 1;
                break;

                case DISCRETE_OUTPUT_REQUEST:
                     response_flag = 1;
                break;

                case NUMERIC_OUTPUT_REQUEST:
                     response_flag = 1;
                break;
              
            }
            
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



void send_data(void)
{
#if 0  
    // stx(1) adr(<=2) tag(1) len(<=2)   data(SIMPLE_FIFO_LENGTH*4)   lrc(<=2) etx(1) + spare(8)
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
#endif    
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
    fifo_push(&ev_fifo, event_id);
}

int recv_event(unsigned short *ev)
{
   return fifo_pop(&ev_fifo, ev);
}


void poll_slaves(void)
{
    static unsigned char tx_buffer[100]; 
    unsigned char *txp,lrc;
    static unsigned long ref;
    int j;
    
    if(!response_flag && (millis() - ref) < 1000 )
       return;

           if(++slave_index == REQUEST_MAX)
               slave_index = 0;
       
            
            switch(request_table[slave_index].type)
            {
            case DISCRETE_INPUT:

                txp = tx_buffer;
                lrc = 0;
                *txp++ = STX;
                txp += add_byte(txp, &lrc, request_table[slave_index].addr);
                txp += add_byte(txp, &lrc, DISCRETE_INPUT_REQUEST);
                txp += add_byte(txp, &lrc, 0);
                txp += add_byte(txp, NULL, (lrc ^ 0xFF)+1);
                *txp++ = ETX;
                break;
            case DISCRETE_OUTPUT:

                txp = tx_buffer;
                lrc = 0;
                *txp++ = STX;
                txp += add_byte(txp, &lrc, request_table[slave_index].addr);
                txp += add_byte(txp, &lrc, DISCRETE_OUTPUT_REQUEST);
                txp += add_byte(txp, &lrc, DISCRETE_INPUT_BOARD_MEMORY_SIZE);
                for(j=0; j<DISCRETE_OUTPUT_BOARD_MEMORY_SIZE; j++)
                {
                    txp += add_byte(txp, &lrc, request_table[slave_index].mem[j]);
                }
                txp += add_byte(txp, NULL, (lrc ^ 0xFF)+1);
                *txp++ = ETX;
                break;

            case NUMERIC_OUTPUT:

                txp = tx_buffer;
                lrc = 0;
                *txp++ = STX;
                txp += add_byte(txp, &lrc, request_table[slave_index].addr);
                txp += add_byte(txp, &lrc, NUMERIC_OUTPUT_REQUEST);
                txp += add_byte(txp, &lrc, NUMERIC_OUTPUT_BOARD_MEMORY_SIZE);
                for(j=0; j<NUMERIC_OUTPUT_BOARD_MEMORY_SIZE; j++)
                {
                    txp += add_byte(txp, &lrc, request_table[slave_index].mem[j]);
                }
                txp += add_byte(txp, NULL, (lrc ^ 0xFF)+1);

                *txp++ = ETX;
                break;
            }

            int len = txp - tx_buffer;
            digitalWrite(TXDEN, HIGH);
            Serial1.write(tx_buffer, len);


            response_flag  = 0; 
            ref = millis();   
}


