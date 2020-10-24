/* Program for power factor correction us ing FIS logic,
 * using successive approximation algorithm to switching capacitor,
 * using IoT function for monitoring Apparent Power and Power factor
 * Sensor used is voltage sensor and current sensor to find RMS voltage and RMS current
 * 
 * this skecth does fuzzy algorithm using fix point method
 * 
 * coded by : alifnl
 * date started  : 01 may 2018
 * date finished : still progress
 * by using Fix point method, fuzzy algorithm was executed in about 53-52 us
 * function working properly : fuzzy function, succesive approximation algorithm, read_sensor, interrupt 0.1 ms, ethernet function, FIR, exponential filter
 */

// ENC28J60 -  STM32F103
//   VCC    -    3.3V
//   GND    -    GND
//   SCK    -    Pin PA5
//   SO     -    Pin PA6
//   SI     -    Pin PA7
//   CS     -    Pin PA4

#include <EtherCard_STM.h>
#include <SPI.h>

//////////////////////////////// IoT Initialization/////////////////
// ethernet interface mac address
static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x32 };
// remote website name
const char website[] PROGMEM = "api.thingspeak.com";
#define APIKEY "WXKETM3KMBU6OYCH"
Stash stash;
static byte session;
byte Ethernet::buffer[700];
///////////////////////////////end////////////////////////////////

///////////******* Fuzzy variable initialization********////////////

float input_Q =0; // input
float input_V = 0;

float MD_Q_mf1, MD_Q_mf2, MD_Q_mf3; // membership degree input Q (VAR)

float MD_V_mf1, MD_V_mf2, MD_V_mf3; // membership degree input V (voltage)         
         
float f1 ,f2, f3, f4, f5, f6,f7,f8,f9; // linear function value
float w1, w2, w3, w4, w5, w6, w7, w8, w9;

int C; // fuzzy output


/////// measurement inizialiation///////
// using DT sense current sensor
// using zmpt101B voltage sensor

int32_t voltage_magnitude[3] = {0}; //int32_t = long dik
int32_t current_magnitude[3] = {0};

int32_t real_power[3] = {0};
int32_t reactive_power[3] = {0};


int16_t voltage_delay[51]  =  {0};
int16_t current_delay[109] =  {0};

int32_t v_alpha= 0; // real
int32_t v_beta = 0; // delayed T/4

int32_t i_alpha = 0; //real
int32_t i_beta  = 0; // delayed T/4

int16_t adc_voltage_value = 0;  // sensor adc value
int16_t adc_current_value = 0; //sensor adc value

float kwh;
float cospi;


///*******************CAPACITOR VARIABLE DECLARATION......///////////////////////
int capacitor[8] = {50,30,20,10,5,2,1,1};
int c_state[8]   = { 0, 0, 0, 0,0,0,0,0};
//int max_cap = capacitor[0] + capacitor[1] + capacitor[2] + capacitor[3] + capacitor[4] + capacitor [5] + capacitor[6] + capacitor[7];
///////////////////////////////////////END//////////////////////////////////////

///////////////////////////////Input - Output Initialization/////////////////////////

const int voltage_sensor = PB1;

const int current_sensor = PB0;

const int C_output1 = PA11;
const int C_output2 = PA10;
const int C_output3 = PA9;
const int C_output4 = PA8;
const int C_output5 = PB15;
const int C_output6 = PB14;
const int C_output7 = PB13;
const int C_output8 = PB12;
const int control = PB11;


/////////////////////////////////////////////////////////////////////////////////////

uint32_t tick ; //uint32_t = unsigned 32 bit
uint32_t tick1;
uint32_t tick2;
uint32_t tick3;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
   //timer 2 setup
  //TIMER2_BASE->CR1 = 0x0000; //Tdts=Tint, upcounter, timer disabled
  //TIMER2_BASE->CR2 = 0x0000; //already set to 0 by default
  TIMER1_BASE->PSC = 7200; //72MHz/7200= 10,000 Hz 
  TIMER1_BASE->ARR = 20000; //20,000/10,000 = 2s
  TIMER1_BASE->CNT = 0;  //clear counter 
  timer_attach_interrupt(TIMER1, 0, handler_tim1); //interrupt on timer update
  TIMER1_BASE->CR1 |= 0x0001; //enable timer.
 // finish timer 2 setup // timer 2 for read sensor

  TIMER2_BASE->PSC = 3600; //72MHz/72000= 1,000 Hz 
  TIMER2_BASE->ARR = 1; //1000/1,000 = 0.1 ms
  TIMER2_BASE->CNT = 0;  //clear counter 
  timer_attach_interrupt(TIMER2, 0, handler_tim2); //interrupt on timer update
 Serial2.begin(1000000);

   // ADC setup//
  pinMode(voltage_sensor, INPUT_ANALOG);
  pinMode(current_sensor, INPUT_ANALOG);
  //output setup//
  pinMode(C_output1, OUTPUT);
  pinMode(C_output2, OUTPUT);
  pinMode(C_output3, OUTPUT);
  pinMode(C_output4, OUTPUT);
  pinMode(C_output5, OUTPUT);
  pinMode(C_output6, OUTPUT);
  pinMode(C_output7, OUTPUT);
  pinMode(C_output8, OUTPUT);
  pinMode(control,INPUT);

  
    
  digitalWrite(C_output1,1);
  digitalWrite(C_output2,1);
  digitalWrite(C_output3,1); 
  digitalWrite(C_output4,1);
  digitalWrite(C_output5,1);
  digitalWrite(C_output6,1);
  digitalWrite(C_output7,1);
  digitalWrite(C_output8,1);

  initialize_ethernet();
  Serial2.println("setup done");
  
}



// the loop function runs over and over again forever
void loop() {
    
 if(millis() - tick >= 1){
    ether.packetLoop(ether.packetReceive());
    tick = millis();
 }

  
  if (millis() - tick1 >= 16000){
    send_data();
    Serial2.println("data sent");
    tick1 = millis();
  }

  if (millis() - tick2 >= 1000){
    kwh += (real_power[2] *(float)2.11 / 1438)/(float)3600;
    tick2 = millis();
  }

  if (millis() - tick3 >= 100){
    //Serial2.println((sqrt(current_magnitude[2])* (float)2.11 / 1438)*1000 ); //fix
    tick3  = millis();
  }

}
void handler_tim2(){ // 0.1 ms
  read_sensor();
  expSmoothing();
}
void handler_tim1(){ // 2s
    Serial2.print("V :");
    Serial2.println(sqrt(voltage_magnitude[2]));
    Serial2.print("I :");
    Serial2.println(sqrt(current_magnitude[2])* (float)2.11 / 1438 ); //fix
    Serial2.print("P :");
    Serial2.println(real_power[2] *(float)2.11 / 1438);
    Serial2.print("Q :");
    Serial2.println(reactive_power[2] *(float)2.11 / 1438);
    Serial2.print("S :");
    Serial2.println(sqrt(voltage_magnitude[2]) * sqrt(current_magnitude[2]) * (float)0.1/71);
    Serial2.print("\n");
    
    if (digitalRead(control) == 1){    
      SAL();
    }else{
      reset_cap();
    }
    digitalWrite(C_output1, !c_state[0]);
    digitalWrite(C_output2, !c_state[1]);
    digitalWrite(C_output3, !c_state[2]); 
    digitalWrite(C_output4, !c_state[3]);
    digitalWrite(C_output5, !c_state[4]);
    digitalWrite(C_output6, !c_state[5]);
    digitalWrite(C_output7, !c_state[6]);
    digitalWrite(C_output8, !c_state[7]);
}


void send_data(){ // send Data to server
  // generate two fake values as payload - by using a separate stash,
  // we can determine the size of the generated message ahead of time
  cospi = cos(atan((float)reactive_power[2]/(float)real_power[2])) * 100;  
  byte sd = stash.create();

  
  stash.print("field1=");
  stash.print(String(kwh));
  stash.print("&field2=");
  stash.print(String(cospi));
  stash.print("&field3=");
  stash.print(String(jumlah_cap()));

  stash.save();

  // generate the header with payload - note that the stash size is used,
  // and that a "stash descriptor" is passed in as argument using "$H"
  Stash::prepare(PSTR("POST /update HTTP/1.0" "\r\n"
                   "Host: $F" "\r\n"
                    "Connection: close" "\r\n"
                    "X-THINGSPEAKAPIKEY: $F" "\r\n"
                    "Content-Type: application/x-www-form-urlencoded" "\r\n"
                    "Content-Length: $D" "\r\n"
                    "\r\n"
                    "$H"),
                    website, PSTR(APIKEY), stash.size(), sd);
  // send the packet - this also releases all stash buffers once done
  session = ether.tcpSend();
}

void initialize_ethernet(void){ // ethernet setup 
  Serial2.println("\n[getDHCPandDNS]");
  
  if (ether.begin(sizeof Ethernet::buffer, mymac,PA4) == 0) 
    Serial2.println( "Failed to access Ethernet controller");
   
    
  if (!ether.dhcpSetup())
    Serial2.println("DHCP failed");
  
  ether.printIp("My IP: ", ether.myip);
  // ether.printIp("Netmask: ", ether.mymask);
  ether.printIp("GW IP: ", ether.gwip);
  ether.printIp("DNS IP: ", ether.dnsip);

  if (!ether.dnsLookup(website))
    Serial2.println("DNS failed");
  ether.printIp("Server: ", ether.hisip);
  

}
void expSmoothing(){
  // a = 0.001998001332667 --> Q0.31 = 4290675
  // x(t) = a* u(t) + (1-a)* x(t-1)

  //Q24
  voltage_magnitude[2] =  ((int64_t)4292820  * (int64_t)voltage_magnitude[0] + (int64_t)  4290674475 * (int64_t)voltage_magnitude[1]) >> 32;
  voltage_magnitude[1] = voltage_magnitude[2];

  current_magnitude[2] =  (((int64_t)4292820 * (int64_t) current_magnitude[0]) + ((int64_t) 4290674475 * (int64_t)current_magnitude[1])) >> 32;
  current_magnitude[1] = current_magnitude[2];

  real_power[2] =  ((((int64_t)4292820 * (int64_t)real_power[0]) + ((int64_t) 4290674475 * (int64_t)real_power[1])) >> 32);
  real_power[1] = real_power[2];

  reactive_power[2] =  (((int64_t)4292820 * reactive_power[0]) + ((int64_t) 4290674475 * reactive_power[1])) >> 32;
  reactive_power[1] = reactive_power[2];
}

void read_sensor(){ //read sensor , find magnitude square, and zero crossing detection for phase measurement  
  
  // Vrms output = 0.234 = 0.3309...
  
  //Q3.12 
  adc_voltage_value = ((int16_t)(analogRead(voltage_sensor)) - (int16_t)2044); // 
  adc_current_value = ((int16_t)(analogRead(current_sensor)) - (int16_t)1974); //fix
  static int16_t adc_V_filtered[15] = {0};
  static int16_t adc_I_filtered[15] = {0};
  int16_t v_original = 0;
  

  ///////////////////////////FIR function for filtering sensor output/////////////////////////////////////
  ///////////////////////////15th orde FIR                            /////////////////////////////////////
  v_original = (((int32_t)adc_V_filtered[14] + (int32_t)adc_V_filtered[13] + (int32_t)adc_V_filtered[12] + (int32_t)adc_V_filtered[11] +
              (int32_t)adc_V_filtered[10] + (int32_t)adc_V_filtered[9]  + (int32_t)adc_V_filtered[8]  + (int32_t)adc_V_filtered[7]  +
              (int32_t)adc_V_filtered[6]  + (int32_t)adc_V_filtered[5]  + (int32_t)adc_V_filtered[4]  + (int32_t)adc_V_filtered[3]  +
              (int32_t)adc_V_filtered[2]  + (int32_t)adc_V_filtered[1]  + (int32_t)adc_V_filtered[0]  + (int32_t)adc_voltage_value  
              ) >> 4);
  
  i_alpha= (((int32_t)adc_I_filtered[14] + (int32_t)adc_I_filtered[13] + (int32_t)adc_I_filtered[12] + (int32_t)adc_I_filtered[11] +
                (int32_t)adc_I_filtered[10] + (int32_t)adc_I_filtered[9]  + (int32_t)adc_I_filtered[8]  + (int32_t)adc_I_filtered[7]  +
                (int32_t)adc_I_filtered[6]  + (int32_t)adc_I_filtered[5]  + (int32_t)adc_I_filtered[4]  + (int32_t)adc_I_filtered[3]  +
                (int32_t)adc_I_filtered[2]  + (int32_t)adc_I_filtered[1]  + (int32_t)adc_I_filtered[0]  + (int32_t)adc_current_value  
                ) >> 4);
  
  //delaying 15 samples for voltage value
  adc_V_filtered[14] = adc_V_filtered[13];
  adc_V_filtered[13] = adc_V_filtered[12];
  adc_V_filtered[12] = adc_V_filtered[11];
  adc_V_filtered[11] = adc_V_filtered[10];
  adc_V_filtered[10] = adc_V_filtered[9];
  adc_V_filtered[9]  = adc_V_filtered[8];
  adc_V_filtered[8]  = adc_V_filtered[7];
  adc_V_filtered[7]  = adc_V_filtered[6];
  adc_V_filtered[6]  = adc_V_filtered[5];
  adc_V_filtered[5]  = adc_V_filtered[4];
  adc_V_filtered[4]  = adc_V_filtered[3];
  adc_V_filtered[3]  = adc_V_filtered[2];
  adc_V_filtered[2]  = adc_V_filtered[1];
  adc_V_filtered[1]  = adc_V_filtered[0];
  adc_V_filtered[0]  = adc_voltage_value;
  
  //delaying 15 samples for current value
  adc_I_filtered[14] = adc_I_filtered[13];
  adc_I_filtered[13] = adc_I_filtered[12];
  adc_I_filtered[12] = adc_I_filtered[11];
  adc_I_filtered[11] = adc_I_filtered[10];
  adc_I_filtered[10] = adc_I_filtered[9];
  adc_I_filtered[9]  = adc_I_filtered[8];
  adc_I_filtered[8]  = adc_I_filtered[7];
  adc_I_filtered[7]  = adc_I_filtered[6];
  adc_I_filtered[6]  = adc_I_filtered[5];
  adc_I_filtered[5]  = adc_I_filtered[4];
  adc_I_filtered[4]  = adc_I_filtered[3];
  adc_I_filtered[3]  = adc_I_filtered[2];
  adc_I_filtered[2]  = adc_I_filtered[1];
  adc_I_filtered[1]  = adc_I_filtered[0];
  adc_I_filtered[0]  = adc_current_value;
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  
  v_alpha   = current_delay[43];
  v_beta = current_delay[93];
  i_beta   = voltage_delay[49];

//  if(v_alpha >=0 && current_delay[44] <= 0){
//    Serial2.println(2);
//  }else if(i_alpha >=0 && voltage_delay[0] <= 0){
//    Serial2.println(1);
//  }else{
//    Serial2.println(0);
//  }

  voltage_magnitude[0] = ((int32_t) v_alpha*(int32_t)v_alpha) + ((int32_t) v_beta * (int32_t)v_beta);
  current_magnitude[0] = ((int32_t) i_alpha*(int32_t)i_alpha) + ((int32_t) i_beta * (int32_t)i_beta);
  real_power[0]        = ((int32_t) v_alpha*(int32_t)i_alpha) + ((int32_t) v_beta * (int32_t)i_beta);
  reactive_power[0]    = ((int32_t) v_beta *(int32_t)i_alpha) - ((int32_t) v_alpha* (int32_t)i_beta);
  
  //delayig 50 voltage samples
  voltage_delay[50] = voltage_delay[49];
  voltage_delay[49] = voltage_delay[48];  
  voltage_delay[48] = voltage_delay[47];
  voltage_delay[47] = voltage_delay[46];
  voltage_delay[46] = voltage_delay[45];
  voltage_delay[45] = voltage_delay[44];
  voltage_delay[44] = voltage_delay[43];
  voltage_delay[43] = voltage_delay[42];
  voltage_delay[42] = voltage_delay[41];
  voltage_delay[41] = voltage_delay[40];
  voltage_delay[40] = voltage_delay[39];
  voltage_delay[39] = voltage_delay[38];
  voltage_delay[38] = voltage_delay[37];  
  voltage_delay[37] = voltage_delay[36];
  voltage_delay[36] = voltage_delay[35];
  voltage_delay[35] = voltage_delay[34];
  voltage_delay[34] = voltage_delay[33];
  voltage_delay[33] = voltage_delay[32];
  voltage_delay[32] = voltage_delay[31];
  voltage_delay[31] = voltage_delay[30];
  voltage_delay[30] = voltage_delay[29];
  voltage_delay[29] = voltage_delay[28];
  voltage_delay[28] = voltage_delay[27];  
  voltage_delay[27] = voltage_delay[26];
  voltage_delay[26] = voltage_delay[25];
  voltage_delay[25] = voltage_delay[24];
  voltage_delay[24] = voltage_delay[23];
  voltage_delay[23] = voltage_delay[22];
  voltage_delay[22] = voltage_delay[21];
  voltage_delay[21] = voltage_delay[20];
  voltage_delay[20] = voltage_delay[19];
  voltage_delay[19] = voltage_delay[18];
  voltage_delay[18] = voltage_delay[17];
  voltage_delay[17] = voltage_delay[16];  
  voltage_delay[16] = voltage_delay[15];
  voltage_delay[15] = voltage_delay[14];
  voltage_delay[14] = voltage_delay[13];
  voltage_delay[13] = voltage_delay[12];
  voltage_delay[12] = voltage_delay[11];
  voltage_delay[11] = voltage_delay[10];
  voltage_delay[10] = voltage_delay[9];
  voltage_delay[9]  = voltage_delay[8];
  voltage_delay[8]  = voltage_delay[7];
  voltage_delay[7]  = voltage_delay[6];
  voltage_delay[6]  = voltage_delay[5];  
  voltage_delay[5]  = voltage_delay[4];
  voltage_delay[4]  = voltage_delay[3];
  voltage_delay[3]  = voltage_delay[2];
  voltage_delay[2]  = voltage_delay[1];
  voltage_delay[1]  = voltage_delay[0];
  voltage_delay[0]  = i_alpha;

  //delaying 50 urrent samples
  current_delay[108]= current_delay[107];  
  current_delay[107]= current_delay[106];  
  current_delay[106]= current_delay[105];
  current_delay[105]= current_delay[104];
  current_delay[104]= current_delay[103];
  current_delay[103]= current_delay[102];
  current_delay[102]= current_delay[101];
  current_delay[101]= current_delay[100];
  current_delay[100]= current_delay[99];
  current_delay[99] = current_delay[98];  
  current_delay[98] = current_delay[97];
  current_delay[97] = current_delay[96];
  current_delay[96] = current_delay[95];
  current_delay[95] = current_delay[94];
  current_delay[94] = current_delay[93];
  current_delay[93] = current_delay[92];
  current_delay[92] = current_delay[91];
  current_delay[91] = current_delay[90];
  current_delay[90] = current_delay[89];
  current_delay[89] = current_delay[88];
  current_delay[88] = current_delay[87];  
  current_delay[87] = current_delay[86];
  current_delay[86] = current_delay[85];
  current_delay[85] = current_delay[84];
  current_delay[84] = current_delay[83];
  current_delay[83] = current_delay[82];
  current_delay[82] = current_delay[81];
  current_delay[81] = current_delay[80];
  current_delay[80] = current_delay[79];
  current_delay[79] = current_delay[78];
  current_delay[78] = current_delay[77];  
  current_delay[77] = current_delay[76];
  current_delay[76] = current_delay[75];
  current_delay[75] = current_delay[74];
  current_delay[74] = current_delay[73];
  current_delay[73] = current_delay[72];
  current_delay[72] = current_delay[71];
  current_delay[71] = current_delay[70];
  current_delay[70] = current_delay[69];
  current_delay[69] = current_delay[68];
  current_delay[68] = current_delay[67];
  current_delay[67] = current_delay[66];  
  current_delay[66] = current_delay[65];
  current_delay[65] = current_delay[64];
  current_delay[64] = current_delay[63];
  current_delay[63] = current_delay[62];
  current_delay[62] = current_delay[61];
  current_delay[61] = current_delay[60];
  current_delay[60] = current_delay[59];
  current_delay[59]  = current_delay[58];
  current_delay[58]  = current_delay[57];//59
  current_delay[57]  = current_delay[56];//58
  current_delay[56]  = current_delay[55];//57
  current_delay[55]  = current_delay[54];//56
  current_delay[54]  = current_delay[53];//55
  current_delay[53]  = current_delay[52];//54
  current_delay[52]  = current_delay[51];//53
  current_delay[51]  = current_delay[50];//52
  current_delay[50]  = current_delay[49]; //51
  
  current_delay[49] = current_delay[48]; //delay 50 samples  
  current_delay[48] = current_delay[47];
  current_delay[47] = current_delay[46];
  current_delay[46] = current_delay[45];
  current_delay[45] = current_delay[44];
  current_delay[44] = current_delay[43];
  current_delay[43] = current_delay[42];
  current_delay[42] = current_delay[41];
  current_delay[41] = current_delay[40];
  current_delay[40] = current_delay[39];
  current_delay[39] = current_delay[38];
  current_delay[38] = current_delay[37];  
  current_delay[37] = current_delay[36];
  current_delay[36] = current_delay[35];
  current_delay[35] = current_delay[34];
  current_delay[34] = current_delay[33];
  current_delay[33] = current_delay[32];
  current_delay[32] = current_delay[31];
  current_delay[31] = current_delay[30];
  current_delay[30] = current_delay[29];
  current_delay[29] = current_delay[28];
  current_delay[28] = current_delay[27];  
  current_delay[27] = current_delay[26];
  current_delay[26] = current_delay[25];
  current_delay[25] = current_delay[24];
  current_delay[24] = current_delay[23];
  current_delay[23] = current_delay[22];
  current_delay[22] = current_delay[21];
  current_delay[21] = current_delay[20];
  current_delay[20] = current_delay[19];
  current_delay[19] = current_delay[18];
  current_delay[18] = current_delay[17];
  current_delay[17] = current_delay[16];  
  current_delay[16] = current_delay[15];
  current_delay[15] = current_delay[14];
  current_delay[14] = current_delay[13];
  current_delay[13] = current_delay[12];
  current_delay[12] = current_delay[11];
  current_delay[11] = current_delay[10];
  current_delay[10] = current_delay[9];
  current_delay[9]  = current_delay[8];
  current_delay[8]  = current_delay[7];
  current_delay[7]  = current_delay[6];
  current_delay[6]  = current_delay[5];  
  current_delay[5]  = current_delay[4];
  current_delay[4]  = current_delay[3];
  current_delay[3]  = current_delay[2];
  current_delay[2]  = current_delay[1];
  current_delay[1]  = current_delay[0];
  current_delay[0]  = v_original;

  
}

void SAL(){ //// succesive approximation algorithm with fuzzy
    int total = 0;
    int index = 0;
    input_V =sqrt(voltage_magnitude[2]);
    input_Q = (reactive_power[2] * (float)2.11 / 1438);
    
      if(input_Q > 0){
        fuzzyfication();
        rule_base();
        defuzzyfication();
        total = jumlah_cap() + (int)C;
        
      }else if(input_Q < 0){
        input_Q *= -1;
        fuzzyfication();
        rule_base();
        defuzzyfication();
        total = jumlah_cap() - (int)C;
        
      }
      // fuzzy logic run after input_Q  were defined
      reset_cap();

      if (total > 119){ // saturation 
        total = 119;
      }else if(total < 0){
        total = 0;
      }
      
      while (total != jumlah_cap()){ // this loop run succesive approximation algorithm function
        if (jumlah_cap() > total){
          c_state[index] = 1;
          c_state[index-1] = 0;
        }else if (jumlah_cap() < total){
          c_state[index] = 1;
        }
        index++;
      }
    
     
  
/* PS : run tested on 01 may 2015, 22:14 +07.00 GMT
 * Worst Execution time : 83 us
 * running and working properly
 */
}

int jumlah_cap(){ // looping for determine sum of which capacitors were activated
  int total = 0;
 if (c_state[0] == 1){
  total+= capacitor[0]; 
 }
 if (c_state[1] == 1){
  total+= capacitor[1]; 
 }
 if (c_state[2] == 1){
  total+= capacitor[2]; 
 }
 if (c_state[3] == 1){
  total+= capacitor[3]; 
 }
 if (c_state[4] == 1){
  total+= capacitor[4]; 
 }
 if (c_state[5] == 1){
  total+= capacitor[5]; 
 }
 if (c_state[6] == 1){
  total+= capacitor[6]; 
 }
 if (c_state[7] == 1){
  total+= capacitor[7]; 
 }
 
  return total;  
}

void reset_cap(){ // looping for reset all capacitor deactivate
  c_state[0] = 0;
  c_state[1] = 0;
  c_state[2] = 0;
  c_state[3] = 0;
  c_state[4] = 0;
  c_state[5] = 0;
  c_state[6] = 0;
  c_state[7] = 0;
  
}

void fuzzyfication(){ // 2 input 3 MF

////////////////////////////////////////////////////////////////////// FUZZIFIKASI UNTUK INPUT DAYA Reaktif //////////////////////////////////////////////////
/*
 * Membership function for input Q is formatted by Q16.0(0-900)
 * 1 / 450 ==> representated by Q0.32 --> 9544372
 * Membership degree represented by Q0.32 format
 * 4294967295 --> Q0.32, real value 1
 */
  
  
  if (input_Q == 0){ // Q = 0 (*Reactive Power)
    MD_Q_mf1 = 1;
    MD_Q_mf2 = 0;
    MD_Q_mf3 = 0;
  }
  
  if(input_Q >  0 && input_Q < 450 ){ // 0 < Q  < 450
    MD_Q_mf1 =  (450   -input_Q)*(0.002222222222222);
    MD_Q_mf2 =  (input_Q-     0)*(0.002222222222222);
    MD_Q_mf3 = 0; 
  }
  if (input_Q == 450){ // Q = 450
    MD_Q_mf1 = 0;
    MD_Q_mf2 = 1;
    MD_Q_mf3 = 0;
  }
  if (input_Q > 450 && input_Q < 900){ // 450 < Q < 900
    MD_Q_mf1 = 0;
    MD_Q_mf2 = (900 - input_Q)*(0.002222222222222);
    MD_Q_mf3 = (input_Q-  450)*(0.002222222222222);
  }
  if (input_Q == 900){ // Q = 900
    MD_Q_mf1 = 0;
    MD_Q_mf2 = 0;
    MD_Q_mf3 = 1;
  }
  
// output MD_Q_xxx is Q0.32 format (32 bit)
  


  /*
 * Membership function for input V is formatted by Q16.0 (200-240)
 * 1 / 20 ==> representated by Q0.32 --> 214748365
 * Membership degree represented by Q0.32 format
 * 4294967295 --> Q0.32, real value 1
 */
 
 if(input_V == 200){
  MD_V_mf1 = 1;
  MD_V_mf2 = 0;
  MD_V_mf3 = 0;
 }
 
 if(input_V > 200 && input_V < 220){
  MD_V_mf1 = (220 - input_V)*(0.05);
  MD_V_mf2 = (input_V - 200)*(0.05);;
  MD_V_mf3 = 0;
 }
 if(input_V == 220){
  MD_V_mf1 = 0;
  MD_V_mf2 = 4294967295;
  MD_V_mf3 = 0;
 }
 if(input_V > 220 && input_V < 240){
  MD_V_mf1 = 0;
  MD_V_mf2 = (240 - input_V)*(0.05);
  MD_V_mf3 = (input_V - 220)*(0.05);
 }
 if(input_V == 240){
  MD_V_mf1 = 0;
  MD_V_mf2 = 0;
  MD_V_mf3 = 1;
 }

// output MD_V_xxx is Q0.32 format (32 bit)

 
  
/* PS : Tested, properly working and running 
 *  test date : 01 may 2018, 11:43 +7.00 GMT
 */
}

void rule_base(){ // linear fuzzy sugeno
  w1 = fmin(MD_V_mf1,MD_Q_mf1);
  w2 = fmin(MD_V_mf1,MD_Q_mf2);
  w3 = fmin(MD_V_mf1,MD_Q_mf3);
  w4 = fmin(MD_V_mf2,MD_Q_mf1);
  w5 = fmin(MD_V_mf2,MD_Q_mf2);
  w6 = fmin(MD_V_mf2,MD_Q_mf3);
  w7 = fmin(MD_V_mf3,MD_Q_mf1);
  w8 = fmin(MD_V_mf3,MD_Q_mf2);
  w9 = fmin(MD_V_mf3,MD_Q_mf3);

/*Multiplier with input_Q represented by Q0.32 format
 *Multiplier with input_V represented by Q0.32 format
 *f output is represented by format Q7.25 because capacitor till 71 uF
 */
  
  //               Q31.32                        Q31.32
                             
  f1 = (-0.0984107773506878*input_V +0.073647116436796*input_Q + 19.2971550754897  ) ;
  f2 = (-0.314006058882155*input_V +0.0723104795871478*input_Q + 66.0972575819782 )  ;
  f3 = (-0.530389444779454*input_V + 0.0732144646871962*input_Q + 112.242244136792 )  ;
  f4 = (-0.115858496023567*input_V + 0.0661618926810254*input_Q + 25.4938570942755) ;
  f5 = (-0.266862262069458*input_V + 0.066139860856694*input_Q + 58.5333744040901) ;
  f6 = (-0.41905251652591*input_V + 0.0661498477671102*input_Q + 91.8375563596481 )  ;
  f7 = (-0.0961152446921975*input_V + 0.0588192921109165*input_Q + 23.4740542003612 );
  f8 = (-0.233130636789534*input_V + 0.0600555452759156*input_Q +  53.8237245667409) ;
  f9 = (-0.371143994042139*input_V + 0.0585448877198033*input_Q + 85.771519256901 )  ;

  

}

void defuzzyfication(){ // weight average method defuzzyfication
  /*       Q7.56
   *  C =  -----    =  C Q(7.25) 
   *       Q31.32
   */
  C =    round(
         (f1 * w1 +  f2 * w2 +  f3* w3 +  f4 *w4 +  f5*w5  +f6*w6 + f7*w7 + f8*w8 + f9*w9)/
         ( w1 + w2 + w3 + w4 +  w5 +  w6 +  w7 +  w8 +  w9)
         ) ;
}
