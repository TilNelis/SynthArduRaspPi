#include <Adafruit_NeoPixel.h>
#include <MIDI_Controller.h> // Include the library
//defines the digital Pin the LED Stripe is connected to
#define LED_PIN     6

//defines how many LED there are in the Stripe
#define LED_COUNT  30

#define OFF 0

#define BRIGHTNESS 100

#define OSCILLATOR_1 5 //defines how many LED there are in Oscillator 1
#define SYNCMODE_OSC_1 2 //defines how many LED there are in syncmode OSC 1
#define OSCILLATOR_2 5 //defines how many LED there are in Oscillator 2
#define SYNCMODE_OSC_2 2 //defines how many LED there are in syncmode OSC 2
#define FILTERTYPE 4 //defines how many LED there are in Filtertype
#define FILTERSLOPE 4 //defines how many LED there are in Filterslope
#define LFOTYPE 5 //defines how many LED there are in the LFOtype
#define VOICING 3 //defines how many LED there are in Voicing

#define NUM_CONTROL_UNITS 8 //defines how many control Units there are
#define MAX_NUM_LEDS_PER_CONTROL_UNIT 5 //defines the maximal number of LEDs at one control unit

//defines digital Outputs for LED
#define Button01 22 //defines the Digital Pin where Button01 is connected to
#define Switch01 23 //defines the Digital Pin where Switch01 is connected to
#define Button02 24 //defines the Digital Pin where Button02 is connected to
#define Switch02 25 //defines the Digital Pin where Switch02 is connected to
#define Button03 26 //defines the Digital Pin where Button03 is connected to
#define Button04 27 //defines the Digital Pin where Button04 is connected to
#define Button05 28 //defines the Digital Pin where Button05 is connected to
#define Button06 29 //defines the Digital Pin where Button06 is connected to

#define IS_SWITCH 0 // Boolean variable defining a switch
#define IS_BUTTON 1 // Boolean variable defining a button
//Those are mutually exclusive

#define DEFAULT_BAUD_RATE 9600 //defines the default baud rate


int i = 0;
int Val_Red = 0;
int Val_Green = 0;
int Val_Blue = 50;


// define constants for making clear that the meaning of the array parameters becomes clear
#define LED_COUNT_MATRIX 0 //
#define CONTROLLER_FLAG 1
#define DIGITAL_PIN 2

int control_unit [NUM_CONTROL_UNITS][3]= {
  {OSCILLATOR_1,    IS_BUTTON, Button01},
  {SYNCMODE_OSC_1,  IS_SWITCH, Switch01},
  {OSCILLATOR_2,    IS_BUTTON, Button02},
  {SYNCMODE_OSC_2,  IS_SWITCH, Switch02},
  {FILTERTYPE,      IS_BUTTON, Button03},
  {FILTERSLOPE,     IS_BUTTON, Button04},
  {LFOTYPE,         IS_BUTTON, Button05},
  {VOICING,         IS_BUTTON, Button06}
};

int Local_offset_current_value = 0;

int global_start_index_LED [NUM_CONTROL_UNITS]; 
//indicates the start for a new control unit eg. the first switch has the 
//value of 5 since the first LED for this control unit is the sixth led in the row

int local_offset_of_burning_LED [NUM_CONTROL_UNITS];
//indicates the offset of the burning LED in each control unit, so if the second LED in any control unit should be illuminated, 
//the value would be 1, since the LED is the second in the control unit 

int cu_current_state[NUM_CONTROL_UNITS]; //cu: control unit; Array for constantly measuring the digital inputs of the arduino (where the switches and buttons are connected to)
int cu_old_state [NUM_CONTROL_UNITS];

// Declare the NeoPixel strip object:
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//---------------------------------------------------------------------------------------------------------------
//the following lines are for the setup of MIDI

//int digitalMIDIread[] = {NUM_CONTROL_UNITS}; //array for storing current state

int controlChange = 176; //controll Change order for MIDI
int noteON = 144;

//________________________________________________________________________________________________________________________________
//Following lines are for MIDI from the Potentiometers, including code for multiplexer


const size_t analogAverage = 8; // Use the average of 8 samples to get smooth transitions and prevent noise


const uint8_t numberOfMultiplexers = 6; // Total number of multiplexers connected to the Arduino
const uint8_t addressWidth = 3; // Number of address lines of the multiplexers

const uint8_t addressLines[addressWidth] = {2, 3, 4}; // The pins connected to the address lines of the multiplexers: DIGITAL OUTPUT PINS FOR MULTIPLEXER (THE SAME 
//THREE OUTPUTS ARE USED FOR ALL MULTIPLEXER)
const uint8_t analogInputs[numberOfMultiplexers] = {A0, A1, A2, A3, A4, A5}; // The pins connected to the outputs of the multiplexers

//________________________________________________________________________________________________________________________________

const uint8_t numberOfInputsPerMultiplexer = 1 << addressWidth; // Number of inputs of each multiplexer

const size_t totalNumberOfAnalogInputs = numberOfMultiplexers * numberOfInputsPerMultiplexer; // the total number of potentiometers

//________________________________________________________________________________________________________________________________


ControlChange * potentiometers[totalNumberOfAnalogInputs]; // create an empty array of pointers to ControlChange objects

//________________________________________________________________________________________________________________________________




void setup() {

  strip.clear();
  strip.begin();
  strip.show();
  strip.setBrightness(BRIGHTNESS); //set brightness to 100 of 250

  Serial.begin(DEFAULT_BAUD_RATE);

  //declares the inputs for the LEDs
  pinMode(Button01, INPUT);
  pinMode(Switch01, INPUT);
  pinMode(Button02, INPUT);
  pinMode(Switch02, INPUT);
  pinMode(Button03, INPUT);
  pinMode(Button04, INPUT);
  pinMode(Button05, INPUT);
  pinMode(Button06, INPUT);

    // Initializes global arrays - done dynamically in favor of future extensions (e.g. larger number of control units)
  global_start_index_LED[0] = 0; 
  local_offset_of_burning_LED[0] = 0;
  cu_current_state[0] = LOW;
  cu_old_state[0] = LOW;
    
  for (int i=1; i<NUM_CONTROL_UNITS; i++) { //initializes all parameters to either LOW or zero
    global_start_index_LED[i]=global_start_index_LED[i-1]+control_unit[i-1][LED_COUNT_MATRIX];
    local_offset_of_burning_LED[i]=0; //ZUM INITIALISIEREN VON LOCAL OFFSET
    cu_current_state[i]=LOW;
    cu_old_state[i]=LOW;
        
    }

        
  for (int i=0; i <NUM_CONTROL_UNITS; i++){
    strip.setPixelColor(global_start_index_LED[i], Val_Red, Val_Green, Val_Blue); //illuminates the initial LED in each control unit
    strip.show();

  }

    //_____________________________________________________________________________________________________________________

    //Following lines are for MIDI from the Potentiometers, including code for multiplexer

    USBMidiController.blink(LED_BUILTIN);  // flash the built-in LED (pin 13 on most boards) on every message
    USBMidiController.setDelay(15);  // wait 15 ms after each message not to flood the connection
    USBMidiController.begin();  // Initialise the USB MIDI connection
    delay(1000); // Wait a second...
    for (unsigned int i = 0; i < totalNumberOfAnalogInputs; i++) {
    potentiometers[i] = new ControlChange(i, 1); // create a new ControlChange object and store a pointer to it in the array
    potentiometers[i]->average(analogAverage); // Use the average of 8 samples to get smooth transitions and prevent noise
    }
    for (unsigned int i = 0; i < addressWidth; i++) {
    pinMode(addressLines[i], OUTPUT); // set all address line pins as outputs
    }
  

}


//___________________________________________________________________________________
//Following lines are functions used in the code

void set_nth_pixel_to_RGB(int num_index, int R_value, int G_value, int B_value){
  // num_index is the global count, not the local one!
  strip.setPixelColor(num_index, R_value, G_value, B_value);
  strip.show();
  
}

void send_some_MIDI(int command, int controllerNumberMIDI, int Value){

  Serial.write(command); //e.g. controlchange: 176
    Serial.write(controllerNumberMIDI); //indicates which controller sends the midi data
    Serial.write(Value); //sets the velocity/Value      
  
}


//__________________________________________________________________________
//for MIDI

void setMuxAddress(unsigned int address) {
  for (unsigned int i = 0; i < addressWidth; i++) {
    digitalWrite(addressLines[i], address & (1 << i));
  }
}

int analogReadMux(unsigned int muxPin) {
  int address = muxPin % numberOfInputsPerMultiplexer; // the input of the multiplexer
  int analogPinIndex = muxPin / numberOfInputsPerMultiplexer; // the index of the multiplexer
  setMuxAddress(address); // select the right input of the multiplexer
  analogRead(analogInputs[analogPinIndex]);  // Throw away first reading
  return analogRead(analogInputs[analogPinIndex]); // read the output of the right multiplexer
}

//__________________________________________________________________________



void loop() {

  for (int i=0; i<NUM_CONTROL_UNITS; i++) { 
    cu_current_state[i] = digitalRead(control_unit[i][DIGITAL_PIN]); //reads the pins

          
  if (control_unit[i][CONTROLLER_FLAG] == IS_SWITCH) { //checks if the controller is a switch
    
  
    if(cu_current_state[i] == HIGH && cu_old_state[i] == LOW){ 
      set_nth_pixel_to_RGB(global_start_index_LED[i]  , Val_Red, Val_Green, Val_Blue); // turns "first" LED in control unit, which has for a switch always offset 0, on
      set_nth_pixel_to_RGB(global_start_index_LED[i]+1, OFF, OFF, OFF); // turn "last" LED in control unit , which has for a switch always offset 1, off 
      send_some_MIDI(controlChange, control_unit[i][DIGITAL_PIN], 127);
      
      cu_old_state[i] = cu_current_state[i];//sets both states to HIGH
      
      } 
      //as long as the switch is HIGH, the same LED will be illuminated   


    
      if (cu_current_state[i] == LOW && cu_old_state[i] == HIGH) {
            set_nth_pixel_to_RGB(global_start_index_LED[i]+1, Val_Red, Val_Green, Val_Blue); // turn "last" LED in control unit, which has for a switch always offset 1, on
      set_nth_pixel_to_RGB(global_start_index_LED[i], OFF, OFF, OFF); // turns "first" LED in control unit , which has for a switch always offset 0, off
      cu_old_state[i] = cu_current_state[i]; //sets both states to LOW

      send_some_MIDI(controlChange, control_unit[i][DIGITAL_PIN], 0);       
    
      }
  }
  
         
  if (control_unit[i][CONTROLLER_FLAG] == IS_BUTTON) {      
  
    
    if (cu_current_state[i] == HIGH && cu_old_state[i] == LOW) {

      set_nth_pixel_to_RGB((global_start_index_LED[i] + local_offset_of_burning_LED[i]), OFF, OFF, OFF);//Turns current illuminated LED off
          
        Local_offset_current_value = local_offset_of_burning_LED[i];
                
        local_offset_of_burning_LED[i] = (local_offset_of_burning_LED[i] + 1); //Adds 1 to the local offset; the next LED will be illuminated             
        local_offset_of_burning_LED[i] = local_offset_of_burning_LED[i] % control_unit[i][LED_COUNT_MATRIX];

        send_some_MIDI(controlChange, control_unit[i][DIGITAL_PIN], 127);
          
        cu_old_state[i] = cu_current_state[i];

      set_nth_pixel_to_RGB((global_start_index_LED[i] + local_offset_of_burning_LED[i]),  Val_Red, Val_Green, Val_Blue);//Turns next LED on 
      
        
    }

    if (cu_current_state[i] == LOW && cu_old_state[i] == HIGH) {

      send_some_MIDI(controlChange, control_unit[i][DIGITAL_PIN], 0);

      cu_old_state[i] = cu_current_state[i];
        }
  
      }
    
  }

    //____________________________________________________________________________________________________________

    //Following lines are for MIDI from the Potentiometers, including code for multiplexer
    for (unsigned int i = 0; i < totalNumberOfAnalogInputs; i++) { // refresh all analog inputs
    uint8_t value = analogReadMux(i) >> 3; // convert 10-bit analog value to 7-bit MIDI value (10-7 = 3)
    potentiometers[i]->refresh(value);
  }

}
