#include "Simple_Function.h"
#include "Constant_Definition.h"
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
// #include <Encoder.h>

ros::NodeHandle nh;
geometry_msgs::Vector3 force;
ros::Publisher force_pub("force_data", &force);

/*== Timer Variables ==*/
Constant_Timer Timer_Encoder ;
Constant_Timer Timer_USB ;
Constant_Timer Timer_Control ;
Constant_Timer Timer_Speed ;

/*== Motor Variables ==*/
volatile signed long slEncoder_Counter = 0 ;
volatile signed long slEncoder_Counter_last = 0 ;
volatile signed long slSpeed = 0;
volatile signed long slLoadCell_Volt_Read_x = 0 ;
volatile signed long slLoadCell_Volt_Read_y = 0 ;
volatile signed long slOutput_test = 0;

//static int iVolt = 0 ;
static int iVolt = SET_VOLT ;
static int iReverse = 10 ;
static char cControlMode = MODE_VOLT ;
static bool bReverse = true ;

/*== Moving average filter setup ==*/
int ma_counter = 0;
volatile signed long avg_num = 10;
volatile signed long avg_volt_x[10] = {0};
volatile signed long avg_volt_y[10] = {0};
volatile signed long avg_x;
volatile signed long avg_y;
int i;

// Encoder myEnc(PIN_ENCODER_A, PIN_ENCODER_B ) ;

void setup(){
  nh.initNode();
  nh.advertise(force_pub);
	Hardware_Setup() ;
	Software_Setup() ;
}

void loop(){
	// USB_CMD_Task() ; // 下達指令
	Control_Task() ; // 進行Motor Control
  Encoder_Task() ; // 讀取Encoder
  force_pub.publish(&force);
  nh.spinOnce();
  delay(10);
}

/*========= Function =========*/
/*==== Setup Function ====*/
void Hardware_Setup(void ){
	/*== LED Setup Function ==*/
		// config IO direction of LED
	pinMode(PIN_LED_ONBOARD, OUTPUT ) ;

	/*== USB Setup Function ==*/
  // config USB UART baud rate
	// Serial.begin(BAUDRATE_USB ) ;

	/*== Motor Setup Function ==*/
	pinMode(PIN_INA, OUTPUT ) ;
	pinMode(PIN_INB, OUTPUT ) ;
	pinMode(PIN_ENABLE, OUTPUT ) ;

	pinMode(PIN_ENCODER_A, INPUT ) ;
	pinMode(PIN_ENCODER_B, INPUT ) ;
  pinMode(X_AXIS, INPUT);
  pinMode(Y_AXIS, INPUT);
	// attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A ), Encoder_Task_Simple, CHANGE ) ;
	// attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B ), Encoder_Task_Simple, CHANGE ) ;

	attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A ), Encoder_Task, CHANGE ) ;
	attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B ), Encoder_Task, CHANGE ) ;

	// Encoder_Task_Simple() ;
}

void Software_Setup(void ){
	/*== Timer Setup Function ==*/
	Timer_USB.Timer_Task(0 ) ;
	Timer_Encoder.Timer_Task(0 ) ;
	Timer_Control.Timer_Task(0 ) ;
  Timer_Speed.Timer_Task(0 ) ;

	/*== Motor Control Setup Function ==*/
	slEncoder_Counter = 0 ;
  //iReverse = 0;
  //iVolt = 0;
	//cControlMode = MODE_STOP ;
	MOTOR_STOP() ;

	/*== Setup Finished!! ==*/
	// Serial.print("SetPoint" ) ;
	// Serial.print(" ") ;
	// Serial.println("Encoder" ) ;

}

/*==== Basic Function ====*/

/*== USB Function ==*/
// void USB_CMD_Task(void ){
// 	static signed long _slData0 = 0 ;
// 	static int _iData0 = 0 ;
//   static int _iTest0 = 0;
// 	static char _cCommand, _cCommand2, _cTemp ;

// 	static String _sStringIn = "" ;
// 	static char _cDataIn[BUFFER_SIZE_USB] ;
// 	static bool _bData_Complete ;

// 	static int _iTemp =  0 ;
	
//   // Read buffer
// 	while((_cTemp = Serial.read() ) != -1 )
// 	{
// 		if(_cTemp == '\n'){
// 			_bData_Complete = true ;
// 			break ;
// 		}
// 		else _sStringIn += _cTemp ;
// 	}

// 	if(_bData_Complete ){
// 		_sStringIn.toCharArray(_cDataIn, BUFFER_SIZE_USB ) ;
// 		// check case
// 		switch(_cDataIn[0] )
// 		{
//       //輸入指令的時候打：s0 -> 停止; v0 -> 力控制模式; r0 -> 迴轉模式
//       case CASE_STOP:
//         cControlMode = MODE_STOP ;
// 				_iTemp = sscanf(_cDataIn, "s%d", &_iData0) ;
// 			break ;
//       case CASE_VOLT:
// 				cControlMode = MODE_VOLT ;
// 				_iTemp = sscanf(_cDataIn, "v%d", &_iData0) ;
// 				if(_iTemp > 0 ){
//           iVolt = SET_VOLT ;
// 				}
// 			break ;
//       case CASE_REVERSE:
//         cControlMode = MODE_REVERSE ;
//         _iTemp = sscanf(_cDataIn, "r%d", &_iData0 ) ;
//         if(_iTemp > 0 ){
// 					iReverse = 10;
// 				}
// 			default:
// 			break ;
// 		}
// 		_sStringIn = "" ;
// 		_bData_Complete = false ;
// 	}
// }

// void USB_Plot_VoltTask(void ){
// 	if(Timer_USB.Timer_Task(TIME_USB_MS_PLOT ) ){
// 		Serial.print(iVolt) ;
// 		Serial.print(" ") ;
//     //Serial.print(slLoadCell_Volt_Read_y) ;
//     Serial.print(avg_y) ;
//     Serial.print(" ") ;
//     Serial.print(avg_x) ;
// 		Serial.print(" ") ;
// 		Serial.println(slOutput_test ) ;
// 	}
// }

/*== Motor Function ==*/
void Encoder_Task(void ){
	/*== Encoder Conunter ==*/
	static uint8_t _pin_data = 0x00 ;
	static uint8_t _pin_data_last = 0x00 ;
	static uint8_t _pin_temp_1 = 0x00 ;
	static uint8_t _pin_temp_2 = 0x00 ;

	_pin_data = DATA_ENCODER ;
	_pin_temp_1 = _pin_data^_pin_data_last ;
	_pin_temp_2 = (_pin_data >> 1 )^_pin_data_last ;

	if(_pin_temp_1 & MASK_ENCODER ){
		if(_pin_temp_2 & MASK_ENCODER_PLUS ) slEncoder_Counter++ ;
		else slEncoder_Counter-- ;
	}

	_pin_data_last = _pin_data ;
}

void Control_Task(void ){
  static int _n = 0;
	static int _iAbs_duty ;
  static int _iCurrentTime = 0; //ms
  static int _iPrevTime = 0; //ms
	static signed long _slOutput_temp ;
  static signed long _slVolt_err ;
	static signed long _slDelta_err_v ;
	static signed long _slIntegral_err_v ;
	static signed long _slPrev_err_v ;
  static signed long _slEncoder_Counter_temp;
  static bool _bSign_output = true;
	static float _fTemp1, _fTemp2 = 0.0 ;
  
  // Read load cell (LC) value, and add an offset 700 (the direction of LC in assembly is opposite)
  slLoadCell_Volt_Read_x = LOAD_CELL_OFFSET - analogRead(X_AXIS);
  slLoadCell_Volt_Read_y = LOAD_CELL_OFFSET - analogRead(Y_AXIS); 
  // Moving average (ma) filter to smooth out the LC value
  avg_x=0;
  avg_y=0;
  ma_counter = ma_counter%avg_num;
  avg_volt_x[ma_counter] = slLoadCell_Volt_Read_x;
  avg_volt_y[ma_counter] = slLoadCell_Volt_Read_y;
  for(i = 0; i<avg_num; i++)
  {
    avg_x += avg_volt_x[i]/avg_num;
    avg_y += avg_volt_y[i]/avg_num;
  }
  ma_counter++;
  // ma end. output is "avg_x" and "avg_y"

  force.x = avg_x;
  force.y = avg_y;
  force.z = 0;

  if(cControlMode == MODE_STOP){
    _slOutput_temp = 0;
    if(_slOutput_temp > 0 ) MOTOR_CW(abs(_slOutput_temp )) ;
    else MOTOR_CCW(abs(_slOutput_temp )) ;
  }
	
  else if(cControlMode == MODE_VOLT){
    if(Timer_Control.Timer_Task(TIME_FORCE_CONTROL_MS ) ){
      /*==安全機制==*/
      /*=超過MAX_VOLT觸發MODE_REVERSE=*/
      //if(slLoadCell_Volt_Read_y >= MAX_VOLT) cControlMode = MODE_REVERSE;
      if(avg_y >= MAX_VOLT ) cControlMode = MODE_REVERSE;
	    
      /*==PID Voltage Control==*/
      /*=達到SET_VOLT(iVolt)夾爪停止轉動=*/
      //_slVolt_err = (signed long)iVolt - slLoadCell_Volt_Read_y ; 
      _slVolt_err = (signed long)iVolt - avg_y ;
      _slDelta_err_v = _slVolt_err - _slPrev_err_v ;
      _slIntegral_err_v += _slVolt_err ;

      _slOutput_temp = ((_slVolt_err * FORCE_CONTROL_P + 
               _slIntegral_err_v * FORCE_CONTROL_I + 
                _slDelta_err_v * FORCE_CONTROL_D ) >> 9 ) ;
      _slPrev_err_v = _slVolt_err ;
      slOutput_test = _slOutput_temp ;

      if(_slOutput_temp >= 0 ) _bSign_output = true ;
      else _bSign_output = false ;

      _slOutput_temp = abs(_slOutput_temp) ;

      if(_slOutput_temp > MAX_DUTY ){
        _slOutput_temp = MAX_DUTY ;
      }

      if(_bSign_output ){
        MOTOR_CW(_slOutput_temp ) ;
      }
      else{
        MOTOR_CCW(_slOutput_temp ) ;
      }
    }
  }
  else if(cControlMode == MODE_REVERSE){
    if(Timer_Speed.Timer_Task(TIME_SPEED_CONTROL_MS ) ){
    _slOutput_temp = iReverse;
    MOTOR_CCW(_slOutput_temp) ;
    }
    if(bReverse){
      bReverse = false;
      _slEncoder_Counter_temp = slEncoder_Counter;
    }
    if(abs(slEncoder_Counter - _slEncoder_Counter_temp) > 50) 
      cControlMode = MODE_STOP;
  }
}
