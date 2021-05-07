
#define ENCODER_USE_INTERRUPTS 1

#include <Bounce2.h>
#include <FlexyStepper.h>
#include <U8glib.h>
#include <Encoder.h>
#include <avr/eeprom.h>
#include <Wire.h>
#include <TimeLib.h>
#include <Time.h>


// ===================================================================================
// =				 Structure to hold configuration settings                        =
// ===================================================================================
struct settings_t
{
	char Valid[21];
	uint16_t Mem1;
	uint16_t Mem2;
	uint16_t Layer_Jog;
	boolean Invert_Direction;
	uint16_t MM_Soft_Limit;
	uint8_t Big_Jog;
	uint16_t Acceleration_Deceleration;
	uint8_t Acceleration_Threshold;
	uint16_t MM_Speed;
	uint16_t MM_Steps;

} Settings;


// ===================================================================================
// =									Constants									 =
// ===================================================================================

// ...................................................................................
// .									IO Pins										 .
// ...................................................................................
#define Left_Button_Pin A0
#define Center_Button_Pin A1
#define Right_Button_Pin A2

#define Encoder_Clock_Pin 2
#define Encoder_Data_Pin 3
#define Encoder_Press_Pin 4

#define Stepper_Enable_Pin 5

#define Stepper_1_Step_Pin 7
#define Stepper_1_Direction_Pin 8

#define External_Trigger_Pin 10
#define Bottom_Home_Pin 11
#define Top_Home_Pin 12

#define LimitTriggered false


// ===================================================================================
// =								Comms Handshake values							 =
// ===================================================================================
#define ASCII_ETX 3

// ===================================================================================
// =						 serial port allocations								 =
// ===================================================================================
#define PCHost Serial


// ===================================================================================
// =									Enumirations								 =
// ===================================================================================

// ...................................................................................
// .							Button State Enumirations							 .
// ...................................................................................
enum ButtonStates
{
	Unchanged,
	Pressed,
	Released
};

// ...................................................................................
// .							 Engine State Enumirations							 .
// ...................................................................................
enum StateEngineStates
{
	Manual,
	Auto,
	SetJogMM
};


// ===================================================================================
// =									Variables									 =
// ===================================================================================

char buf[3];
uint16_t Target_Z_Pos, Display_Z_Pos,Soft_Limit;
ButtonStates Left_Button_State, Center_Button_State, Right_Button_State, Encoder_Button_State, Trigger_Button_State;
ButtonStates Last_Left_Button_State, Last_Center_Button_State, Last_Right_Button_State, Last_Encoder_Button_State, Last_Trigger_Button_State;
boolean Left_Button_Now, Center_Button_Now, Right_Button_Now, Encoder_Button_Now, Trigger_Button_Now;
boolean  UI_Update, Last_Bottom_Home_State, Last_Top_Home_State, StopHit, Setting, ConfigMode;;
uint8_t RunningMode = Manual;
uint8_t OldIncrement = 0;
time_t BootSeconds;

// ===================================================================================
// =						Data_Port Packet Incoming Data Buffer	                 =
// ===================================================================================
char CommandIn[70];
char PayloadValue[70];
uint8_t CommandPointer = 0;

// ===================================================================================
// =						Object instantiations									 =
// ===================================================================================

Bounce Left_Button = Bounce();
Bounce Right_Button = Bounce();
Bounce Center_Button = Bounce();
Bounce Encoder_Press = Bounce();

Bounce External_Trigger = Bounce();

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);

Encoder Knob(Encoder_Data_Pin, Encoder_Clock_Pin);

FlexyStepper LifterMotors;

// ===================================================================================
// =								Startup Code									 =
// ===================================================================================
void setup()
{
	PCHost.begin(9600);

	while (!PCHost);

	LoadSettings();

	LifterMotors.connectToPins(Stepper_1_Step_Pin, Stepper_1_Direction_Pin, Settings.Invert_Direction);

	SetupMotorParameters();

	pinMode(Stepper_Enable_Pin, OUTPUT);

	pinMode(Encoder_Clock_Pin, INPUT_PULLUP);
	pinMode(Encoder_Data_Pin, INPUT_PULLUP);


	pinMode(Bottom_Home_Pin, INPUT_PULLUP);
	pinMode(Top_Home_Pin, INPUT_PULLUP);

	External_Trigger.interval(5);
	External_Trigger.attach(External_Trigger_Pin, INPUT_PULLUP);

	Left_Button.interval(5);
	Left_Button.attach(Left_Button_Pin, INPUT_PULLUP);

	Right_Button.interval(5);
	Right_Button.attach(Right_Button_Pin, INPUT_PULLUP);

	Center_Button.interval(5);
	Center_Button.attach(Center_Button_Pin, INPUT_PULLUP);



	Encoder_Press.interval(5);
	Encoder_Press.attach(Encoder_Press_Pin, INPUT_PULLUP);


	u8g.setColorIndex(1);

	Splash_Display();

	BootSeconds = now() + 3;

	while (BootSeconds > now() || ConfigMode)
	{

		if (CheckForPCCommands())
		{
			Config_Display();
			ConfigMode = true;
		}

	}

		
	digitalWrite(Stepper_Enable_Pin, HIGH);

	DoHome();

	RunningMode = Manual;

	UI_Update = true;
	Manual_Display();

}

// ===================================================================================
// =								Main Loop										 =
// ===================================================================================
void loop()
{
	CheckButtonStates();
	RunStateEngine();
	MoveBed();
}

// ===================================================================================
// =							State Engine Logic									 =
// ===================================================================================
void RunStateEngine(void)
{

	if ((digitalRead(Bottom_Home_Pin) != Last_Bottom_Home_State) || (digitalRead(Top_Home_Pin) != Last_Top_Home_State))
	{
		Last_Top_Home_State = digitalRead(Top_Home_Pin);
		Last_Bottom_Home_State = digitalRead(Bottom_Home_Pin);
		UI_Update = true;
	}

	ModeChangeTest();

	switch (RunningMode)
	{
	case Manual:
	{

		if (GetKnob() != Target_Z_Pos)
		{
			Target_Z_Pos = GetKnob();
			UI_Update = true;
		}

		if (Left_Button_ChangeState() == Pressed)
		{
			if (Target_Z_Pos >= Settings.Big_Jog)
			{
				Target_Z_Pos -= Settings.Big_Jog;
			}
			else
			{
				Target_Z_Pos = 0;
			}
			SetKnob(Target_Z_Pos);
			LifterMotors.setTargetPositionInMillimeters(Target_Z_Pos);
			UI_Update = true;

		}

		if (Right_Button_ChangeState() == Pressed)
		{
			Target_Z_Pos += Settings.Big_Jog;
			SetKnob(Target_Z_Pos);
			LifterMotors.setTargetPositionInMillimeters(Target_Z_Pos);
			UI_Update = true;

		}

		if (Center_Button_ChangeState() == Pressed)
		{
			DoHome();
			RunningMode = Manual;
			UI_Update = true;
		}

		Manual_Display();


		break;
	}
	case Auto:
	{
		Auto_Display();
		if (Trigger_Button_ChangeState() == Pressed)
		{
			Target_Z_Pos += Settings.Layer_Jog;
			SetKnob(Target_Z_Pos);
			LifterMotors.setTargetPositionInMillimeters(Target_Z_Pos);
			UI_Update = true;
			Auto_Display();
		}

		break;
	}
	case SetJogMM:
	{

		OldIncrement = Settings.Layer_Jog;

		SetKnob(OldIncrement);
		Setting = true;
		while (Setting)
		{

			CheckButtonStates();


			Settings.Layer_Jog = GetKnob();

			Increment_Display();

			if (Center_Button_ChangeState() == Pressed) // save
			{
				SaveSettings();
				Setting = false;
				break;
			}


			if (ModeChangeTest()) // cause a no save and menu change
			{
				Settings.Layer_Jog = OldIncrement;
				Setting = false;
				break;
			}

		}
		SetKnob(Target_Z_Pos);

		UI_Update = true;
		RunningMode = Manual;
		Manual_Display();
		break;
	}
	}
}

// ===================================================================================
// =							Button State Checks									 =
// ===================================================================================
boolean CheckButtonStates(void)
{
	Left_Button.update();
	Right_Button.update();
	Encoder_Press.update();
	Center_Button.update();
	External_Trigger.update();

	Left_Button_Now = (Left_Button.read() == LOW);
	Center_Button_Now = (Center_Button.read() == LOW);
	Right_Button_Now = (Right_Button.read() == LOW);
	Encoder_Button_Now = (Encoder_Press.read() == LOW);
	Trigger_Button_Now = (External_Trigger.read() == HIGH);


	Left_Button_State = Left_Button_Now ? Pressed : Released;
	Center_Button_State = Center_Button_Now ? Pressed : Released;
	Right_Button_State = Right_Button_Now ? Pressed : Released;
	Encoder_Button_State = Encoder_Button_Now ? Pressed : Released;
	Trigger_Button_State = Trigger_Button_Now ? Pressed : Released;

	return Left_Button_Now || Center_Button_Now || Right_Button_Now || Encoder_Button_Now || Trigger_Button_Now;
}
ButtonStates Left_Button_ChangeState(void)
{
	CheckButtonStates();
	ButtonStates retval;
	if (Left_Button_State != Last_Left_Button_State)
	{
		Last_Left_Button_State = Left_Button_State;
		retval = Left_Button_State;
	}
	else
	{
		Last_Left_Button_State = Left_Button_State;
		retval = Unchanged;
	}
	return retval;
}
ButtonStates Right_Button_ChangeState(void)
{
	CheckButtonStates();
	ButtonStates retval;
	if (Right_Button_State != Last_Right_Button_State)
	{
		Last_Right_Button_State = Right_Button_State;
		retval = Right_Button_State;
	}
	else
	{
		Last_Right_Button_State = Right_Button_State;
		retval = Unchanged;
	}
	return retval;
}
ButtonStates Center_Button_ChangeState(void)
{
	CheckButtonStates();
	ButtonStates retval;
	if (Center_Button_State != Last_Center_Button_State)
	{
		Last_Center_Button_State = Center_Button_State;
		retval = Center_Button_State;
	}
	else
	{
		Last_Center_Button_State = Center_Button_State;
		retval = Unchanged;
	}
	return retval;
}
ButtonStates Encoder_Button_ChangeState(void)
{
	CheckButtonStates();
	ButtonStates retval;
	if (Encoder_Button_State != Last_Encoder_Button_State)
	{
		Last_Encoder_Button_State = Encoder_Button_State;
		retval = Encoder_Button_State;
	}
	else
	{
		Last_Encoder_Button_State = Encoder_Button_State;
		retval = Unchanged;
	}
	return retval;
}
ButtonStates Trigger_Button_ChangeState(void)
{
	CheckButtonStates();
	ButtonStates retval;
	if (Trigger_Button_State != Last_Trigger_Button_State)
	{
		Last_Trigger_Button_State = Trigger_Button_State;
		retval = Trigger_Button_State;
	}
	else
	{
		Last_Trigger_Button_State = Trigger_Button_State;
		retval = Unchanged;
	}
	return retval;
}
boolean ModeChangeTest()
{
	if (Encoder_Button_ChangeState() == Pressed)
	{
		RunningMode++;
		RunningMode %= 3;
		UI_Update = true;

		switch (RunningMode)
		{
		case Manual:
		{
			Manual_Display();
			break;
		}
		case Auto:
		{
			Auto_Display();
			break;
		}
		case SetJogMM:
		{
			Increment_Display();
			break;
		}
		}
		return true;
	}
	return false;
}

// ===================================================================================
// =							Encoder value Wrapper 							     =
// ===================================================================================
void SetKnob(int32_t Value)
{
	Knob.write((Value * 4));
}
int32_t GetKnob(void)
{
	if (Knob.read() < 0)
	{
		Knob.write(0);
	}

	Soft_Limit = (Knob.read() / 4);
	if (Soft_Limit > Settings.MM_Soft_Limit)
	{
		Soft_Limit = Settings.MM_Soft_Limit;
		SetKnob(Soft_Limit);
	}

	return Soft_Limit;
}

// ===================================================================================
// =									Motor Functions								 =
// ===================================================================================
void DoHome(void)
{

	Homing_Display();

	if (LifterMotors.moveToHomeInMillimeters(2, Settings.MM_Speed, 5000, Bottom_Home_Pin))
	{
		SetKnob(0);
		Target_Z_Pos = 0;
	}

}
boolean MoveValid(void)
{
	StopHit = false;

	if (LifterMotors.getCurrentPositionInMillimeters() == Target_Z_Pos)
	{
		return false;
	}
	else
	{
		if (LifterMotors.getCurrentPositionInMillimeters() > Target_Z_Pos)
		{
			StopHit = (digitalRead(Bottom_Home_Pin) != LimitTriggered);
			return !StopHit;
		}
		else
		{
			StopHit = (digitalRead(Top_Home_Pin) != LimitTriggered);
			return !StopHit;
		}
	}
}
void MoveBed(void)
{
	while (MoveValid())
	{
		LifterMotors.processMovement();
		if (RunningMode == Manual)
		{
			Target_Z_Pos = GetKnob();
			if (MoveValid())
			{
				if (Target_Z_Pos != Display_Z_Pos)
				{
					UI_Update = true;
					Manual_Display();
					Display_Z_Pos = Target_Z_Pos;
				}
				LifterMotors.setTargetPositionInMillimeters(Target_Z_Pos);
			}
		}
	}

	if (StopHit)
	{
		Target_Z_Pos = LifterMotors.getCurrentPositionInMillimeters();
		LifterMotors.setCurrentPositionInMillimeters(Target_Z_Pos);
		SetKnob(Target_Z_Pos);
		UI_Update = true;
	}

}
void SetupMotorParameters()
{
	LifterMotors.setStepsPerMillimeter(Settings.MM_Steps);
	LifterMotors.setSpeedInMillimetersPerSecond(Settings.MM_Speed);
	LifterMotors.setAccelerationInMillimetersPerSecondPerSecond(Settings.Acceleration_Deceleration);
	LifterMotors.setInitialSpeedInMillimetersPerSecond(Settings.Acceleration_Threshold);
	LifterMotors.setFinalSpeedInMillimetersPerSecond(Settings.Acceleration_Threshold);
}

// ===================================================================================
// =								Screen Functions								 =
// ===================================================================================
void CenterText(int Height, const char* s)
{
	int Len = (strlen(s) * 6);
	int Left = ((100 - Len) / 2) + 15;
	u8g.drawStr(Left, Height, s);
}
void Config_Display()
{
	u8g.firstPage();
	do
	{
		u8g.setFont(u8g_font_courR08);
		u8g.drawRFrame(0, 0, u8g.getWidth(), u8g.getHeight(), 5);
		CenterText(10, "Configuration Mode");
		u8g.drawHLine(0, 14, u8g.getWidth());
		CenterText(28, "Press Application");
		CenterText(44, "Re-Start");
		u8g.drawHLine(0, 48, u8g.getWidth());
		CenterText(60, "Configuration Mode");
	} while (u8g.nextPage());
}
void Splash_Display(void)
{
	u8g.firstPage();
	do
	{
		u8g.setFont(u8g_font_courR08);
		u8g.drawRFrame(0, 0, u8g.getWidth(), u8g.getHeight(), 5);
		CenterText(10, "EmbeddedAT");
		u8g.drawHLine(0, 14, u8g.getWidth());
		CenterText(35, "Ortur LM2 Bed");
		u8g.drawHLine(0, 48, u8g.getWidth());
		CenterText(61, "Auto Adjust");
	} while (u8g.nextPage());
}
void Auto_Display(void)
{
	if (!UI_Update) return;
	UI_Update = false;

	u8g.firstPage();
	do
	{
		u8g.setFont(u8g_font_courR08);
		CenterText(10, "Auto");
		DynamicMenuPart();
		DrawNumbers();
	} while (u8g.nextPage());
}
void Manual_Display(void)
{
	if (!UI_Update) return;
	UI_Update = false;

	u8g.firstPage();
	do
	{
		u8g.setFont(u8g_font_courR08);
		CenterText(10, "Jog");
		CenterText(61, "-10    Home    +10");
		DynamicMenuPart();
		DrawNumbers();
	} while (u8g.nextPage());
}
void Set_Display(void)
{
	u8g.firstPage();
	do
	{
		u8g.setFont(u8g_font_courR08);
		u8g.drawRFrame(0, 0, u8g.getWidth(), u8g.getHeight(), 5);
		CenterText(10, "Preset Save");
		CenterText(38, "Choose Location");
		u8g.drawHLine(0, 48, u8g.getWidth());
		CenterText(61, "P1      Home      P2");
		DynamicMenuPart();
	} while (u8g.nextPage());

}
void Increment_Display(void)
{
	u8g.firstPage();
	do
	{
		u8g.setFont(u8g_font_courR08);
		CenterText(10, "Set");
		u8g.drawStr(3, 35, "Increment"); 
		u8g.drawStr(80, 35, itoa(Settings.Layer_Jog , buf, 10)); 
		CenterText(61, "Save");
		DynamicMenuPart();
	} while (u8g.nextPage());
}
void Homing_Display(void)
{
	u8g.firstPage();
	do
	{
		u8g.setFont(u8g_font_courR08);
		CenterText(10, "Homing");
		CenterText(35, "Please Wait");
		DynamicMenuPart();
	} while (u8g.nextPage());
}
void DynamicMenuPart(void)
{
	u8g.drawRFrame(0, 0, u8g.getWidth(), u8g.getHeight(), 5);
	u8g.drawHLine(0, 14, u8g.getWidth());

	u8g.drawCircle(9, 7, 4);
	u8g.drawCircle(118, 7, 4); // Down

	if ((digitalRead(Bottom_Home_Pin) != LimitTriggered))
	{
		u8g.drawDisc(9, 7, 5);
	}

	if ((digitalRead(Top_Home_Pin) != LimitTriggered))
	{
		u8g.drawDisc(118, 7, 5); // Down
	}

	u8g.drawHLine(0, 48, u8g.getWidth());
}
void DrawNumbers(void)
{
	u8g.drawStr(3, 28, "Current"); u8g.drawStr(80, 28, itoa(Target_Z_Pos, buf, 10));
	u8g.drawStr(3, 44, "Increment"); u8g.drawStr(80, 44, itoa(Settings.Layer_Jog, buf, 10));

}

// ===================================================================================
// =								Settings			  							 =
// ===================================================================================
void LoadSettings(void)
{
	eeprom_read_block((void*)&Settings, (void*)(sizeof(Settings)), sizeof(Settings));

	if (strstr(Settings.Valid, "Ab V1.0") == NULL)
	{
		Settings.Mem1 = Target_Z_Pos; // 0 = off
		Settings.Mem2 = Target_Z_Pos; // max power Output
		Settings.Layer_Jog = 1;
		Settings.Invert_Direction = false;
		Settings.MM_Soft_Limit = 300;
		Settings.Big_Jog = 10;
		Settings.Acceleration_Deceleration = 10;
		Settings.Acceleration_Threshold = 1;
		Settings.MM_Soft_Limit = 3000;
		Settings.MM_Steps = 100;
		Settings.MM_Speed = 5;
		SaveSettings();
	}
}
void SaveSettings(void)
{
	strcpy(Settings.Valid, "Ab V1.0");
	eeprom_write_block((const void*)&Settings, (void*)(sizeof(Settings)), sizeof(Settings));
}

// ===================================================================================
// =			Check out serial stream on the PC port								 =
// ===================================================================================
boolean CheckForPCCommands()
{
	boolean Retval = false;
	char inchar;
	if (PCHost.available() > 0)
	{
		while (PCHost.available() > 0)
		{
			inchar = PCHost.read();
			if ((int)inchar > 31 || (char)inchar == '\r' || (char)inchar == '\n')
			{
				if ((char)inchar == '\r' || (char)inchar == '\n')
				{
					if (CommandPointer > 0) ProcessPCResponseString();
					CommandPointer = 0;
					while (PCHost.available() > 0) PCHost.read();
					break;
				}
				else
				{
					if (CommandPointer < 50) CommandIn[CommandPointer++] = ((char)inchar);
				}
			}

			if (PCHost.available() == 0) delay(50);
		}
		Retval = true;
	}
	return Retval;
}

// ===================================================================================
// =							PC Command Processor					             =
// ===================================================================================
void ProcessPCResponseString()
{

	memset(PayloadValue, 0, 70);

	if (CommandPointer > 1)
	{
		for (uint8_t cLoop = 2; cLoop < CommandPointer; cLoop++)
		{
			PayloadValue[cLoop - 2] = CommandIn[cLoop];
		}
	}
	
	
	//PCHost.write(byte(ASCII_ENQ));

	switch (CommandIn[0])
	{

	case  'Q':
		SendConfigData();
		CommandPointer = 0;
		return;
		break;

	case  'I': // Direction Invert
		Settings.Invert_Direction = (ArrayToInt(PayloadValue) == 1);	// Float Device ID
		SaveSettings();
		break;

	case  'N': // Big Jog Value
		Settings.Big_Jog  = ArrayToInt(PayloadValue);	
		SaveSettings();
		break;

	case  'L': // Layer Jog 
		Settings.Layer_Jog = ArrayToInt(PayloadValue);
		SaveSettings();
		break;

	case 'S': // Soft Limit
			Settings.MM_Soft_Limit = ArrayToInt(PayloadValue);
			SaveSettings();
			break;

	case 'R': // Steps per mm
		Settings.MM_Steps = ArrayToInt(PayloadValue);
		SaveSettings();
		break;

	case 'V': // Speed mm per Second
		Settings.MM_Speed = ArrayToInt(PayloadValue);
		SaveSettings();
		break;

	case 'A': // Acceleration Up/Down
		Settings.Acceleration_Deceleration = ArrayToInt(PayloadValue);
		SaveSettings();
		break;

	case 'T': // Acceleration Threshold
		Settings.Acceleration_Threshold = ArrayToInt(PayloadValue);
		SaveSettings();
		break;

	}

	delay(500);
	PCHost.write((byte)ASCII_ETX);
	PCHost.flush();

}

void SendConfigData(void)
{
	PCHost.print(F("Q"));
	PCHost.print(F("I,"));
	PCHost.print(Settings.Invert_Direction);
	PCHost.print(F(";N,"));
	PCHost.print(Settings.Big_Jog);
	PCHost.print(F(";R,"));
	PCHost.print(Settings.MM_Steps);
	PCHost.print(F(";T,"));
	PCHost.print(Settings.Acceleration_Threshold);
	PCHost.print(F(";A,"));
	PCHost.print(Settings.Acceleration_Deceleration);
	PCHost.print(F(";V,"));
	PCHost.print(Settings.MM_Speed);
	PCHost.print(F(";L,"));
	PCHost.print(Settings.Layer_Jog);
	PCHost.print(F(";S,"));
	PCHost.print(Settings.MM_Soft_Limit);
	PCHost.print(F("\r\n"));
	PCHost.flush();
}

// ===================================================================================
// =					String formatting helper function				             =
// ===================================================================================
unsigned long ArrayToInt(char* Number)

{
	unsigned int i = 0;
	int aLen = 0;
	unsigned long  value = 0;

	while (Number[aLen++] != 0);

	for (i = 0; i < aLen; i++)
	{
		if (Number[i] >= (int)'0' && Number[i] <= (int)'9')
		{
			value = (10 * value) + ((byte)Number[i] - (int)'0');
		}
	}

	return value;
}

