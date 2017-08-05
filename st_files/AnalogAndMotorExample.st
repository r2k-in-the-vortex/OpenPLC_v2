PROGRAM My_Program
  VAR
  	lamp0			AT %QX0.0	: BOOL;
  	lamp1			AT %QX0.1	: BOOL;
  	lamp2			AT %QX0.2	: BOOL;
  	lamp3			AT %QX0.3	: BOOL;
  	
    mReadyToEnable	AT %IX0.0	: BOOL;
    mReady			AT %IX0.1	: BOOL;
    mWarning		AT %IX0.2	: BOOL;
    mError			AT %IX0.3	: BOOL;
    mSyncError		AT %IX0.4	: BOOL;
    
    button			AT %IX0.5	: BOOL;
    
    mEnable			AT %QX0.4	: BOOL;
    mReset			AT %QX0.5	: BOOL;
    
    mVelocity		AT %QW1		: UINT;
    
    analog1			AT %IW1		: UINT;
  END_VAR
  VAR
  	voltage			: REAL;
  	direction		: BOOL;
  	buttonpulse		: R_TRIG;
  END_VAR
	buttonpulse.CLK := button;
	buttonpulse();
	
	if buttonpulse.Q then
		direction := NOT direction;
	end_if;


	voltage := (10.0/32767.0)*UINT_TO_REAL(analog1);

	lamp0	:= direction;
	lamp1	:= NOT direction;
	lamp2	:= mError;
	lamp3	:= button;
  
    mReset	:= mError OR mWarning;
    mEnable	:= mReadyToEnable;
    
	if mReady then
		if direction then
			mVelocity := WORD_TO_UINT(INT_TO_WORD(REAL_TO_INT(100.0*voltage)));
		else
			mVelocity := WORD_TO_UINT(INT_TO_WORD(REAL_TO_INT(-100.0*voltage)));
		end_if;
	else
		mVelocity := 0; 
	end_if;
END_PROGRAM


CONFIGURATION Config0

  RESOURCE Res0 ON PLC
    TASK TaskMain(INTERVAL := T#50ms,PRIORITY := 0);
    PROGRAM Inst0 WITH TaskMain : My_Program;
  END_RESOURCE
END_CONFIGURATION
