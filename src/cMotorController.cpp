//===========================================================================
/*
    This file controls defines a class "MotorControl" that interfaces
    with the Sensoray 626 to allow control of a motor with encoder.
*/
//===========================================================================
#include "shared_data.h"
#include "cMotorController.h"
#include "windows.h"

// DEFINES =================================================================

// Define used for max Sensoray Encoder Count
#define MAX_COUNT 16777215

// Define used for DAC Voltage Conversion
#define DAC_VSCALAR 819.1

// Define the encoder counts per revolution
#define ENCODER_CPR 360 // (12*30 for Pololu motor)

// Define used for Encoder count to angle in radian
#define ENCCOUNT_TO_RAD 2*3.1415926535897932384/(ENCODER_CPR*4.0) // 4 comes from edges counts per quadrature cycle

// Define the torque constant torque = I*KT
#define KT 78.44 //mNm/A

// Define the amplifier output to voltage input ratio I = I_RATIO*V
#define AMPS_TO_VOLTS 10 //Amps*10 = # Volts, Amps = Volts/10

// Define the max amperage of the motor
#define MAX_AMPS .25




// Constructor of motor controller =========================================
cMotorController::cMotorController(int inputMotorID)
{
    motorNum = inputMotorID;
    channelNum = MotorNumToChannelNum(motorNum);
}




// Get angle of motor ======================================================
double cMotorController::GetMotorAngle()
{
    double motorAngle = 0;
#ifdef SENSORAY626
    //Lock before we access Sensorarray stuff
    m_mutex.lock();
    unsigned long EncoderRaw = S626_CounterReadLatch(0, channelNum);
    m_mutex.unlock();
    // Change the raw encoder count into a double centered around 0
    double EncoderPos = 0;
    if (EncoderRaw < ((MAX_COUNT+1)/2))
        EncoderPos = EncoderRaw;
    else if (EncoderRaw >= ((MAX_COUNT+1)/2))
        EncoderPos = -MAX_COUNT+1+(double)EncoderRaw;

    double rawMotorAngle = ENCCOUNT_TO_RAD*EncoderPos;
    motorAngle = rawMotorAngle - offsetAngle;
#endif
    return motorAngle;
}

// Destructor of motor controller ================================
cMotorController::~cMotorController()
{
    close();
}


// Open connection to 626 Board
int cMotorController::open()
{
#ifdef SENSORAY626
    // load the .dll
    S626_DLLOpen();

    // open the 626 card
    S626_OpenBoard(0,0,0,0);

    if (S626_GetErrors(0) != 0)
        return S626_GetErrors(0);
#endif
    return 0;
}

// Write all outputs to 0 and then close 626 board =========================
int cMotorController::close()
{
#ifdef SENSORAY626
    ////////////////////////////////////////////////////////////////////////////
    // Unregister board number 0 after programming analog and digital outputs
    // to their application-defined "shutdown" states.
    ////////////////////////////////////////////////////////////////////////////
    // Program all analog outputs to zero volts.
    for ( WORD DacChan = 0; DacChan < 4; S626_WriteDAC( 0, DacChan++, 0 ));
    // Program all digital outputs to the inactive state.
    for ( WORD Group = 0; Group < 3; S626_DIOWriteBankSet( 0, Group++, 0 ));
    // Unregister the board and release its handle.
    S626_CloseBoard( 0 );

    // close the DLL
    S626_DLLClose();
#endif
    return 0;
}



// Convert the motor number to the encoder number that corresponds======================
int cMotorController::MotorNumToChannelNum(int motorNumArg)
{
    int counterNumRet;
    switch(motorNumArg){
    case 1: counterNumRet = 0;
        break;
    case 2: counterNumRet = 1;
        break;
    case 3: counterNumRet = 2;
        break;
    }
    return counterNumRet;
}

// Init 626 encoder tracking and reading ============================
void cMotorController::InitEncoder()
{
#ifdef SENSORAY626
    open();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Configure board 0, counter counterNum as a quadrature counter. An active Index will reset
    // the counter core to zero and generate an interrupt request.
    ///////////////////////////////////////////////////////////////////////////////////////////

    // Set counter operating mode.
    S626_CounterModeSet( 0, channelNum,
    //( LOADSRC_INDX << BF_LOADSRC ) | // Index causes preload.
    ( INDXSRC_SOFT << BF_INDXSRC ) | // Hardware index is disabled
    ( INDXPOL_POS << BF_INDXPOL ) | // Active high index.
    ( CLKSRC_COUNTER << BF_CLKSRC ) | // Operating mode is Counter.
    ( CLKPOL_POS << BF_CLKPOL ) | // Active high clock.
    ( CLKMULT_4X << BF_CLKMULT ) | // Clock multiplier is 4x.
    ( CLKENAB_ALWAYS << BF_CLKENAB ) ); // Counting is always enabled.

    // Initialize preload value to zero so that the counter core will be set
    // to zero upon the occurance of an Index.
    S626_CounterPreload( 0, channelNum, 0 );

    // Enable latching of accumulated counts on demand. This assumes that
    // there is no conflict with the latch source used by paired counter 2B.
    S626_CounterLatchSourceSet( 0, channelNum, LATCHSRC_AB_READ );

    // Enable the counter to generate interrupt requests upon index.
    S626_CounterIntSourceSet( 0, channelNum, INTSRC_INDX );

    // Simulate Index Pulse to reset count at 0 on startup
    S626_CounterSoftIndex(0, channelNum);
#endif

    // Set initial condition of offset angle until funciton is called
    offsetAngle = 0;
}

void cMotorController::SetOffsetAngle()
{
#ifdef SENSORAY626
    //Lock before we access Sensorarray stuff
    m_mutex.lock();
    unsigned long EncoderRaw = S626_CounterReadLatch(0, channelNum);
    m_mutex.unlock();
    // Change the raw encoder count into a double centered around 0
    double EncoderPos = 0;
    if (EncoderRaw < ((MAX_COUNT+1)/2))
        EncoderPos = EncoderRaw;
    else if (EncoderRaw >= ((MAX_COUNT+1)/2))
        EncoderPos = -MAX_COUNT+1+(double)EncoderRaw;

     offsetAngle = ENCCOUNT_TO_RAD*EncoderPos;
#endif
}

void cMotorController::SetOutputTorque(double desiredTorque)
{
    double VoltOut = 0;
    // Set Maximum Voltage output to allow maximum amperage limit
    double MaxVolt = MAX_AMPS*AMPS_TO_VOLTS;

    double desiredAmps = desiredTorque/KT;
    VoltOut = desiredAmps*AMPS_TO_VOLTS;

    // Write Voltage out to DAC 0 with software limits checked
    if ( VoltOut > MaxVolt ) VoltOut = MaxVolt;
    else if ( VoltOut < -MaxVolt ) VoltOut = -MaxVolt;

#ifdef SENSORAY626
    S626_WriteDAC(0,channelNum,VoltOut*DAC_VSCALAR);
#endif

    this->voltageOutput = VoltOut;
}


