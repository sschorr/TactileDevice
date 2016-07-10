//===========================================================================
/*
    This file controls defines a class "MotorControl" that interfaces
    with the Sensoray 626 to allow control of a motor with encoder.
*/
//===========================================================================
#include "shared_data.h"


// Constructor of motor controller =========================================
cMotorController::cMotorController(int inputMotorID)
{
    motorNum = inputMotorID;
    channelNum = MotorNumToChannelNum(motorNum);
}

// Init 626 encoder tracking and reading ============================
void cMotorController::InitEncoder()
{
#ifdef SENSORAY826
    int fail = S826_CounterModeWrite(PCI_BOARD, channelNum, MODE_ENC);
    if (fail == 0)
       qDebug() << "encoder" << channelNum << "init'ed";
    S826_CounterStateWrite(PCI_BOARD, channelNum, 1); //sets the counter to start keeping track
#endif

    // Set initial condition of offset angle until funciton is called
    offsetAngle = 0;
}

void cMotorController::SetOffsetAngle()
{
#ifdef SENSORAY826
    uint EncoderRaw;
    S826_CounterSnapshot(PCI_BOARD, channelNum);
    S826_CounterSnapshotRead(PCI_BOARD, channelNum, &EncoderRaw, nullptr, nullptr, T_WAIT);

    int EncoderPos = 0;
    if (EncoderRaw <= (MAXCOUNT/2))
        EncoderPos = EncoderRaw;
    else if (EncoderRaw >= (MAXCOUNT/2))
        EncoderPos = -(MAXCOUNT)+ EncoderRaw;

    offsetAngle = ENCCOUNT_TO_RAD*EncoderPos;

#endif

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


// Get angle of motor ======================================================
double cMotorController::GetMotorAngle()
{
    double motorAngle = 0;

#ifdef SENSORAY826
    uint EncoderRaw;
    S826_CounterSnapshot(PCI_BOARD, channelNum);
    S826_CounterSnapshotRead(PCI_BOARD, channelNum, &EncoderRaw, nullptr, nullptr, T_WAIT);

    int EncoderPos = 0;
    if (EncoderRaw <= (MAXCOUNT/2))
        EncoderPos = EncoderRaw;
    else if (EncoderRaw > (MAXCOUNT/2))
        EncoderPos = -(MAXCOUNT)+ EncoderRaw;

    double rawMotorAngle = ENCCOUNT_TO_RAD*EncoderPos;
    motorAngle = -(rawMotorAngle - offsetAngle);

#endif

#ifdef SENSORAY626
    //Lock before we access Sensorarray stuff
    //m_mutex.lock();
    unsigned long EncoderRaw = S626_CounterReadLatch(0, channelNum);
    //m_mutex.unlock();
    // Change the raw encoder count into a double centered around 0
    double EncoderPos = 0;
    if (EncoderRaw < ((MAX_COUNT+1)/2))
        EncoderPos = EncoderRaw;
    else if (EncoderRaw >= ((MAX_COUNT+1)/2))
        EncoderPos = -MAX_COUNT+1+(double)EncoderRaw;

    double rawMotorAngle = ENCCOUNT_TO_RAD*EncoderPos;
    motorAngle = -(rawMotorAngle - offsetAngle);
#endif
    return motorAngle;
}

// Destructor of motor controller ================================
cMotorController::~cMotorController()
{
    close();
}

// Write all outputs to 0 and then close 626 board =========================
int cMotorController::close()
{
#ifdef SENSORAY826
    S826_SystemClose();
#endif

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



void cMotorController::SetOutputTorque(double desiredTorque)
{
    double VoltOut = 0;
    // we need to limit the max voltage output to a corresponding max amperage;
    double MaxVolt = MAX_AMPS*AMPS_TO_VOLTS; //0.92

    double desiredAmps = desiredTorque/KT;
    VoltOut = desiredAmps*AMPS_TO_VOLTS;

    // Write Voltage out to DAC with software limits checked
    if (VoltOut > MaxVolt)
    {
        VoltOut = MaxVolt;
    }
    else if (VoltOut < (-MaxVolt))
    {
        VoltOut = -MaxVolt;
    }

    long writeData = (long)(VoltOut*DAC_VSCALAR);

#ifdef SENSORAY626
    S626_WriteDAC(0,channelNum,writeData);
#endif

    //Setting for GUI
    this->voltageOutput = VoltOut;
}


