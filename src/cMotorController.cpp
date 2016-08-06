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
    channelNum = inputMotorID; //now just indexing motors directly
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

void cMotorController::InitDACOut()
{
#ifdef SENSORAY826
    int fail = S826_DacRangeWrite(PCI_BOARD, channelNum, VOLTRANGE, 0);
    if (fail == 0)
       qDebug() << "DAC" << channelNum << "init'ed";
#endif

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
    double Vmax; double Vmin;
    // determine range of DAC
    if (VOLTRANGE == 2)
    {
        Vmax = 5;
        Vmin = -5;
    } else {
        Vmax = 10;
        Vmin = -10;
    }
    uint setPoint = (0.0-Vmin)/(Vmax-Vmin) * MAXSETPNT;
    S826_DacDataWrite(PCI_BOARD, channelNum, setPoint, 0);
    S826_CounterStateWrite(PCI_BOARD, channelNum, 0);
#endif
    return 0;
}

void cMotorController::SetOutputTorque(double desiredTorque)
{
    double Vmax; //range of DAC
    double Vmin; //range of DAC

    double VoltOut = 0;
    double MaxVolt = MAX_AMPS*AMPS_TO_VOLTS; //0.96 //we need to limit the max voltage output to a corresponding max amperage;

    double desiredAmps = desiredTorque/KT;
    VoltOut = desiredAmps*AMPS_TO_VOLTS;

    // Check to make sure we're not assigning more than we want
    if (VoltOut > MaxVolt)
    {
        VoltOut = MaxVolt;
    }
    else if (VoltOut < (-MaxVolt))
    {
        VoltOut = -MaxVolt;
    }
    // determine range of DAC
    if (VOLTRANGE == 2)
    {
        Vmax = 5;
        Vmin = -5;
    } else {
        Vmax = 10;
        Vmin = -10;
    }
    uint setPoint = (VoltOut-Vmin)/(Vmax-Vmin) * MAXSETPNT;

#ifdef SENSORAY826
    S826_DacDataWrite(PCI_BOARD, channelNum, setPoint, 0);
#endif

    this->voltageOutput = VoltOut; //Store for GUI

}


