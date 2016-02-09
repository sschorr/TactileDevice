#include "magtracker.h"

magTracker::magTracker()
{
}

magTracker::~magTracker()
{
}

void magTracker::InitMagTracker(){
    // variable declarations
    pRecord0 = &record0;
    pRecord1 = &record1;

    // initialize the magnetic tracker
    qDebug("Initializing the ATC3DG system...\n");
    errorCode = InitializeBIRDSystem();
    if(errorCode==BIRD_ERROR_SUCCESS){
        qDebug("Initialized ATC3DG system\n");
    }

    // get configurations
    errorCode = GetBIRDSystemConfiguration(&ATC3DG.m_config);
    pSensor = new CSensor[ATC3DG.m_config.numberSensors];
    for(i=0;i<ATC3DG.m_config.numberSensors;i++)
    {
        errorCode = GetSensorConfiguration(i, &pSensor[i].m_config);
        if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
        qDebug("Got sensors configuration\n");
    }
    pXmtr = new CXmtr[ATC3DG.m_config.numberTransmitters];
    for(i=0;i<ATC3DG.m_config.numberTransmitters;i++)
    {
        errorCode = GetTransmitterConfiguration(i, &pXmtr[i].m_config);
        if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
        qDebug("Got transmitters configuration\n");
    }

    measFreq = 80.0;
    // set parameters for recording
    SET_SYSTEM_PARAMETER(SELECT_TRANSMITTER,	0);
    SET_SYSTEM_PARAMETER(POWER_LINE_FREQUENCY,	60.0);
    SET_SYSTEM_PARAMETER(AGC_MODE,				SENSOR_AGC_ONLY);
    SET_SYSTEM_PARAMETER(MEASUREMENT_RATE,		measFreq);
    SET_SYSTEM_PARAMETER(MAXIMUM_RANGE,			72.0);
    SET_SYSTEM_PARAMETER(METRIC,				true);

    for(sensorID=0;sensorID<ATC3DG.m_config.numberSensors;sensorID++){
        SET_SENSOR_PARAMETER(sensorID, DATA_FORMAT, DOUBLE_POSITION_MATRIX_TIME_Q_BUTTON);
        {
            // initialize a structure of angles
            DOUBLE_ANGLES_RECORD anglesRecord = {0, 0, 0};
            SET_SENSOR_PARAMETER(sensorID, ANGLE_ALIGN, anglesRecord);
        }
        SET_SENSOR_PARAMETER(sensorID, HEMISPHERE, FRONT);
        SET_SENSOR_PARAMETER(sensorID, FILTER_AC_WIDE_NOTCH, false);
        SET_SENSOR_PARAMETER(sensorID, FILTER_AC_NARROW_NOTCH, false);
        SET_SENSOR_PARAMETER(sensorID, FILTER_DC_ADAPTIVE, 0.0);
    }

    transmitterID = 0;
    {
        // initialize a structure of angles
        DOUBLE_ANGLES_RECORD anglesRecord = {0, 0, 0};
        SET_TRANSMITTER_PARAMETER(transmitterID, REFERENCE_FRAME, anglesRecord);
    }
    SET_TRANSMITTER_PARAMETER(transmitterID, XYZ_REFERENCE_FRAME, false);

}

chai3d::cTransform magTracker::CheckPose(int trackerNum){
    chai3d::cTransform returnTransform;
    chai3d::cVector3d returnVec;
    chai3d::cMatrix3d returnMatrix;
    double x,y,z;

    if (trackerNum == 0)
    {
        // test the reading of the magnetic tracker
        errorCode = GetAsynchronousRecord(trackerNum, pRecord0, sizeof(record0));
        if(errorCode!=BIRD_ERROR_SUCCESS) {errorHandler(errorCode);}
        // get the status of the last data record
        // only report the data if everything is okay
        x = (record0.x - 200)/1000.0;
        y = record0.y/1000.0;
        z = record0.z/1000.0;
        returnVec.set(x, y, z);

        returnMatrix.set(record0.s[0][0], record0.s[0][1], record0.s[0][2],
                         record0.s[1][0], record0.s[1][1], record0.s[1][2],
                         record0.s[2][0], record0.s[2][1], record0.s[2][2]);

        returnTransform.set(returnVec, returnMatrix);
        return returnTransform;
    }
    if (trackerNum == 1)
    {
        // test the reading of the magnetic tracker
        errorCode = GetAsynchronousRecord(trackerNum, pRecord1, sizeof(record1));
        if(errorCode!=BIRD_ERROR_SUCCESS) {errorHandler(errorCode);}
        // get the status of the last data record
        // only report the data if everything is okay
        x = (record1.x - 200)/1000.0;
        y = record1.y/1000.0;
        z = record1.z/1000.0;
        returnVec.set(x, y, z);

        returnMatrix.set(record1.s[0][0], record1.s[0][1], record1.s[0][2],
                         record1.s[1][0], record1.s[1][1], record1.s[1][2],
                         record1.s[2][0], record1.s[2][1], record1.s[2][2]);

        returnTransform.set(returnVec, returnMatrix);
        return returnTransform;
    }
}

chai3d::cVector3d magTracker::CheckPos(int trackerNum){

    chai3d::cVector3d returnVec;
    double x,y,z;

    if (trackerNum == 0)
    {
        // test the reading of the magnetic tracker
        errorCode = GetAsynchronousRecord(trackerNum, pRecord0, sizeof(record0));
        if(errorCode!=BIRD_ERROR_SUCCESS) {errorHandler(errorCode);}
        // get the status of the last data record
        // only report the data if everything is okay
        x = (record0.x - 200)/1000.0;
        y = record0.y/1000.0;
        z = record0.z/1000.0;
        returnVec.set(x, y, z);
        return returnVec;
    }
    if (trackerNum == 1)
    {
        // test the reading of the magnetic tracker
        errorCode = GetAsynchronousRecord(trackerNum, pRecord1, sizeof(record1));
        if(errorCode!=BIRD_ERROR_SUCCESS) {errorHandler(errorCode);}
        // get the status of the last data record
        // only report the data if everything is okay
        x = (record1.x - 200)/1000.0;
        y = record1.y/1000.0;
        z = record1.z/1000.0;
        returnVec.set(x, y, z);
        return returnVec;
    }
}

chai3d::cMatrix3d magTracker::CheckRot(int trackerNum){
    chai3d::cMatrix3d returnMatrix;

    if (trackerNum == 0)
    {
        returnMatrix.set(record0.s[0][0], record0.s[0][1], record0.s[0][2],
                         record0.s[1][0], record0.s[1][1], record0.s[1][2],
                         record0.s[2][0], record0.s[2][1], record0.s[2][2]);
    }

    if (trackerNum == 1)
    {
        returnMatrix.set(record1.s[0][0], record1.s[0][1], record1.s[0][2],
                         record1.s[1][0], record1.s[1][1], record1.s[1][2],
                         record1.s[2][0], record1.s[2][1], record1.s[2][2]);
    }
    return returnMatrix;
}
