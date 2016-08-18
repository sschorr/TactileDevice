#include "experiment_thread.h"

Experiment_Thread::Experiment_Thread(QObject *parent) :
    QThread(parent)
{
}

Experiment_Thread::~Experiment_Thread()
{
}

void Experiment_Thread::initialize()
{
    p_CommonData->currentEnvironmentState = none;
    p_CommonData->currentExperimentState = idleExperiment;
    p_CommonData->pairNo = 1;
    p_CommonData->trialNo = 1;
}

void Experiment_Thread::run()
{
    forever
    {
        Sleep(50);
        switch(p_CommonData->currentExperimentState)
        {

        case idleExperiment:
            break;

        case palpationTrial:
            break;

        case palpationLineTrial:
            p_CommonData->recordFlag = true;
            break;

        case palpationLineWritingToFile:
            break;

        case palpationLineBreak:
            break;
        case sizeWeightTrial:
            //p_CommonData->recordFlag = true;
            break;

        case frictionTrial:
            p_CommonData->referenceFriction = std::stod(p_CommonData->frictionProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "Reference", NULL /*default*/));
            p_CommonData->comparisonFriction = std::stod(p_CommonData->frictionProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "Comparisons", NULL /*default*/));
            p_CommonData->referenceFirst = atoi(p_CommonData->frictionProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "ReferenceFirst", NULL /*default*/));
            p_CommonData->tactileFeedback = atoi(p_CommonData->frictionProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "SkinStretch", NULL /*default*/));

            if(p_CommonData->referenceFirst)
            {
                if(p_CommonData->pairNo == 1)
                {
                    p_CommonData->p_expFrictionBox->m_material->setStaticFriction(p_CommonData->referenceFriction);
                    p_CommonData->p_expFrictionBox->m_material->setDynamicFriction(p_CommonData->referenceFriction*0.9);
                }
                else if(p_CommonData->pairNo == 2)
                {
                    p_CommonData->p_expFrictionBox->m_material->setStaticFriction(p_CommonData->comparisonFriction);
                    p_CommonData->p_expFrictionBox->m_material->setDynamicFriction(p_CommonData->comparisonFriction*0.9);
                }
            }
            else if(!p_CommonData->referenceFirst)
            {
                if(p_CommonData->pairNo == 2)
                {
                    p_CommonData->p_expFrictionBox->m_material->setStaticFriction(p_CommonData->referenceFriction);
                    p_CommonData->p_expFrictionBox->m_material->setDynamicFriction(p_CommonData->referenceFriction*0.9);
                }
                else if(p_CommonData->pairNo == 1)
                {
                    p_CommonData->p_expFrictionBox->m_material->setStaticFriction(p_CommonData->comparisonFriction);
                    p_CommonData->p_expFrictionBox->m_material->setDynamicFriction(p_CommonData->comparisonFriction*0.9);
                }
            }
            p_CommonData->recordFlag = true;
            break;

        case trialBreak:
            p_CommonData->recordFlag = false;
            break;
        }
    }
}
