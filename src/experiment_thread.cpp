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
        switch(p_CommonData->currentExperimentState)
        {

        case idleExperiment:
            break;

        case trial:
            p_CommonData->recordFlag = true;
            p_CommonData->referenceFriction = std::stod(p_CommonData->protocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "Reference", NULL /*default*/));
            p_CommonData->comparisonFriction = std::stod(p_CommonData->protocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "Comparisons", NULL /*default*/));
            p_CommonData->referenceFirst = atoi(p_CommonData->protocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "ReferenceFirst", NULL /*default*/));
            p_CommonData->tactileFeedback = atoi(p_CommonData->protocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "SkinStretch", NULL /*default*/));

            if(p_CommonData->referenceFirst)
            {
                if(p_CommonData->pairNo == 1)
                {
                    p_CommonData->p_expFrictionBox->m_material->setStaticFriction(p_CommonData->referenceFriction);
                    p_CommonData->p_expFrictionBox->m_material->setDynamicFriction(p_CommonData->referenceFriction);
                }
                else if(p_CommonData->pairNo == 2)
                {
                    p_CommonData->p_expFrictionBox->m_material->setStaticFriction(p_CommonData->comparisonFriction);
                    p_CommonData->p_expFrictionBox->m_material->setDynamicFriction(p_CommonData->comparisonFriction);
                }
            }
            else if(!p_CommonData->referenceFirst)
            {
                if(p_CommonData->pairNo == 2)
                {
                    p_CommonData->p_expFrictionBox->m_material->setStaticFriction(p_CommonData->referenceFriction);
                    p_CommonData->p_expFrictionBox->m_material->setDynamicFriction(p_CommonData->referenceFriction);
                }
                else if(p_CommonData->pairNo == 1)
                {
                    p_CommonData->p_expFrictionBox->m_material->setStaticFriction(p_CommonData->comparisonFriction);
                    p_CommonData->p_expFrictionBox->m_material->setDynamicFriction(p_CommonData->comparisonFriction);
                }
            }
            break;

        case trialBreak:
            break;
        }
    }
}
