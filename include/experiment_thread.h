#ifndef EXPERIMENT_THREAD_H
#define EXPERIMENT_THREAD_H

#include <QThread>
#include <QDebug>
#include "shared_data.h"

class Experiment_Thread : public QThread
{
    Q_OBJECT
public:
    explicit Experiment_Thread(QObject *parent = 0);
    ~Experiment_Thread();
    void initialize();

    shared_data *p_CommonData;

protected:
    void run();
};

#endif // EXPERIMENT_THREAD_H
