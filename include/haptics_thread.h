#ifndef HAPTICS_THREAD_H
#define HAPTICS_THREAD_H

#include <QObject>
#include <QThread>
#include "shared_data.h"

class haptics_thread : public QThread
{
    Q_OBJECT


public:
    // Public Functions ============================================
    explicit haptics_thread(QObject *parent = 0);
    ~haptics_thread();
    void initialize();

    // Commands a position to the Mechanism
    void PositionController(Eigen::Vector3d);


    // Public Variables ============================================
    shared_data* p_CommonData; //create a pointer to a shared_data struct



protected:
    void run();

};

#endif // HAPTICS_THREAD_H
