#ifndef MAILWORKER_H
#define MAILWORKER_H
#include <QObject>

#include <sstream>
#include <iomanip>
#include <stdlib.h> // required for system call


class MailWorker: public QObject
{
    Q_OBJECT


public:
    explicit MailWorker(QObject *parent = nullptr);
    ~MailWorker();

signals:

public slots:
    void getFromMain1(QString);

private:


};

#endif // MAILWORKER_H
