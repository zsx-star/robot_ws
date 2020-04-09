#ifndef PERFORMANCERECORDER_H
#define PERFORMANCERECORDER_H
#include<QString>
#include<QSettings>

class PerformanceRecorder
{
public:
    PerformanceRecorder(QString file);
    PerformanceRecorder() = delete;
    void writePerformance(const QString& key, const QString& value, QString group="");
    void readPerformance(const QString&  key, QString& value,  QString group="");

private:
    QString file_;

};

#endif // PERFORMANCERECORDER_H
