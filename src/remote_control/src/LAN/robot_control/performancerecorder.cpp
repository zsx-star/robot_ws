#include "performancerecorder.h"

PerformanceRecorder::PerformanceRecorder(QString file)
{
    file_ = file;
}

void PerformanceRecorder::writePerformance(const QString& key, const QString& value, QString group)
{
    //创建配置文件操作对象
    QSettings *config = new QSettings(file_, QSettings::IniFormat);

    //将信息写入配置文件
    config->beginGroup(group);
    config->setValue(key, value);
    config->endGroup();
    delete config;
}

void PerformanceRecorder::readPerformance(const QString &key, QString &value, QString group)
{
    //创建配置文件操作对象
    QSettings *config = new QSettings(file_, QSettings::IniFormat);

    //读取配置信息
    value = config->value(group + "/" + key).toString();
    delete config;
}



