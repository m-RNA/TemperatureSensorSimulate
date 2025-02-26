#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <time.h>
#include <QFile>
#include <QSettings>
#include <QTextCodec>

double MainWindow::randomGauss(double stddev = 0.1, double average = 0.0)
{
    std::normal_distribution<double> ans(average, stddev);
    return ans(generator);
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    loadSettings();

    current = ui->label_ADC->text().toDouble();
    pid = new PID(ui->doubleSpinBox_P->value(), ui->doubleSpinBox_I->value(), ui->doubleSpinBox_D->value(), ui->spinBox_Limit->value());

    serial_Std = serialInit(serial_Std, "COM2");
    serial_Dtm = serialInit(serial_Dtm, "COM4");
    if (serial_Std->isOpen())
        ui->label_Std->setText("标准：已连接");
    if (serial_Dtm->isOpen())
        ui->label_Dtm->setText("待定：已连接");
    qDebug() << serial_Std;
    qDebug() << serial_Dtm;

    // 每100ms模拟一次水温变化
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::simulate);
    timer->start(ui->spinBox_Period->value());

    generator = std::default_random_engine(time(0));
}

MainWindow::~MainWindow()
{
    saveSettings();

    serial_Std->close();
    serial_Dtm->close();

    delete ui;
    delete pid;
}

void MainWindow::loadSettings()
{
    if (QFile::exists("config.ini") == false)
        return;

    QSettings settings("config.ini", QSettings::IniFormat);
    // settings.setIniCodec(QTextCodec::codecForName("GB18030"));

    settings.beginGroup("MainWindow");
    ui->label_Temperature->setText(settings.value("Temperature", "25").toString());
    ui->label_ADC->setText(settings.value("ADC", "4879210").toString());

    ui->cbAnalyse_Std->setCurrentIndex(settings.value("Analyse_Std", 1).toInt());
    ui->cbAnalyse_Dtm->setCurrentIndex(settings.value("Analyse_Dtm", 0).toInt());

    ui->spinBox_Period->setValue(settings.value("Period", 50).toInt());
    ui->doubleSpinBox_P->setValue(settings.value("P", 0.08).toDouble());
    ui->doubleSpinBox_I->setValue(settings.value("I", 0).toDouble());
    ui->doubleSpinBox_D->setValue(settings.value("D", 0.1).toDouble());
    ui->spinBox_Limit->setValue(settings.value("Limit", 1000).toInt());
    ui->comboBox->setCurrentIndex(settings.value("TargetIndex", 2).toInt());
    ui->spinBox_Target->setValue(settings.value("Target", 4879210).toInt());
    ui->spinBox_WaveTemperature->setValue(settings.value("WaveTemperature", 0.008).toDouble());
    ui->spinBox_WaveADC->setValue(settings.value("WaveADC", 200).toDouble());
    settings.endGroup();
}

void MainWindow::saveSettings()
{
    QSettings settings("config.ini", QSettings::IniFormat);
    // settings.setIniCodec(QTextCodec::codecForName("GB18030"));

    settings.beginGroup("MainWindow");
    settings.setValue("Temperature", ui->label_Temperature->text());
    settings.setValue("ADC", ui->label_ADC->text());

    settings.setValue("Analyse_Std", ui->cbAnalyse_Std->currentIndex());
    settings.setValue("Analyse_Dtm", ui->cbAnalyse_Dtm->currentIndex());

    settings.setValue("Period", ui->spinBox_Period->value());
    settings.setValue("P", ui->doubleSpinBox_P->value());
    settings.setValue("I", ui->doubleSpinBox_I->value());
    settings.setValue("D", ui->doubleSpinBox_D->value());
    settings.setValue("Limit", ui->spinBox_Limit->value());
    settings.setValue("TargetIndex", ui->comboBox->currentIndex());
    settings.setValue("Target", ui->spinBox_Target->value());
    settings.setValue("WaveTemperature", ui->spinBox_WaveTemperature->value());
    settings.setValue("WaveADC", ui->spinBox_WaveADC->value());
    settings.endGroup();
}

QSerialPort *MainWindow::serialInit(QSerialPort *sp, const QString &portName)
{
    // 初始化串口
    sp = new QSerialPort(this);
    // 设置串口名
    sp->setPortName(portName);
    // 设置波特率
    sp->setBaudRate(QSerialPort::Baud115200);
    // 设置数据位数
    sp->setDataBits(QSerialPort::Data8);
    // 设置奇偶校验
    sp->setParity(QSerialPort::NoParity);
    // 设置停止位
    sp->setStopBits(QSerialPort::OneStop);
    // 设置流控制
    sp->setFlowControl(QSerialPort::NoFlowControl);
    // 打开串口
    sp->open(QIODevice::ReadWrite);

    return sp;
}

// 该程序用于模拟深海温度的变化
PID::PID(double p, double i, double d, double limit)
{
    this->p = p;
    this->i = i;
    this->d = d;
    this->limit = limit;
    lastError = 0;
    integral = 0;
}

void PID::setP(double p)
{
    this->p = p;
}

void PID::setI(double i)
{
    this->i = i;
}

void PID::setD(double d)
{
    this->d = d;
}

void PID::setLimit(double l)
{
    limit = l;
}

double PID::update(double target, double current)
{
    double error = target - current;
    integral += error;
    double derivative = error - lastError;
    lastError = error;

    double temp = p * error + i * integral + d * derivative;
    if (temp > limit)
        temp = limit;
    else if (temp < -limit)
        temp = -limit;
    return temp;
}

double ADC2Temp(double adc)
{
    return adc * adc * adc * 6.54944412E-18 - adc * adc * 0.0000000000927421342 + adc * 0.000505346901 - 993.579977;
}

void MainWindow::simulate()
{
    double delta = pid->update(ui->spinBox_Target->value(), current);
    current += delta;
    double temperature = ADC2Temp(current);
    QString strTemp = QString::number(temperature);
    QString strCurrent = QString::number(current, 'f', 0);

    ui->label_Temperature->setText(strTemp);
    ui->label_ADC->setText(strCurrent);

    double noiseTemp = ui->spinBox_WaveTemperature->value() * randomGauss();
    double noiseCurrent = ui->spinBox_WaveADC->value() * randomGauss();
    // qDebug() << "1 " << noiseTemp;
    // qDebug() << "2 " << noiseCurrent;
    // qDebug() << strCurrent.toUtf8().data(); // << '\t' << temperature;
    if (ui->cbAnalyse_Std->currentIndex() == 0)
        strTemp = QString::number(temperature + noiseTemp) + "\r\n";
    else
        strTemp = "$T=" + QString::number(temperature + noiseTemp) + " ;\r\n";

    if (ui->cbAnalyse_Dtm->currentIndex() == 0)
        strCurrent = QString::number(current + noiseCurrent, 'f', 0) + "\r\n";
    else
        strCurrent = "$T=" + QString::number(current + noiseCurrent, 'f', 0) + " ;\r\n";

    serial_Std->write(strTemp.toUtf8().data());
    serial_Dtm->write(strCurrent.toUtf8().data());
}

void MainWindow::on_spinBox_Period_valueChanged(int arg1)
{
    timer->setInterval(arg1);
}

void MainWindow::on_doubleSpinBox_P_valueChanged(double arg1)
{
    pid->setP(arg1);
}

void MainWindow::on_doubleSpinBox_I_valueChanged(double arg1)
{
    pid->setI(arg1);
}

void MainWindow::on_doubleSpinBox_D_valueChanged(double arg1)
{
    pid->setD(arg1);
}

void MainWindow::on_spinBox_Limit_valueChanged(int arg1)
{
    pid->setLimit(arg1);
}

void MainWindow::on_spinBox_Target_valueChanged(int arg1)
{
    ui->label_TargetTemp->setText(QString::number(ADC2Temp(arg1)));
}

void MainWindow::on_comboBox_activated(const QString &arg1)
{
    ui->spinBox_Target->setValue(arg1.toInt());
}
