#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QSerialPort>
#include <random>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class PID
{
public:
    PID(double p, double i, double d, double limit);
    void setP(double p);
    void setI(double i);
    void setD(double d);
    void setLimit(double l);

    double update(double target, double current);

private:
    double p;
    double i;
    double d;
    double lastError;
    double integral;
    double limit;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    double randomGauss(double stddev, double average);
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void loadSettings();
    void saveSettings();

private slots:
    void on_spinBox_Period_valueChanged(int arg1);

    void on_doubleSpinBox_P_valueChanged(double arg1);

    void on_doubleSpinBox_I_valueChanged(double arg1);

    void on_doubleSpinBox_D_valueChanged(double arg1);

    void on_spinBox_Limit_valueChanged(int arg1);

    void on_spinBox_Target_valueChanged(int arg1);

    void on_comboBox_activated(const QString &arg1);

private:
    Ui::MainWindow *ui;
    double current = 25;
    PID *pid = nullptr;
    QTimer *timer = nullptr;
    QSerialPort *serial_Std = nullptr;
    QSerialPort *serial_Dtm = nullptr;

    std::default_random_engine generator;

    // RandomGauss *randomGauss_Std = nullptr;
    // RandomGauss *randomGauss_Dtm = nullptr;

    void simulate();
    QSerialPort *serialInit(QSerialPort *sp, const QString &portName);
};
#endif // MAINWINDOW_H
