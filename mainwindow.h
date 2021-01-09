#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "qcustomplot.h"
#include <QMainWindow>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
private slots:
    void clickedGraph(QMouseEvent * event);

    void on_Class2_stateChanged(int arg1);

    void on_Class1_stateChanged(int arg1);

    void clearmultilines(int count);

    void initializeRandom();

    void on_pushButton_clicked();

    void on_buttonclear_clicked();

    void on_tabWidget_currentChanged(int index);

    void on_buttonCadd_clicked();

    void on_buttondelta_clicked();

    void on_buttonPerceptronMul_clicked();

    void on_buttonDeltaMul_clicked();

    void on_buttondeltamultilayer_clicked();

    void on_pushButtonSet_clicked();

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void addPoint(double x,double y,int cls);

    void drawline(double, double, double);

    void drawmultiline(double * lines,int size);

    void Perceptron();

    void makeplot();

    void clearall();

    int getNeuralOut(double x ,double y);

    void printBoard();

    void PerceptronMultineron();

    void clearboard();

    void Perceptron_Delta();

    void PerceptronMultineron_Delta();

    void HiddenLayer();

private:
    Ui::MainWindow *ui;
    QVector<QCPScatterStyle::ScatterShape> shapes;
    QVector<QColor> colors;
    struct Class{
        QVector <double> P_x;
        QVector <double> P_y;
    }Cpoints;

    QVector <Class> Cvector;

    double w[3];
    double *w2;
    double *v3;
    double *w3;
    int currentwin=0;
    bool printed=false;
    int pointcount=0;
    int classSize=0;
    int hiddenNeuronSize=0;

};
#endif // MAINWINDOW_H
