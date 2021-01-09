#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QRandomGenerator>

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), ui(new Ui::MainWindow){
    ui->setupUi(this);

    shapes << QCPScatterStyle::ssStar;
    shapes << QCPScatterStyle::ssCross;
    shapes << QCPScatterStyle::ssDisc;
    shapes << QCPScatterStyle::ssCrossSquare;
    shapes << QCPScatterStyle::ssPlusSquare;
    shapes << QCPScatterStyle::ssCrossCircle;
    shapes << QCPScatterStyle::ssPlusCircle;
    shapes << QCPScatterStyle::ssPeace;
    shapes << QCPScatterStyle::ssCircle;
    shapes << QCPScatterStyle::ssSquare;
    shapes << QCPScatterStyle::ssDiamond;
    shapes << QCPScatterStyle::ssTriangle;
    shapes << QCPScatterStyle::ssTriangleInverted;
    shapes << QCPScatterStyle::ssPlus;
    shapes << QCPScatterStyle::ssCustom;

    colors << QColor("red");
    colors << QColor("green");
    colors << QColor("blue");
    colors << QColor("darkYellow");
    colors << QColor("magenta");
    colors << QColor("darkBlue");
    colors << QColor("gray");
    colors << QColor("black");
    colors << QColor("darkRed");
    colors << QColor("darkGreen");
    colors << QColor("cyan");
    colors << QColor("darkCyan");
    colors << QColor("darkMagenta");
    colors << QColor("darkGray");

    makeplot();

    ui->tabWidget->setTabText(0,"Single_Neuron");
    ui->tabWidget->setTabText(1,"Multi_Neuron");
    ui->tabWidget->setTabText(2,"Multi_Layer");
    connect(ui->grafikplot,SIGNAL(mousePress(QMouseEvent*)),SLOT(clickedGraph(QMouseEvent*)));

    connect(ui->actionRandomly,SIGNAL(triggered()),SLOT(initializeRandom()));
    //printBoard();
}

MainWindow::~MainWindow()
{
    delete ui;
}

int sgn(double val){
    return val > 0 ? 1 : -1 ;

}

void MainWindow::addPoint(double x,double y,int cls){

    while(Cvector.size()<=cls){
        Cvector.append(Class());
    }
    qDebug()<<"csize:"<<Cvector.size()<<"cls:"<<cls;
    Cvector[cls-1].P_x.append(x);
    Cvector[cls-1].P_y.append(y);

    ui->grafikplot->graph(cls)->setData(Cvector[cls-1].P_x,Cvector[cls-1].P_y);

    /*for(int i=0;i<Cvector.size();i++){
        qDebug()<<Cvector[i].P_x;
        qDebug()<<Cvector[i].P_y<<"\n";

    }*/

    ui->grafikplot->replot();
    ui->grafikplot->update();
    pointcount++;
}

void MainWindow::makeplot(){

    ui->grafikplot->addGraph();
    ui->grafikplot->addGraph();
    ui->grafikplot->addGraph();

    ui->grafikplot->graph(1)->setScatterStyle(shapes[1]);
    ui->grafikplot->graph(2)->setScatterStyle(shapes[2]);

    ui->grafikplot->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->grafikplot->graph(2)->setLineStyle(QCPGraph::lsNone);

    ui->grafikplot->graph(1)->setPen(QPen(colors[0]));
    ui->grafikplot->graph(2)->setPen(QPen(colors[1]));

    ui->grafikplot->xAxis->setLabel("x");
    ui->grafikplot->yAxis->setLabel("y");
    ui->grafikplot->xAxis->setRange(-5, 5);
    ui->grafikplot->yAxis->setRange(-5, 5);

    ui->grafikplot->replot();

}

void MainWindow::clickedGraph(QMouseEvent * event){

    QPoint point = event->pos();

    ui->label1->setText(QString("x:%1 y:%2").arg(ui->grafikplot->xAxis->pixelToCoord(point.x())).arg(ui->grafikplot->yAxis->pixelToCoord(point.y())));

    if(currentwin==0){
        if(ui->Class1->isChecked()){
            //qDebug()<<"Class1";
            //qDebug()<<ui->grafikplot->xAxis->pixelToCoord(point.x())<<ui->grafikplot->yAxis->pixelToCoord(point.y());
            addPoint(ui->grafikplot->xAxis->pixelToCoord(point.x()),ui->grafikplot->yAxis->pixelToCoord(point.y()),1);

            ui->label2->setText(QString("Class1: %1").arg(Cvector[0].P_x.size()));
        }
        else{
            //qDebug()<<"Class2";
            //qDebug()<<ui->grafikplot->xAxis->pixelToCoord(point.x())<<ui->grafikplot->yAxis->pixelToCoord(point.y());
            addPoint(ui->grafikplot->xAxis->pixelToCoord(point.x()),ui->grafikplot->yAxis->pixelToCoord(point.y()),2);

            ui->label2->setText(QString("Class2: %1").arg(Cvector[1].P_x.size()));
        }
    }
    else if(currentwin==1){
        addPoint(ui->grafikplot->xAxis->pixelToCoord(point.x()),ui->grafikplot->yAxis->pixelToCoord(point.y()),ui->comboBox->currentIndex()+1);
    }
    else if(currentwin==2){
        if(ui->comboBoxMultiLayer->count()>0)
            addPoint(ui->grafikplot->xAxis->pixelToCoord(point.x()),ui->grafikplot->yAxis->pixelToCoord(point.y()),ui->comboBoxMultiLayer->currentIndex()+1);

    }
}

void MainWindow::on_pushButtonSet_clicked()
{

    classSize=ui->lineEditClassSize->text().toInt();
    hiddenNeuronSize=ui->lineEditHiddenSize->text().toInt();

    ui->comboBoxMultiLayer->clear();
    for(int i=1;i<=classSize;i++)
        ui->comboBoxMultiLayer->addItem(QString("%1").arg(i));

    for(int t=2;t<classSize;t++){
        ui->grafikplot->addGraph();
        ui->grafikplot->graph(t+1)->setScatterStyle(shapes[t+3]);
        ui->grafikplot->graph(t+1)->setLineStyle(QCPGraph::lsNone);
        ui->grafikplot->graph(t+1)->setPen(QPen(colors[t]));

    }


}

void MainWindow::on_buttonPerceptronMul_clicked()
{
    PerceptronMultineron();
}

void MainWindow::on_buttonDeltaMul_clicked()
{
    PerceptronMultineron_Delta();
}

void MainWindow::on_pushButton_clicked()
{
    Perceptron();
}

void MainWindow::on_buttondeltamultilayer_clicked()
{
    HiddenLayer();
}

void MainWindow::on_buttondelta_clicked()
{
    Perceptron_Delta();
}

void MainWindow::on_Class2_stateChanged(int arg1)
{
    if(arg1==0)
        ui->Class1->setChecked(1);
    else
        ui->Class1->setChecked(0);
}

void MainWindow::on_Class1_stateChanged(int arg1)
{
    if(arg1==0)
        ui->Class2->setChecked(1);
    else
        ui->Class2->setChecked(0);
}

void MainWindow::initializeRandom(){

    w[0] = QRandomGenerator::global()->generateDouble()*2-1;

    w[1] = QRandomGenerator::global()->generateDouble()*2-1;

    w[2] = QRandomGenerator::global()->generateDouble()*2-1;

    drawline(w[0],w[1],w[2]);

}

void MainWindow::drawline(double x1,double x2,double x3){

    QVector<double> x(501),y(501);
    for(int i=0;i<501;i++){
        x[i] = i/50.0 - 5;
        y[i] = (x1*x[i]+x3)/-x2;
    }
    ui->grafikplot->graph(0)->setData(x,y);
    ui->grafikplot->graph(0)->setPen(QPen(colors[0]));
    ui->grafikplot->replot();
}

void MainWindow::drawmultiline(double * lines, int size){

    //qDebug()<<size<<" tane çızgı var    ";
    QVector<double> x(501),y(501);
    for(int t = 0 ; t < size ; t++){

        for(int i=0;i<501;i++){
            x[i] = i/50.0 - 5;
            y[i] = (lines[t * 3]*x[i]+lines[ t * 3 + 2])/-lines[t * 3 +1];
        }
        //qDebug()<<lines[t * 3]<<lines[ t * 3 + 2]<<lines[t * 3 +1];
        ui->grafikplot->graph(ui->grafikplot->graphCount()-t-1)->setData(x,y);
        ui->grafikplot->graph(ui->grafikplot->graphCount()-t-1)->setPen(QPen(colors[t]));
        ui->grafikplot->replot();
    }

}

void MainWindow::clearmultilines(int count){

    for(int t=0;t<count;t++)
        ui->grafikplot->graph(ui->grafikplot->graphCount()-count+1)->data()->clear();
}

void MainWindow::clearall(){

    Cvector.clear();
    ui->comboBox->clear();

    ui->comboBox->addItem("1");
    ui->comboBox->addItem("2");
    pointcount=0;
}

void MainWindow::clearboard(){
    for(int i = 0 ; i < Cvector.size();i++){
        Cvector[i].P_x.clear();
        Cvector[i].P_y.clear();
    }
    Cvector.clear();
    ui->grafikplot->clearItems();
    for( int g=0; g < ui->grafikplot->graphCount(); g++ )
        ui->grafikplot->graph(g)->data()->clear();
    ui->grafikplot->replot();
    pointcount=0;

}

void MainWindow::on_buttonclear_clicked()
{
    clearboard();
}

void MainWindow::on_tabWidget_currentChanged(int index)
{
    clearall();
    clearboard();
    currentwin=index;
    printed=false;
}

void MainWindow::on_buttonCadd_clicked()
{
    //qDebug()<<"csize:"<<Cvector.size()<<"graphsize:"<<ui->grafikplot->graphCount();

    if(Cvector.size() > ui->comboBox->count()){
        ui->comboBox->addItem(QString("%1").arg(Cvector.size()));
        Cvector.append(Class());

        ui->grafikplot->addGraph();
        ui->grafikplot->graph(Cvector.size()-1)->setScatterStyle(shapes[Cvector.size()+1]);
        ui->grafikplot->graph(Cvector.size()-1)->setLineStyle(QCPGraph::lsNone);
        ui->grafikplot->graph(Cvector.size()-1)->setPen(QPen(colors[Cvector.size()-2]));
    }

}

int MainWindow::getNeuralOut(double x,double y){


    double * neuronoutput = new double[classSize];
    double * neuronnet = new double[classSize];
    double * hiddenoutput = new double[hiddenNeuronSize];
    double * hiddennet = new double[hiddenNeuronSize];
    double bias = -1.0;


    for(int hddcounter = 0 ; hddcounter < hiddenNeuronSize ; hddcounter++){//hidden layer

        hiddennet[hddcounter] = x * v3[3*hddcounter+0]+ y * v3[3*hddcounter+1]+ bias * v3[3*hddcounter+2];
        hiddenoutput[hddcounter]=(1/(1+exp(-hiddennet[hddcounter])));

    }
    hiddenoutput[hiddenNeuronSize-1]=bias;

    for(int neucounter = 0 ; neucounter < classSize ; neucounter++){//output layer

        neuronnet[neucounter] = 0;
        for(int hddcounter = 0 ; hddcounter < hiddenNeuronSize; hddcounter++){

            neuronnet[neucounter] += hiddenoutput[hddcounter] * w3[neucounter * hiddenNeuronSize + hddcounter] ;

        }

        neuronoutput[neucounter]=(1/(1+exp(-neuronnet[neucounter])));



    }
    //qDebug()<<x<<y;
    int maxindex=0;
    for(int i=0;i<classSize;i++){
        maxindex = (neuronoutput[i] > neuronoutput[maxindex] ) ? i : maxindex;
    }
        //qDebug()<<maxindex;
        return maxindex;

}

void MainWindow::printBoard(){
    ui->grafikplot->addGraph();

    ui->grafikplot->graph()->setLineStyle(QCPGraph::lsNone);
    QVector <double> x,y;




    for(int cls=0;cls<classSize;cls++){
        x.clear();
        y.clear();
        ui->grafikplot->addGraph();
        ui->grafikplot->graph()->setLineStyle(QCPGraph::lsNone);
        QColor e(colors[cls]);
        e.setAlpha(70);
        QCPScatterStyle d(shapes[9],e,1.5);

        ui->grafikplot->graph()->setScatterStyle(d);

        for(double i = -5 ; i <= 5 ; i=i+0.1){
            for(double j = -5 ; j < 5 ;j=j+0.1){
                if(getNeuralOut(i,j)==cls){
                    x.append(i);
                    y.append(j);

                }

            }
        }

        ui->grafikplot->graph()->setData(x,y);
        ui->grafikplot->replot();

    }




}

void MainWindow::Perceptron(){

    double learningrate=0.0005;
    double bias = 1;
    double net = 0;
    int desiredVal = 0;
    int output = 0;
    int error = 1;

    while(error>0){
        error = 0;
        for(int j = 0;j<Cvector.size();j++){
            for(int i = 0;i<Cvector[j].P_x.size() ;i++){

                net = w[0]*Cvector[j].P_x[i] + w[1]*Cvector[j].P_y[i] + w[2]*bias;

                output = sgn(net);
                if(j==0)
                    desiredVal=1;
                else if(j==1)
                    desiredVal=-1;

                w[0] += learningrate * (desiredVal - output) * Cvector[j].P_x[i];
                w[1] += learningrate * (desiredVal - output) * Cvector[j].P_y[i];
                w[2] += learningrate * (desiredVal - output) * bias;

                error += abs(desiredVal - output)/2;

                drawline(w[0],w[1],w[2]);

            }
        }

    ui->label->setText(QString("w0: %1     w1: %2     w2: %3").arg(w[0]).arg(w[1]).arg(w[2]));
    }

}

void MainWindow::PerceptronMultineron(){

    int size = Cvector.size();//class sayısı kadar çizgi
    double learningrate = 1;
    double bias = 1;
    double * net = new double[size-1];
    double * desiredVal = new double[size-1];
    double error = 1;
    w2 = new double[size*3];
    int cycle=0;
    if(!printed){

        for(int y=0;y<size-1;y++)
            ui->grafikplot->addGraph();

    }
    for(int y=0;y<(size-1)*3;y++)
        w2[y]=0;
    qDebug()<<size<<Cvector[0].P_x.size();

    if(printed){
        clearmultilines(size);
    }

    while(error > 0){
        error = 0;

        for(int cls = 0; cls < Cvector.size() ; cls++){//noktalar için sınıfları gezer

            for(int t = 0 ; t < size ; t++){//

                if(t==cls){
                    desiredVal[t]=+1;
                }
                else{
                    desiredVal[t]=-1;
                }
            }

            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){//noktaları gezer

                for(int t=0 ; t < size-1 ; t++){
                    net[t]=0;
                }

                for(int k=0 ; k < size-1 ; k++){
                    net[k]+=w2[k * 3   ]  * Cvector[cls].P_x[Pcounter];
                    net[k]+=w2[k * 3 + 1] * Cvector[cls].P_y[Pcounter];
                    net[k]+=w2[k * 3 + 2] * bias;
                }

                for(int f = 0 ; f < size-1 ; f++){
                    if(desiredVal[f] != sgn( net[f] )){
                        w2[f * 3     ] += learningrate * (desiredVal[f] - sgn( net[f] ))* Cvector[cls].P_x[Pcounter];
                        w2[f * 3 + 1 ] += learningrate * (desiredVal[f] - sgn( net[f] ))* Cvector[cls].P_y[Pcounter];
                        w2[f * 3 + 2 ] += learningrate * (desiredVal[f] - sgn( net[f] ))* bias;
                    }

                    drawmultiline(w2,size-1);
                }

                for(int r=0;r<size-1;r++)
                    error+=abs(desiredVal[r] - sgn( net[r]))/2;
            }
        }
    cycle++;
    ui->labelcycle_2->setText(QString("Cycle %1").arg(cycle));
    ui->lableError->setText(QString("Error %1").arg(error));
    }
    printed=true;
}

void MainWindow::Perceptron_Delta(){

    double learningrate=0.5;
    double bias = 1;
    double net = 0;
    double desiredVal = 0;
    double output = 0;
    double derivationOutput = 0;
    double error = 0.5;
    int cycle=0;

    if(ui->checknorm->isChecked()){
        qDebug()<<"normalize selected";
        int meanX = 0,meanY = 0;
        double varianceX=0,varianceY=0;

        for(int cls = 0; cls < Cvector.size() ; cls++){
            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){
                meanX+=Cvector[cls].P_x[Pcounter]/pointcount;
                meanY+=Cvector[cls].P_y[Pcounter]/pointcount;
            }
        }

        for(int cls = 0; cls < Cvector.size() ; cls++){
            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){
                varianceX+=pow((meanX-Cvector[cls].P_x[Pcounter]),2)/pointcount;
                varianceY+=pow((meanY-Cvector[cls].P_y[Pcounter]),2)/pointcount;
            }
        }

        varianceX=sqrt(varianceX);
        varianceY=sqrt(varianceY);

        for(int cls = 0; cls < Cvector.size() ; cls++){
            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){

                Cvector[cls].P_x[Pcounter]=(Cvector[cls].P_x[Pcounter]-meanX)/varianceX;
                Cvector[cls].P_y[Pcounter]=(Cvector[cls].P_y[Pcounter]-meanY)/varianceY;
                qDebug()<<Cvector[cls].P_x[Pcounter]<<Cvector[cls].P_y[Pcounter];
            }
        }
        for(int cls = 0; cls < Cvector.size()-1 ; cls++)
            qDebug()<<Cvector[cls].P_x<<Cvector[cls].P_y;

        for(int i=1;i<Cvector.size();i++)
            ui->grafikplot->graph(i)->setData(Cvector[i-1].P_x,Cvector[i-1].P_y);

        ui->grafikplot->replot();
        ui->grafikplot->update();
    }

    while(error > 0.25){
        error = 0;
        for(int cls = 0; cls < Cvector.size() ; cls++){
            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){

                net = w[0]*Cvector[cls].P_x[Pcounter] + w[1]*Cvector[cls].P_y[Pcounter] + w[2]*bias;

                output =((2 / (1 + exp(-net))) - 1);
                derivationOutput = 0.5 * (1 - pow(output, 2));

                if(cls==0)
                    desiredVal=1;
                else
                    desiredVal=-1;

                if(desiredVal!=output){
                    w[0] += learningrate * (desiredVal - output) * Cvector[cls].P_x[Pcounter] * derivationOutput;
                    w[1] += learningrate * (desiredVal - output) * Cvector[cls].P_y[Pcounter] * derivationOutput;
                    w[2] += learningrate * (desiredVal - output) * bias * derivationOutput ;
                }
                error += 0.5 * pow((desiredVal - output) , 2);

                drawline(w[0],w[1],w[2]);

            }
        }

    ui->label->setText(QString("w0: %1     w1: %2     w2: %3").arg(w[0]).arg(w[1]).arg(w[2]));
    ui->labelcycle->setText(QString("Cycle: %1   Error:%2").arg(cycle).arg(error));
    //qDebug()<<error;
    cycle++;
    }

}

void MainWindow::PerceptronMultineron_Delta(){

    int size = Cvector.size();
    double learningrate=0.09;
    double bias = 1;

    double * derivation = new double[size];
    double * output = new double[size];
    double * net = new double[size];

    int  * desiredVal = new int[size];
    double error = 999;
    int cycle=0;

    w2 = new double[size*3];

    if(!printed){

        for(int y=0;y<size-1;y++)
            ui->grafikplot->addGraph();

    }

    for(int y=0;y<(size)*3;y++)
        w2[y]=0;

    if(printed){
        clearmultilines(size);
    }

    for(int t=0 ; t < size ; t++){
        net[t]=0;
        output[t]=0;
        derivation[t]=0;
    }

    if(ui->MultiNormalize->isChecked()){
        qDebug()<<"normalize selected";
        int meanX = 0,meanY = 0;
        double varianceX=0,varianceY=0;

        for(int cls = 0; cls < Cvector.size() ; cls++){
            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){
                meanX+=Cvector[cls].P_x[Pcounter]/pointcount;
                meanY+=Cvector[cls].P_y[Pcounter]/pointcount;
            }
        }

        for(int cls = 0; cls < Cvector.size() ; cls++){
            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){
                varianceX+=pow((meanX-Cvector[cls].P_x[Pcounter]),2)/pointcount;
                varianceY+=pow((meanY-Cvector[cls].P_y[Pcounter]),2)/pointcount;
            }
        }

        varianceX=sqrt(varianceX);
        varianceY=sqrt(varianceY);

        for(int cls = 0; cls < Cvector.size() ; cls++){
            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){

                Cvector[cls].P_x[Pcounter]=(Cvector[cls].P_x[Pcounter]-meanX)/varianceX;
                Cvector[cls].P_y[Pcounter]=(Cvector[cls].P_y[Pcounter]-meanY)/varianceY;
            }
        }
        for(int i=1;i<Cvector.size();i++)
            ui->grafikplot->graph(i)->setData(Cvector[i-1].P_x,Cvector[i-1].P_y);

        ui->grafikplot->replot();
        ui->grafikplot->update();
    }

    while(error > 1){
        error = 0;

        for(int cls = 0; cls < size ; cls++){

            for(int t=0;t<size;t++){
                if(cls==t)
                    desiredVal[t]=1;
                else
                    desiredVal[t]=-1;
            }

            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){

                for (int i=0 ; i < size ; i++){
                    net[i] = w2[i*3+0]*Cvector[cls].P_x[Pcounter]+ w2[i*3+1]*Cvector[cls].P_y[Pcounter]+w2[i*3+2]*bias ;

                    output[i] = ((2 / (1 + exp(-net[i]))) - 1);
                    if(desiredVal[i]!=output[i]){

                        derivation[i] = 0.5 * (1 - pow(output[i], 2));
                        w2[i*3+0] += learningrate * 0.5 * (desiredVal[i] - output[i]) * Cvector[cls].P_x[Pcounter] * derivation[i];
                        w2[i*3+1] += learningrate * 0.5 * (desiredVal[i] - output[i]) * Cvector[cls].P_y[Pcounter] * derivation[i];
                        w2[i*3+2] += learningrate * 0.5 * (desiredVal[i] - output[i]) * bias * derivation[i] ;
                    }

                    error+=0.5 * pow((desiredVal[i] - output[i]),2);

                    drawmultiline(w2,size-1);
                }
            }

        }

    cycle++;
    qDebug()<<error;

    ui->labelcycle_2->setText(QString("Cycle %1").arg(cycle));
    ui->lableError->setText(QString("Error %1").arg(error));

    }

    printed=true;

}

void MainWindow::HiddenLayer(){
    double error=999.0;
    double learningrate=0.8;
    double bias = -1.0;
    bool flag=false;
    //hiddenNeuronSize++;//bias için +1
    double * neuronoutput = new double[classSize];
    double * neuronnet = new double[classSize];
    hiddenNeuronSize++;
    double * hiddenoutput = new double[hiddenNeuronSize];
    double * hiddennet = new double[hiddenNeuronSize];

    int  * desiredVal = new int[classSize];
    int cycle=0;
    double * errorO = new double[classSize] ;
    double * errorY = new double[hiddenNeuronSize] ;

    double temp;

    v3 = new double[hiddenNeuronSize*3];
    w3 = new double[hiddenNeuronSize*classSize];

    QRandomGenerator64 rd;

    for(int i=0;i<hiddenNeuronSize*classSize;i++){
        w3[i]=i/10.0;//******************************
    }
    for(int i=0;i<hiddenNeuronSize*3;i++)
        v3[i]=i/10.0;//***************************
    if(!printed){

        for(int y=0;y<hiddenNeuronSize*classSize;y++)
            ui->grafikplot->addGraph();
    }

    if(printed){
        clearmultilines(hiddenNeuronSize*classSize);
    }

    if(ui->checkBoxNormalizeMulti->isChecked()){
        qDebug()<<"normalize selected";
        int meanX = 0,meanY = 0;
        double varianceX=0,varianceY=0;

        for(int cls = 0; cls < Cvector.size() ; cls++){
            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){
                meanX+=Cvector[cls].P_x[Pcounter]/pointcount;
                meanY+=Cvector[cls].P_y[Pcounter]/pointcount;
            }
        }

        for(int cls = 0; cls < Cvector.size() ; cls++){
            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){
                varianceX+=pow((meanX-Cvector[cls].P_x[Pcounter]),2)/pointcount;
                varianceY+=pow((meanY-Cvector[cls].P_y[Pcounter]),2)/pointcount;
            }
        }

        varianceX=sqrt(varianceX);
        varianceY=sqrt(varianceY);

        for(int cls = 0; cls < Cvector.size() ; cls++){
            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){

                Cvector[cls].P_x[Pcounter]=(Cvector[cls].P_x[Pcounter]-meanX)/varianceX;
                Cvector[cls].P_y[Pcounter]=(Cvector[cls].P_y[Pcounter]-meanY)/varianceY;
            }
        }

        for(int i=1;i<Cvector.size();i++)
            ui->grafikplot->graph(i)->setData(Cvector[i-1].P_x,Cvector[i-1].P_y);

        ui->grafikplot->replot();
        ui->grafikplot->update();
    }

    while(error>0.1){
        error=0;
        flag=false;

        /*for(int i=0;i<hiddenNeuronSize*classSize;i++)
            tempw3[i]=w3[i];*/

        for(int cls = 0; cls < Cvector.size() ; cls++){


            for(int t=0;t<classSize;t++){
                if(cls==t)
                    desiredVal[t]=1;
                else
                    desiredVal[t]=0;
            }


            for(int Pcounter = 0; Pcounter < Cvector[cls].P_x.size() ; Pcounter++){
                //qDebug()<<"cls"<<cls<<"Pcounter"<<Pcounter;


                for(int hddcounter = 0 ; hddcounter < hiddenNeuronSize ; hddcounter++){//hidden layer

                    hiddennet[hddcounter] = Cvector[cls].P_x[Pcounter] * v3[3*hddcounter+0]+ Cvector[cls].P_y[Pcounter] * v3[3*hddcounter+1]+ bias * v3[3*hddcounter+2];

                    hiddenoutput[hddcounter]=(1/(1+exp(-hiddennet[hddcounter])));
                    //qDebug()<<"hdd"<<hddcounter<<"hddval"<<hiddenoutput[hddcounter]<<"xmul"
                    //       <<v3[3*hddcounter+0]<<"ymul"<<v3[3*hddcounter+1]<<"biasmul"<<v3[3*hddcounter+2];
                }
                hiddenoutput[hiddenNeuronSize-1]=bias;

                for(int neucounter = 0 ; neucounter < classSize ; neucounter++){//output layer

                    neuronnet[neucounter] = 0;
                    for(int hddcounter = 0 ; hddcounter < hiddenNeuronSize; hddcounter++){

                        neuronnet[neucounter] += hiddenoutput[hddcounter] * w3[neucounter * hiddenNeuronSize + hddcounter] ;

                        //qDebug()<<"neuc"<<neucounter<<"hddout"<<hiddenoutput[hddcounter]<<"w3"<<
                        //          w3[neucounter * hiddenNeuronSize + hddcounter]<<"w3counter"<<neucounter * hiddenNeuronSize + hddcounter
                         //           ;
                    }


                    neuronoutput[neucounter]=(1/(1+exp(-neuronnet[neucounter])));



                    error+= 0.5 * pow((desiredVal[neucounter] - neuronoutput[neucounter]) , 2);
                }

                for(int neucounter = 0 ; neucounter < classSize ; neucounter++){//errorO

                    errorO[neucounter] = 0.5 * ((desiredVal[neucounter]-neuronoutput[neucounter])
                                                * (1 - neuronoutput[neucounter])*neuronoutput[neucounter]);
                    //qDebug()<<"errorO"<<errorO[neucounter]<<"neu"<<neuronoutput[neucounter];
                }


                for(int hddcounter = 0 ; hddcounter < hiddenNeuronSize ; hddcounter++){//errorY   !! netin değil output
                        temp=0;
                    for(int neucounter = 0 ; neucounter < classSize ; neucounter++){
                        temp+=errorO[neucounter] * w3[neucounter *hiddenNeuronSize + hddcounter];
                        //qDebug()<<"neocounter"<<neucounter<<"errorO"<<errorO[neucounter]<<"w3counter"
                         //      <<neucounter *hiddenNeuronSize + hddcounter<<"w3"<<w3[neucounter *hiddenNeuronSize + hddcounter]
                        //      <<"hddcounter"<<hddcounter<<"temp"<<temp;
                    }

                    errorY[hddcounter] = 0.5 * ((1-hiddenoutput[hddcounter])*hiddenoutput[hddcounter]) * temp;

                }

                for(int neucounter = 0 ; neucounter < classSize ; neucounter++){
                    for(int hddcounter = 0 ; hddcounter < hiddenNeuronSize ; hddcounter++){
                        //qDebug()<<"w3"<<v3[neucounter*hiddenNeuronSize+hddcounter]<<"ccc"<<neucounter*hiddenNeuronSize+hddcounter
                         //      <<"hddout"<<hiddenoutput[hddcounter]<<"errorO"<<errorO[neucounter];
                         w3[neucounter*hiddenNeuronSize+hddcounter] += learningrate * hiddenoutput[hddcounter] * errorO[neucounter];

                    }
                }

                double tempvec[3]={Cvector[cls].P_x[Pcounter],Cvector[cls].P_y[Pcounter],bias};

                for(int incounter = 0 ; incounter < 3 ; incounter++){
                    for(int hddcounter = 0 ; hddcounter < hiddenNeuronSize-1 ; hddcounter++){
                        //qDebug()<<"v3"<<v3[hddcounter* 3 + incounter]<<"ccc"<<hddcounter* 3 + incounter
                        //   <<"temp"<<tempvec[incounter]<<"error"<<errorY[hddcounter];

                        v3[ hddcounter* 3 + incounter] += learningrate * tempvec[incounter] * errorY[hddcounter];


                    }
                }

                /*for(int i=0;i<classSize*hiddenNeuronSize;i++)
                    qDebug()<<w3[i];*/
            }

        }
    cycle++;


    /*for(int i=0;i<classSize;i++)
        qDebug()<<neuronoutput[i];*/


    //drawmultiline(w3,hiddenNeuronSize*classSize);

    //break;
    qDebug()<<error<<cycle;
    /*for(int i=0;i<hiddenNeuronSize*classSize && flag==false;i++){

        if(tempw3[i]!=w3[i])
            flag=true;
    }*/
    }

    //drawmultiline(w3,1);
    qDebug()<<error<<cycle;
    printBoard();
    printed=true;
}

