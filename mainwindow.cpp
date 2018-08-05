#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //setWindowIcon(QIcon(":/new/prefix1/images/logo_10%.png"));
    setWindowTitle(QString::fromLocal8Bit("hellWarrior����վ"));

    //�������
    setWindowState(Qt::WindowMaximized);
    //��ʼ������
    initialSerial();

    //***
    setupPlot(ui->customPlot);

    isMyComOpen = false;
    isStart = false;
    //ui->button_close->setEnabled(false);
    myCom=NULL;
    ui->statusBar->showMessage(QString::fromLocal8Bit("��ӭʹ��hellWarrior����վ"));

    standardAltimetro = new Altimetro(ui->standardSixAltimetro);    //�½�Altimetro����
    standardQADI = new QADI(ui->standardSixQADI);    //�½�QADI����
    standardQCompass = new QCompass(ui->standardSixQCompass);//�½�QCompass����
}

MainWindow::~MainWindow()
{
    if(myCom!=NULL){
        if(myCom->isOpen()){
            myCom->close();
    }
        delete myCom;
  }
    delete ui;
}

//change eventһ���ǵ�ǰwidget״̬�ı�󴥷���
//������ı䡢���Ըı�֮��ġ�
//�÷�����Ҫ����ı��¼��������Ըı��ִ����ز�����
void MainWindow::changeEvent(QEvent *e)    //��д���¼�������
{
    QMainWindow::changeEvent(e);           //�û���ִ���¼�������
    switch (e->type()) {                   //�����¼������б�
    case QEvent::LanguageChange:           //��������Ըı��¼�
        ui->retranslateUi(this);           //����UI�����ԣ�����������ģ��û��Զ��巽����
        break;
    default:
        break;
    }
}

void MainWindow::on_helpBtn_clicked()
{
    QMessageBox::about(this,QString::fromLocal8Bit("������Ϣ"),QString::fromLocal8Bit(
                       "1��ѡ���þ��崮�����ã�����򿪴��ڼ����յ���Ϣ""\n"
                       "2��������ϵ��ʽ��QQ����344964277""\n"
                       "3����ӭ�������˻�����վ��ƿ���Ⱥ��QQȺ�ţ�631625639��""\n"));
}

void MainWindow::initialSerial()
{
    //���崮��
    ui->portNameComboBox->addItem("COM4");
    ui->portNameComboBox->addItem("COM1");
    ui->portNameComboBox->addItem("COM2");
    ui->portNameComboBox->addItem("COM3");
    ui->portNameComboBox->addItem("COM5");
    ui->portNameComboBox->addItem("COM6");
    ui->portNameComboBox->addItem("COM7");
    ui->portNameComboBox->addItem("COM8");
    ui->portNameComboBox->addItem("COM9");
    //���岨����
    ui->baudRateComboBox->addItem("115200");
    ui->baudRateComboBox->addItem("9600");
    ui->baudRateComboBox->addItem("1200");
    ui->baudRateComboBox->addItem("2400");
    ui->baudRateComboBox->addItem("4800");
    ui->baudRateComboBox->addItem("14400");
    ui->baudRateComboBox->addItem("19200");
    ui->baudRateComboBox->addItem("38400");
    ui->baudRateComboBox->addItem("56000");
    ui->baudRateComboBox->addItem("128000");
    ui->baudRateComboBox->addItem("256000");
    //��������λ
    ui->dataBitsComboBox->addItem("8");
    ui->dataBitsComboBox->addItem("5");
    ui->dataBitsComboBox->addItem("6");
    ui->dataBitsComboBox->addItem("7");
    //У��λ
    ui->parityComboBox->addItem("none");
    ui->parityComboBox->addItem("odd");
    ui->parityComboBox->addItem("even");
    //ֹͣλ
    ui->stopBitsComboBox->addItem("1");
    ui->stopBitsComboBox->addItem("1.5");
    ui->stopBitsComboBox->addItem("2");
    //������
    ui->flowTypeComboBox->addItem("off");
    ui->flowTypeComboBox->addItem("hardware");
    ui->flowTypeComboBox->addItem("xonxoff");

    //��������ѡ����γ���  �ַ�or����
    ui->ContentSelectComboBox->addItem(QString::fromLocal8Bit("��ʾ����"));
    ui->ContentSelectComboBox->addItem(QString::fromLocal8Bit("��ʾ�ַ�"));
}

//comboBox�ؼ���������
void MainWindow::setComboBoxEnabled(bool status)
{
    ui->portNameComboBox->setEnabled(status);
    ui->baudRateComboBox->setEnabled(status);
    ui->dataBitsComboBox->setEnabled(status);
    ui->stopBitsComboBox->setEnabled(status);
    ui->parityComboBox->setEnabled(status);
    ui->flowTypeComboBox->setEnabled(status);
}

//�򿪴��ڿ���
void MainWindow::on_openMyComBtn_clicked()
{
    QString portName = ui->portNameComboBox->currentText();   //��ȡ������
    myCom = new QextSerialPort(portName);        //���崮�ڶ��󣬲����ݲ������ڹ��캯���������г�ʼ��

    connect(myCom, SIGNAL(readyRead()), this, SLOT(readMyCom()));
    //ʵ�����źŵĲ۵����ӡ��������������յ������Ժ󣬾ͻ��readMyCom()������

    //ע�⣺��Ҫ�ȴ򿪴��ڣ�Ȼ�������ô��ڵĲ�������Ȼ������Ч������
    if(myCom->open(QIODevice::ReadWrite))
    {
        myCom->flush(); //���뻺�����ڴ���ȡ

    //���ò�����
    myCom->setBaudRate((BaudRateType)ui->baudRateComboBox->currentText().toInt());

    //��������λ
    myCom->setDataBits((DataBitsType)ui->dataBitsComboBox->currentText().toInt());

    //����У��
    switch(ui->parityComboBox->currentIndex()){
    case 0:
         myCom->setParity(PAR_NONE);
         break;
    case 1:
        myCom->setParity(PAR_ODD);
        break;
    case 2:
        myCom->setParity(PAR_EVEN);
        break;
    default:
        myCom->setParity(PAR_NONE);
        qDebug("set to default : PAR_NONE");
        break;
    }

    //����ֹͣλ
    switch(ui->stopBitsComboBox->currentIndex()){
    case 0:
        myCom->setStopBits(STOP_1);
        break;
    case 1:
 #ifdef Q_OS_WIN
        myCom->setStopBits(STOP_1_5);
 #endif
        break;
    case 2:
        myCom->setStopBits(STOP_2);
        break;
    default:
        myCom->setStopBits(STOP_1);
        qDebug("set to default : STOP_1");
        break;
    }

    //��������������
    myCom->setFlowControl(FLOW_OFF);

    //������ʱ
    myCom->setTimeout(TIME_OUT);

    QMessageBox::information(this, QString::fromLocal8Bit("�򿪳ɹ�"), QString::fromLocal8Bit("�ѳɹ��򿪴���") + portName, QMessageBox::Ok);

   //�������
    ui->openMyComBtn->setEnabled(false);
    ui->closeMyComBtn->setEnabled(true);
    ui->helpBtn->setEnabled(true);
    //�ؼ�����
    setComboBoxEnabled(false);

    ui->statusBar->showMessage(QString::fromLocal8Bit("�򿪴��ڳɹ���"));
    }

    else
    {
       QMessageBox::critical(this, QString::fromLocal8Bit("��ʧ��"), QString::fromLocal8Bit("δ�ܴ򿪴��� ") + portName + QString::fromLocal8Bit("\n�ô����豸�����ڻ��ѱ�ռ��"), QMessageBox::Ok);
       return;
    }

//        //��������嶨ʱ����Ȼ�󴥷���ʼPolling��ѯ
//        myReadTimer = new QTimer(this);
//        myReadTimer->setInterval(10);
//        connect(myReadTimer, SIGNAL(timeout()), this, SLOT(readMyCom()));    //�źźͲۺ��������������ڻ�����������ʱ�����ж����ڲ���
//        this->myReadTimer->start();         //��ʼpoll��ѯ����
}

//�رմ��ڿ���
void MainWindow::on_closeMyComBtn_clicked()
{
    myCom->close();
    delete myCom;
    myCom = NULL;

//    this->myReadTimer->stop();          //�ر�poll����
//    myCom->close();
//    model->removeRows(0, model->rowCount());
//    model->setRowCount(row);
//    model->setColumnCount(column);
//    dcFlag = 0;
//    ui->radarWidget->update();  //ˢ������ϵ��ʵʱ��ʾ����

    ui->openMyComBtn->setEnabled(true);
    ui->helpBtn->setEnabled(true);
    //�ؼ�����
    setComboBoxEnabled(true);

    QMessageBox::information(this, QString::fromLocal8Bit("����״̬��"),QString::fromLocal8Bit("�ر�"),QMessageBox::Ok);
}


//��ȡ����
void MainWindow::readMyCom()
{
    static unsigned char buffer[BUFFER_SIZE] = {0};
    static unsigned  char counter = 0;

    if(myCom->bytesAvailable()<=0){return;}  //�������ʱ���ָ��ڴ�ռ��

    switch(ui->ContentSelectComboBox->currentIndex()){
    case 0:
    /************************************************************************************************************************************
    *�Ӵ����ж������ݲ���ת��Ϊ��������̬��ֵ
    ************************************************************************************************************************************/
             char ch;
//             int   x1_,x2_,x3_,x_alt,x_tempr,x_press,x_IMUpersec;
//             float _x1, _x2, _x3,_x_alt,_x_tempr,_x_press,_x_IMUpersec;
             mavlink_attitude_t g_MavLink_Att_f;
             mavlink_gps_raw_int_t g_MavLink_GPS_f;

             float Longitude,Latitude,Altitude;
             float roll,pitch,yaw;
            // float Longitude = 0.0f,Latitude = 0.0f,Altitude = 0.0f;
            // float roll = 0.0f,pitch = 0.0f,yaw = 0.0f;

//MavLink���ݽṹ�� magic + len + seq + sysid + compid + msgid(1byte) + payload64(len byte) + checksum(2byte)

             while(myCom->bytesAvailable()){
                 myCom->getChar(&ch);
                 buffer[counter] = ch;

                 if(buffer[0]!=BUFFER_HEAD1 && counter==0)
                 {
                     return;
                 }

                 counter++;

                 if(counter == 6 && buffer[5] != BUFFER_HEAD6_Att && buffer[5] != BUFFER_HEAD6_GPS )
                 {
                     counter = 0;
                     return;
                 }

                 if(counter >= (buffer[1]+8) )
                 {
                     counter = 0;
                     switch(buffer[5])
                     {
                         case BUFFER_HEAD6_Att:
                                 //��̬��Ϣ����
                                 mavlink_msg_attitude_decode(buffer, &g_MavLink_Att_f);//����g_Attitude_f������������ǰ�Ӹ�&����

                                 //����ת�Ƕ�
                                 yaw = g_MavLink_Att_f.yaw*r2d;
                                 roll = g_MavLink_Att_f.roll*r2d;
                                 pitch = g_MavLink_Att_f.pitch*r2d;

                                 standardQADI->setData(pitch,roll);
                                 standardQCompass->setData(yaw,Altitude,Altitude);
                                 plotUpdate(ui->customPlot, yaw,Altitude,Altitude);


//                                 standardQADI->setData(roll,pitch);
//                                 //ƫ�� ���� ��ת�ź�ʾ��
//                                 plotUpdate(ui->customPlot, yaw, pitch, roll);
                         break;

                         case BUFFER_HEAD6_GPS:
                                 //GPS��Ϣ����
                                 mavlink_msg_gps_raw_int_decode(buffer,&g_MavLink_GPS_f);//����g_Attitude_f������������ǰ�Ӹ�&����

                                 Longitude = (float)g_MavLink_GPS_f.lon / 1E7f;
                                 Latitude = (float)g_MavLink_GPS_f.lat / 1E7f;
                                 Altitude = (float)g_MavLink_GPS_f.alt / 1E3f;

//                                 standardQCompass->setData(yaw,Altitude,Altitude);
                         break;

                         default:
                             counter = 0;
//                         plotUpdate(ui->customPlot, _x1, _x2, _x3);

                     }//end of switch(buffer[5])


                 }//end of if(counter >= (buffer[1]+8) )

//                 standardQADI->setData(roll,pitch);
//                 standardQCompass->setData(yaw,Altitude,Altitude);
//                 plotUpdate(ui->customPlot, yaw,Altitude,Altitude);


            } //end of while(myCom->bytesAvailable())
//             plotUpdate(ui->customPlot, _x1, _x2, _x3);

             break;


    case 1:
//��ʾ֡���ݵĴ���
        QByteArray temp = myCom->readAll();
        QString buf;

    if(!temp.isEmpty()){
            ui->textBrowser->setTextColor(Qt::black);
            if(ui->chradioButton->isChecked()){
                buf = temp;
            }
            else if(ui->hexradioButton->isChecked()){
                QString str;
                for(int i = 0; i < temp.count(); i++){
                    QString s;
                    s.sprintf("0x%02x, ", (unsigned char)temp.at(i));
                    buf += s;
                }
            }
            if(!write2fileName.isEmpty()){
                QFile file(write2fileName);
                //�����ʧ���������ʾ���˳�����
                if(!file.open(QFile::WriteOnly | QIODevice::Text)){
                    QMessageBox::warning(this, QString::fromLocal8Bit("д���ļ�"), QString::fromLocal8Bit("���ļ� %1 ʧ��, �޷�д��\n%2").arg(write2fileName).arg(file.errorString()), QMessageBox::Ok);
                    return;
                }
                QTextStream out(&file);
                out<<buf;
                file.close();
            }

           //�ú�����Ӱ��GUI��������
           //ui->textBrowser->setText(ui->textBrowser->document()->toPlainText() + buf);
             ui->textBrowser->append(buf);
                if(ui->textBrowser->document()->toPlainText().size() > DISPLAY_SIZE)
                {
                    ui->textBrowser->setText("");
                }
                QTextCursor cursor = ui->textBrowser->textCursor();
                cursor.movePosition(QTextCursor::End);
                ui->textBrowser->setTextCursor(cursor);

                ui->recvbyteslcdNumber->display(ui->recvbyteslcdNumber->value() + temp.size());
                ui->statusBar->showMessage(QString::fromLocal8Bit("�ɹ���ȡ%1�ֽ�����").arg(temp.size()));
       }
    break;
    }
}

//����������
void MainWindow::on_actionClearBytes_clicked()
{
    if(ui->recvbyteslcdNumber->value() == 0){
        QMessageBox::information(this, QString::fromLocal8Bit("��ʾ��Ϣ"), QString::fromLocal8Bit("ò���Ѿ�������ѽ:)"), QMessageBox::Ok);
    }else{
        ui->recvbyteslcdNumber->display(0);
        ui->statusBar->showMessage(QString::fromLocal8Bit("�������Ѿ�����"));
    }
}

//��ռ�¼
void MainWindow::on_clearUpBtn_clicked()
{
    ui->textBrowser->clear();
    ui->statusBar->showMessage(QString::fromLocal8Bit("��¼�Ѿ����"));
}


//����  �˵���
void MainWindow::on_actionAbout_triggered()
{
    aboutdlg.show();
    // ���������м���ʾ
    int x =this->x() + (this->width() - aboutdlg.width()) / 2;
    int y =this->y() + (this->height() - aboutdlg.height()) / 2;
    aboutdlg.move(x, y);
    ui->statusBar->showMessage(QString::fromLocal8Bit("����hellWarrior"));
}

//����textBrowser�е�����
void MainWindow::on_actionSave_clicked()
{

    if(ui->textBrowser->toPlainText().isEmpty()){
        QMessageBox::information(this, "��ʾ��Ϣ", QString::fromLocal8Bit("ò�ƻ�û������! ����Ҫ�ڷ��ͱ༭��������Ҫ���͵�����"), QMessageBox::Ok);
        return;
    }

    QString filename = QFileDialog::getSaveFileName(this, QString::fromLocal8Bit("����Ϊ"), QString::fromLocal8Bit("δ����.txt"));
    QFile file(filename);
    //����û�ȡ���˱�����ֱ���˳�����
    if(file.fileName().isEmpty()){
        return;
    }
    //�����ʧ���������ʾ���˳�����
    if(!file.open(QFile::WriteOnly | QIODevice::Text)){
        QMessageBox::warning(this, QString::fromLocal8Bit("�����ļ�"), QString::fromLocal8Bit("���ļ� %1 ʧ��, �޷�����\n%2").arg(filename).arg(file.errorString()), QMessageBox::Ok);
        return;
    }
    //д���ݵ��ļ�
    QTextStream out(&file);
    out<<ui->textBrowser->toPlainText();
    file.close();
    //�޸Ĵ��ڱ���Ϊ�����ļ�·��
    setWindowTitle("saved: " + QFileInfo(filename).canonicalFilePath());


}

//�˳�����
void MainWindow::on_actionExit_clicked()
{
    this->close();
}

/************************************************************************************************************************************
*��ͼ
************************************************************************************************************************************/
//��ͼ��ʼ��
void MainWindow::setupPlot(QCustomPlot *customPlot)
{
    t = new QVector<double>;
    x1 = new QVector<double>;
    x2 = new QVector<double>;
    x3 = new QVector<double>;

    customPlot->legend->setVisible(true);
    customPlot->legend->setFont(QFont("Helvetica",9));

    customPlot->addGraph();
    customPlot->graph(0)->setName("yaw");
    customPlot->graph(0)->setData(*t, *x1);
    customPlot->graph(0)->setPen(QPen(Qt::blue));

    customPlot->addGraph();
    customPlot->graph(1)->setName("pitch");
    customPlot->graph(1)->setData(*t, *x2);
    customPlot->graph(1)->setPen(QPen(Qt::red));

    customPlot->addGraph();
    customPlot->graph(2)->setName("roll");
    customPlot->graph(2)->setData(*t, *x3);
    customPlot->graph(2)->setPen(QPen(Qt::green));

    customPlot->xAxis->setLabel("time/s");
    //customPlot->xAxis->setAutoTickCount(9);
    customPlot->yAxis->setLabel("angle/(m/s)");
    //customPlot->yAxis->setAutoTickCount(9);
    customPlot->xAxis2->setVisible(true);
    customPlot->xAxis2->setTickLabels(false);
    customPlot->yAxis2->setVisible(true);
    customPlot->yAxis2->setTickLabels(false);
    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));
    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    customPlot->replot();
}

//��ͼ����
void MainWindow::plotUpdate(QCustomPlot *customPlot, double _x1, double _x2, double _x3)
{
    static int num = 0;
    static int _counter = 0;
    double time = (_counter++)*0.1;
    t->append(time);
    x1->append(_x1);
    x2->append(_x2);
    x3->append(_x3);
    if(_counter >= 300){
        t->remove(0);
        x1->remove(0);
        x2->remove(0);
        x3->remove(0);
    }
    customPlot->graph(0)->rescaleAxes(false);
    customPlot->yAxis->setRange(-360, 360);
   // customPlot->xAxis->setRange(time-3, time+1);
    customPlot->graph(0)->setData(*t, *x1);
    customPlot->graph(1)->setData(*t, *x2);
    customPlot->graph(2)->setData(*t, *x3);

    //һ֡�ػ�һ��
    customPlot->replot();

    //ʮ֡�ػ�һ��
//    num++;
//    if(num == 10)
//    {
//        num = 0;
//        customPlot->replot();
//    }
}



void MainWindow::on_altimetroPushButton_clicked()
{
    standardAltimetro->show();
//    altimetro.show();

    ui->statusBar->showMessage(QString::fromLocal8Bit("��ѹ�߶ȱ���ʾ"));
}




