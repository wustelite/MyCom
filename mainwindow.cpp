#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //setWindowIcon(QIcon(":/new/prefix1/images/logo_10%.png"));
    setWindowTitle(QString::fromLocal8Bit("hellWarrior地面站"));

    //窗口最大化
    setWindowState(Qt::WindowMaximized);
    //初始化串口
    initialSerial();

    //***
    setupPlot(ui->customPlot);

    isMyComOpen = false;
    isStart = false;
    //ui->button_close->setEnabled(false);
    myCom=NULL;
    ui->statusBar->showMessage(QString::fromLocal8Bit("欢迎使用hellWarrior地面站"));

    standardAltimetro = new Altimetro(ui->standardSixAltimetro);    //新建Altimetro对象
    standardQADI = new QADI(ui->standardSixQADI);    //新建QADI对象
    standardQCompass = new QCompass(ui->standardSixQCompass);//新建QCompass对象
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

//change event一般是当前widget状态改变后触发的
//如字体改变、语言改变之类的。
//该方法主要捕获改变事件，当语言改变后，执行相关操作。
void MainWindow::changeEvent(QEvent *e)    //重写的事件处理方法
{
    QMainWindow::changeEvent(e);           //让基类执行事件处理方法
    switch (e->type()) {                   //根据事件类型判别
    case QEvent::LanguageChange:           //如果是语言改变事件
        ui->retranslateUi(this);           //更新UI的语言（看方法字面的，用户自定义方法）
        break;
    default:
        break;
    }
}

void MainWindow::on_helpBtn_clicked()
{
    QMessageBox::about(this,QString::fromLocal8Bit("帮助信息"),QString::fromLocal8Bit(
                       "1、选定好具体串口设置，点击打开串口即可收到消息""\n"
                       "2、作者联系方式（QQ）：344964277""\n"
                       "3、欢迎加入无人机地面站设计开发群（QQ群号：631625639）""\n"));
}

void MainWindow::initialSerial()
{
    //定义串口
    ui->portNameComboBox->addItem("COM4");
    ui->portNameComboBox->addItem("COM1");
    ui->portNameComboBox->addItem("COM2");
    ui->portNameComboBox->addItem("COM3");
    ui->portNameComboBox->addItem("COM5");
    ui->portNameComboBox->addItem("COM6");
    ui->portNameComboBox->addItem("COM7");
    ui->portNameComboBox->addItem("COM8");
    ui->portNameComboBox->addItem("COM9");
    //定义波特率
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
    //定义数据位
    ui->dataBitsComboBox->addItem("8");
    ui->dataBitsComboBox->addItem("5");
    ui->dataBitsComboBox->addItem("6");
    ui->dataBitsComboBox->addItem("7");
    //校核位
    ui->parityComboBox->addItem("none");
    ui->parityComboBox->addItem("odd");
    ui->parityComboBox->addItem("even");
    //停止位
    ui->stopBitsComboBox->addItem("1");
    ui->stopBitsComboBox->addItem("1.5");
    ui->stopBitsComboBox->addItem("2");
    //数据流
    ui->flowTypeComboBox->addItem("off");
    ui->flowTypeComboBox->addItem("hardware");
    ui->flowTypeComboBox->addItem("xonxoff");

    //串口内容选择如何呈现  字符or波形
    ui->ContentSelectComboBox->addItem(QString::fromLocal8Bit("显示波形"));
    ui->ContentSelectComboBox->addItem(QString::fromLocal8Bit("显示字符"));
}

//comboBox控件开关设置
void MainWindow::setComboBoxEnabled(bool status)
{
    ui->portNameComboBox->setEnabled(status);
    ui->baudRateComboBox->setEnabled(status);
    ui->dataBitsComboBox->setEnabled(status);
    ui->stopBitsComboBox->setEnabled(status);
    ui->parityComboBox->setEnabled(status);
    ui->flowTypeComboBox->setEnabled(status);
}

//打开串口开关
void MainWindow::on_openMyComBtn_clicked()
{
    QString portName = ui->portNameComboBox->currentText();   //获取串口名
    myCom = new QextSerialPort(portName);        //定义串口对象，并传递参数，在构造函数里对其进行初始化

    connect(myCom, SIGNAL(readyRead()), this, SLOT(readMyCom()));
    //实现了信号的槽的连接。这样，当串口收到数据以后，就会调readMyCom()函数。

    //注意：得要先打开串口，然后再设置串口的参数，不然设置无效！！！
    if(myCom->open(QIODevice::ReadWrite))
    {
        myCom->flush(); //存入缓冲区内待读取

    //设置波特率
    myCom->setBaudRate((BaudRateType)ui->baudRateComboBox->currentText().toInt());

    //设置数据位
    myCom->setDataBits((DataBitsType)ui->dataBitsComboBox->currentText().toInt());

    //设置校验
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

    //设置停止位
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

    //设置数据流控制
    myCom->setFlowControl(FLOW_OFF);

    //设置延时
    myCom->setTimeout(TIME_OUT);

    QMessageBox::information(this, QString::fromLocal8Bit("打开成功"), QString::fromLocal8Bit("已成功打开串口") + portName, QMessageBox::Ok);

   //界面管理
    ui->openMyComBtn->setEnabled(false);
    ui->closeMyComBtn->setEnabled(true);
    ui->helpBtn->setEnabled(true);
    //控件冻结
    setComboBoxEnabled(false);

    ui->statusBar->showMessage(QString::fromLocal8Bit("打开串口成功！"));
    }

    else
    {
       QMessageBox::critical(this, QString::fromLocal8Bit("打开失败"), QString::fromLocal8Bit("未能打开串口 ") + portName + QString::fromLocal8Bit("\n该串口设备不存在或已被占用"), QMessageBox::Ok);
       return;
    }

//        //定义出具体定时器，然后触发后开始Polling查询
//        myReadTimer = new QTimer(this);
//        myReadTimer->setInterval(10);
//        connect(myReadTimer, SIGNAL(timeout()), this, SLOT(readMyCom()));    //信号和槽函数关联，当串口缓冲区有数据时，进行读串口操作
//        this->myReadTimer->start();         //开始poll查询操作
}

//关闭串口开关
void MainWindow::on_closeMyComBtn_clicked()
{
    myCom->close();
    delete myCom;
    myCom = NULL;

//    this->myReadTimer->stop();          //关闭poll操作
//    myCom->close();
//    model->removeRows(0, model->rowCount());
//    model->setRowCount(row);
//    model->setColumnCount(column);
//    dcFlag = 0;
//    ui->radarWidget->update();  //刷新坐标系，实时显示背景

    ui->openMyComBtn->setEnabled(true);
    ui->helpBtn->setEnabled(true);
    //控件激活
    setComboBoxEnabled(true);

    QMessageBox::information(this, QString::fromLocal8Bit("串口状态："),QString::fromLocal8Bit("关闭"),QMessageBox::Ok);
}


//读取数据
void MainWindow::readMyCom()
{
    static unsigned char buffer[BUFFER_SIZE] = {0};
    static unsigned  char counter = 0;

    if(myCom->bytesAvailable()<=0){return;}  //以免空闲时出现高内存占用

    switch(ui->ContentSelectComboBox->currentIndex()){
    case 0:
    /************************************************************************************************************************************
    *从串口中读出数据并且转化为飞行器姿态数值
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

//MavLink数据结构： magic + len + seq + sysid + compid + msgid(1byte) + payload64(len byte) + checksum(2byte)

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
                                 //姿态信息解码
                                 mavlink_msg_attitude_decode(buffer, &g_MavLink_Att_f);//这里g_Attitude_f如果报错就在其前加个&试试

                                 //弧度转角度
                                 yaw = g_MavLink_Att_f.yaw*r2d;
                                 roll = g_MavLink_Att_f.roll*r2d;
                                 pitch = g_MavLink_Att_f.pitch*r2d;

                                 standardQADI->setData(pitch,roll);
                                 standardQCompass->setData(yaw,Altitude,Altitude);
                                 plotUpdate(ui->customPlot, yaw,Altitude,Altitude);


//                                 standardQADI->setData(roll,pitch);
//                                 //偏航 俯仰 滚转信号示波
//                                 plotUpdate(ui->customPlot, yaw, pitch, roll);
                         break;

                         case BUFFER_HEAD6_GPS:
                                 //GPS信息解码
                                 mavlink_msg_gps_raw_int_decode(buffer,&g_MavLink_GPS_f);//这里g_Attitude_f如果报错就在其前加个&试试

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
//显示帧内容的代码
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
                //如果打开失败则给出提示并退出函数
                if(!file.open(QFile::WriteOnly | QIODevice::Text)){
                    QMessageBox::warning(this, QString::fromLocal8Bit("写入文件"), QString::fromLocal8Bit("打开文件 %1 失败, 无法写入\n%2").arg(write2fileName).arg(file.errorString()), QMessageBox::Ok);
                    return;
                }
                QTextStream out(&file);
                out<<buf;
                file.close();
            }

           //该函数会影响GUI的流畅性
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
                ui->statusBar->showMessage(QString::fromLocal8Bit("成功读取%1字节数据").arg(temp.size()));
       }
    break;
    }
}

//计数器清零
void MainWindow::on_actionClearBytes_clicked()
{
    if(ui->recvbyteslcdNumber->value() == 0){
        QMessageBox::information(this, QString::fromLocal8Bit("提示消息"), QString::fromLocal8Bit("貌似已经清零了呀:)"), QMessageBox::Ok);
    }else{
        ui->recvbyteslcdNumber->display(0);
        ui->statusBar->showMessage(QString::fromLocal8Bit("计数器已经清零"));
    }
}

//清空记录
void MainWindow::on_clearUpBtn_clicked()
{
    ui->textBrowser->clear();
    ui->statusBar->showMessage(QString::fromLocal8Bit("记录已经清空"));
}


//关于  菜单栏
void MainWindow::on_actionAbout_triggered()
{
    aboutdlg.show();
    // 在主窗口中间显示
    int x =this->x() + (this->width() - aboutdlg.width()) / 2;
    int y =this->y() + (this->height() - aboutdlg.height()) / 2;
    aboutdlg.move(x, y);
    ui->statusBar->showMessage(QString::fromLocal8Bit("关于hellWarrior"));
}

//保存textBrowser中的内容
void MainWindow::on_actionSave_clicked()
{

    if(ui->textBrowser->toPlainText().isEmpty()){
        QMessageBox::information(this, "提示消息", QString::fromLocal8Bit("貌似还没有数据! 您需要在发送编辑框中输入要发送的数据"), QMessageBox::Ok);
        return;
    }

    QString filename = QFileDialog::getSaveFileName(this, QString::fromLocal8Bit("保存为"), QString::fromLocal8Bit("未命名.txt"));
    QFile file(filename);
    //如果用户取消了保存则直接退出函数
    if(file.fileName().isEmpty()){
        return;
    }
    //如果打开失败则给出提示并退出函数
    if(!file.open(QFile::WriteOnly | QIODevice::Text)){
        QMessageBox::warning(this, QString::fromLocal8Bit("保存文件"), QString::fromLocal8Bit("打开文件 %1 失败, 无法保存\n%2").arg(filename).arg(file.errorString()), QMessageBox::Ok);
        return;
    }
    //写数据到文件
    QTextStream out(&file);
    out<<ui->textBrowser->toPlainText();
    file.close();
    //修改窗口标题为保存文件路径
    setWindowTitle("saved: " + QFileInfo(filename).canonicalFilePath());


}

//退出程序
void MainWindow::on_actionExit_clicked()
{
    this->close();
}

/************************************************************************************************************************************
*绘图
************************************************************************************************************************************/
//绘图初始化
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

//绘图更新
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

    //一帧重绘一次
    customPlot->replot();

    //十帧重绘一次
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

    ui->statusBar->showMessage(QString::fromLocal8Bit("气压高度表显示"));
}




