#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QString>
#include<QMessageBox>
#include<QTime>
#include<QTimer>
#include"qextserial/qextserialport.h"
#include<QFile>
#include<QInputDialog>
#include<QFileDialog>
#include<QDebug>

#include"qcustomplot/qcustomplot.h"
#include"aboutdialog.h"
#include"instrument/altimetro.h"
#include"instrument/qadi.h"
#include"instrument/qcompass.h"

#include<QTextStream>

//mavlink���ͷ�ļ�
//#include"mavlink/v1.0/common/mavlink_msg_gps_raw_int.h"
#include"mavlink/v1.0/checksum.h"
//#include"mavlink/v1.0/mavlink_conversions.h"
//#include"mavlink/v1.0/mavlink_helpers.h"
//#include"mavlink/v1.0/mavlink_types.h"
//#include"mavlink/v1.0/protocol.h"

//��ʱ��TIME_OUT�Ǵ��ڶ�д����ʱ
#define TIME_OUT 10
#define DISPLAY_SIZE 2500

//����mavlink�����Ϣ
#define MAVLINK_MSG_ID_ATTITUDE 30 //��̬����id
#define MAVLINK_MSG_ID_ATTITUDE_LEN 28 //��̬������Ч���ݳ���
#define MAVLINK_MSG_ID_ATTITUDE_CRC 39 //GPS����CRCУ����Կ

#define MAVLINK_MSG_ID_GPS_RAW_INT 24 //GPS����id
#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 30 //GPS������Ч���ݳ���
#define MAVLINK_MSG_ID_GPS_RAW_INT_CRC 24 //GPS����CRCУ����Կ

#define MAVLINK_CORE_HEADER_LEN 5

#define pi 3.1415927f;
#define d2r pi/180.0f
#define r2d 180.0f/pi


//���ڽ������� ֡ͷ
//#define BUFFER_HEAD1    0Xa5
//#define BUFFER_HEAD2    0X5a
//#define BUFFER_HEAD3    0X10
//#define BUFFER_HEAD4    0Xa1
//#define BUFFER_SIZE     18

#define BUFFER_HEAD1 0xFE
#define BUFFER_HEAD6_Att  MAVLINK_MSG_ID_ATTITUDE
#define BUFFER_HEAD6_GPS  MAVLINK_MSG_ID_GPS_RAW_INT
#define BUFFER_SIZE     36

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    aboutdialog aboutdlg;    //����
//    Altimetro   altimetro;   //�߶ȱ�
//    QADI   qadi;

    QextSerialPort *myCom;

    QTimer *myReadTimer;
    QString write2fileName;//д��ȡ�Ĵ������ݵ����ļ�
    //****
    bool isMyComOpen;
    bool isStart;
    QVector<double> *t;
    QVector<double> *x1, *x2, *x3;

    void setupPlot(QCustomPlot *customPlot);
    void plotUpdate(QCustomPlot *customPlot, double _x1, double _x2, double _x3);

    Altimetro *standardAltimetro;       //��ѹ�߶ȱ�   �Ǳ���
    QADI  *standardQADI;
    QCompass *standardQCompass;
////mavlink
    typedef struct
    {
        uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
        float roll; ///< Roll angle (rad, -pi..+pi)
        float pitch; ///< Pitch angle (rad, -pi..+pi)
        float yaw; ///< Yaw angle (rad, -pi..+pi)
        float rollspeed; ///< Roll angular speed (rad/s)
        float pitchspeed; ///< Pitch angular speed (rad/s)
        float yawspeed; ///< Yaw angular speed (rad/s)
    }mavlink_attitude_t; //MavLink��̬���ݽṹ��

    typedef struct
    {
     uint64_t time_usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
     int32_t lat; ///< Latitude in 1E7 degrees
     int32_t lon; ///< Longitude in 1E7 degrees
     int32_t alt; ///< Altitude in 1E3 meters (millimeters) above MSL
     uint16_t eph; ///< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
     uint16_t epv; ///< GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
     uint16_t vel; ///< GPS ground speed (m/s * 100). If unknown, set to: 65535
     uint16_t cog; ///< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
     uint8_t fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
     uint8_t satellites_visible; ///< Number of satellites visible. If unknown, set to 255
    } mavlink_gps_raw_int_t;

    //�������뺯��
    //static uint8_t mavlink_msg_attitude_decode(uint8_t *rec, mavlink_attitude_t *attitude);
    //MavLink��̬���뺯��
    static uint8_t mavlink_msg_attitude_decode(uint8_t *rec, mavlink_attitude_t *attitude)
    {
        uint16_t checksummm = 0;
        const char *p = (const char *)rec;
          //ִ��У��
          checksummm = crc_calculate( rec+1, MAVLINK_CORE_HEADER_LEN);
            crc_accumulate_buffer( &checksummm, (p + 6), *(p + 1) );
            crc_accumulate(MAVLINK_MSG_ID_ATTITUDE_CRC, &checksummm);

          if( (checksummm & 0xFF) == *(p + *(p+1) + 6) && (checksummm >> 8) == *(p + *(p+1) + 7) )//У��ֵͨ�����ܽ���
            {
                memcpy(&attitude, rec + 6, MAVLINK_MSG_ID_ATTITUDE_LEN);
                  return 1;
            }
            else
            {
                return 0;
            }

    }

    //static uint8_t mavlink_msg_gps_raw_int_decode(uint8_t *rec, mavlink_gps_raw_int_t *gps_raw_int);
    //MavLink GPS���뺯��
    static uint8_t mavlink_msg_gps_raw_int_decode(uint8_t *rec, mavlink_gps_raw_int_t *gps_raw_int)
    {
        uint16_t checksummm = 0;
        const char *p = (const char *)rec;
          //ִ��У��
          checksummm = crc_calculate( rec+1, MAVLINK_CORE_HEADER_LEN);
            crc_accumulate_buffer( &checksummm, (p + 6), *(p + 1) );
            crc_accumulate(MAVLINK_MSG_ID_GPS_RAW_INT_CRC, &checksummm);

          if( (checksummm & 0xFF) == *(p + *(p+1) + 6) && (checksummm >> 8) == *(p + *(p+1) + 7) )//У��ֵͨ�����ܽ���
            {
                memcpy(gps_raw_int, rec + 6, MAVLINK_MSG_ID_GPS_RAW_INT_LEN);
                  return 1;
            }
            else
            {
                return 0;
            }
    }

protected:
 //   bool eventFilter(QObject *obj,QEvent *e);
    void changeEvent(QEvent *e);
    void setComboBoxEnabled(bool status);

private slots:
    void on_helpBtn_clicked();
    void on_openMyComBtn_clicked();
    void on_closeMyComBtn_clicked();
    void readMyCom();

    void on_clearUpBtn_clicked();

    void on_actionClearBytes_clicked();

    void on_actionExit_clicked();

    void on_actionSave_clicked();

    void on_actionAbout_triggered();

    void on_altimetroPushButton_clicked();

public:
    void initialSerial();
};

#endif // MAINWINDOW_H










