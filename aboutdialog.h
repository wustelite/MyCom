#ifndef ABOUTDIALOG_H
#define ABOUTDIALOG_H

#include <QDialog>

namespace Ui {
class aboutdialog;
}

class aboutdialog : public QDialog
{
    Q_OBJECT

public:
    explicit aboutdialog(QWidget *parent = 0);
    ~aboutdialog();

protected:
    void changeEvent(QEvent *e);

private slots:
    void on_closeBtn_clicked();

private:
    Ui::aboutdialog *ui;
};

#endif // ABOUTDIALOG_H
