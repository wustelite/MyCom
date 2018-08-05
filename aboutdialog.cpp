#include "aboutdialog.h"
#include "ui_aboutdialog.h"

aboutdialog::aboutdialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::aboutdialog)
{
    ui->setupUi(this);
}

aboutdialog::~aboutdialog()
{
    delete ui;
}

void aboutdialog::changeEvent(QEvent *e)
{
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}


//¹Ø±Õ
void aboutdialog::on_closeBtn_clicked()
{
    this->close();
}
