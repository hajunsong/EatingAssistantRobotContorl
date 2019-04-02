#pragma once

#include <QThread>

class AlgorithmThread : public QThread
{
	Q_OBJECT
public:
	explicit AlgorithmThread(QObject *partent = nullptr);
private:
	void run();
signals:
	void finish();
public slots:
};

