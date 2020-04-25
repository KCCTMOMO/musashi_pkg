#include "Data.h"

Data::Data()
{
	state = last_state = State::Init;
}

Data::~Data()
{
}

void Data::initTime()
{
	t_start = std::chrono::system_clock::now();
	t_now = std::chrono::system_clock::now();
	dt = 0.0;
	T = 0.0;
}

void Data::updateTime()
{
	std::chrono::system_clock::time_point _tmp;
	_tmp = std::chrono::system_clock::now();

	dt = std::chrono::duration_cast<std::chrono::milliseconds>(_tmp - t_now).count();
	T = std::chrono::duration_cast<std::chrono::milliseconds>(_tmp - t_start).count();

	t_now = _tmp;
}