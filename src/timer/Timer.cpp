/*!
@file		Timer.cpp
@brief		functions in CTimer
*/

#include "Timer.h"
#include <iostream>

/*
@brief		finish measuring
@param[in]	flag	show ms on console or not
@retval		duration in ms
*/
int CTimer::Pop(const bool flag)
{
	int duration;

#ifdef _WIN32
	LARGE_INTEGER end, freq;
	QueryPerformanceCounter(&end);
	QueryPerformanceFrequency(&freq);

	duration = int((double)(end.QuadPart - mTasks.back().stime.QuadPart) / float(freq.QuadPart)*1000.0);
#else
	std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
	std::chrono::duration<double> sec = end - mTasks.back().stime;

	duration = int(std::chrono::duration_cast<std::chrono::milliseconds>(sec).count());
#endif

	if (flag){
		std::cout << mTasks.back().name.c_str() << ": " << duration << "ms" << std::endl;
	}

	mTasks.pop_back();

	return duration;
}

/*
@brief		start measuring
@param[in]	name	task name
*/
void CTimer::Push(const std::string &name)
{
	sTask tmp;
	tmp.name = name;

#ifdef _WIN32
	QueryPerformanceCounter(&tmp.stime);
#else
	tmp.stime = std::chrono::system_clock::now();
#endif

	mTasks.push_back(tmp);
}