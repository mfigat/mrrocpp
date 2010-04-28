#ifndef _DOUBLE_WRITER_HH
#define _DOUBLE_WRITER_HH

#include <iostream>

#include <boost/thread/xtime.hpp>

#include "RemoteAgent.hh"
#include "DataBuffer.hh"

class DoubleWriter : public Agent
{
private:
	RemoteAgent reader;
	RemoteBuffer<double> DoubleBuf;
	double cnt;
public:

	DoubleWriter(const std::string & name) :
		Agent(name),
		reader("Reader"),
		DoubleBuf(reader, "double buffer"),
		cnt(0.1)
	{
	}

	bool step()
	{
		std::cout << "Writer: " << cnt << std::endl;
		DoubleBuf.Set(cnt);
		cnt += 1.0;
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		return true;
	}
};

#endif /* _DOUBLE_WRITER_HH */