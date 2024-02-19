#ifndef _SENSOR_
#define _SENSOR_

class Sensor{
	public:
		Sensor();

		virtual ~Sensor();

		virtual void start() = 0;
		virtual void stop() = 0;
}

#endif
