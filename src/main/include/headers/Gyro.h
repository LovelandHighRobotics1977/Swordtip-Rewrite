#ifndef _GYRO_H
#define _GYRO_H

#include "AHRS.h"
#include "Libraries.h"

class Gyro {
	private:
		inline static Gyro* instance{nullptr};
		Gyro() = default;
		~Gyro() = default;
	
	public:
	Gyro(const Gyro&) = delete;
	Gyro& operator=(const Gyro&) = delete;

	static Gyro* GetInstance(){
		if ( !instance ){
			instance = new Gyro();
		}
		return instance;
	}

	AHRS ahrs{frc::I2C::kMXP};
};

#endif	// _GYRO_H