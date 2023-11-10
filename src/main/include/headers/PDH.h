#ifndef _PDH_BOARD_H
#define _PDH_BOARD_H

#include <frc/PowerDistribution.h>

class PDH {
	private:
		inline static PDH* instance{nullptr};
		PDH() = default;
		~PDH() = default;
	
	public:
	PDH(const PDH&) = delete;
	PDH& operator=(const PDH&) = delete;

	static PDH* GetInstance(){
		if ( !instance ){
			instance = new PDH();
		}
		return instance;
	}

	frc::PowerDistribution m_PDH{1, frc::PowerDistribution::ModuleType::kRev};

	double CanWarnings(){
		return m_PDH.GetFaults().CanWarning;
	}

	double Brownouts(){
		return m_PDH.GetFaults().Brownout;
	}

	double GetCurrent(int channel){
		return m_PDH.GetCurrent(channel);
	}

};

#endif	// _PDH_BOARD_H